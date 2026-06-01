/**
 * @file sea_clutter_engine.cpp
 * @brief 海杂波引擎实现（FIR 高斯谱整形方案）
 *
 * 实现 SeaClutterEngine 的核心逻辑，包括：
 * - 初始化：FIR 系数设计 + K 分布 LUT + patch 网格构建 + GPU 状态初始化
 * - 逐 PRT 生成（三种 GPU 并行路径）：
 *   1. 一维距离并行 (get_clutter_for_pulse_timed)
 *   2. 二维 range×patch 并行 (get_clutter_for_pulse_timed_2d)
 *   3. 二维快速路径 (get_clutter_for_pulse_timed_2d_fast)
 * - 活动 patch 窗口构建与俯仰权重管理
 *
 * 每 PRT 工作流程：
 *   counter-based RNG → FIR 高斯谱滤波 → 幅度分布调制
 *   → 天线方向图方位加权叠加 → 俯仰加权 → 输出距离向复杂波
 */

#include "clutter/sea_clutter_engine.h"
#include "core/tools/math_utils.h"

#include <boost/math/distributions/gamma.hpp>
#include <boost/math/distributions/normal.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace radar::clutter
{
    namespace
    {
        using Clock = std::chrono::steady_clock;

        /// @brief 计算两个时间点之间的毫秒间隔
        double elapsed_ms(const Clock::time_point &start, const Clock::time_point &finish)
        {
            return std::chrono::duration<double, std::milli>(finish - start).count();
        }

        /// @brief 将方位角归一化到 [0, 360) 度范围内
        ///
        /// 使用 fmod 进行浮点取模，处理负角度（如 -30° → 330°）和超出 360° 的情况。
        Scalar normalize_azimuth_360(Scalar az_deg)
        {
            Scalar wrapped = std::fmod(az_deg, 360.0f);
            if (wrapped < 0.0f)
            {
                wrapped += 360.0f;
            }
            return wrapped;
        }

        /// @brief 循环索引包装（支持负索引，用于方位单元循环查找）
        ///
        /// 将任意整数 idx 映射到 [0, size-1] 范围内。
        /// 例如 idx=-1, size=360 → 359（即 359° 方位角）。
        int wrap_index(int idx, int size)
        {
            if (size <= 0)
            {
                return 0;
            }
            idx %= size;
            if (idx < 0)
            {
                idx += size;
            }
            return idx;
        }

        /// @brief 安全转换 int → size_t，负数映射为 0
        std::size_t to_size(int value)
        {
            return static_cast<std::size_t>(std::max(value, 0));
        }

        /// @brief 构建 K 分布 sqrt(tau) 查找表（CPU 端一次性预计算）
        ///
        /// K 分布的纹理分量 tau 服从 Gamma(ν, 1/ν) 分布。
        /// GPU 端通过 counter-based RNG 生成标准正态随机数 z，
        /// 再通过本 LUT 将 z 映射为 sqrt(tau)，避免了 GPU 端昂贵的
        /// Gamma CDF 逆函数计算。
        ///
        /// 映射方法（利用 CDF 变换法）：
        /// 1. 将标准正态 z ∈ [-8, 8] 均匀采样 lut_size 个点
        /// 2. 通过正态 CDF Φ(z) 映射到均匀分布 u ∈ (0, 1)
        /// 3. 通过 Gamma CDF 逆函数 F⁻¹(u) 映射 u → tau
        /// 4. 存储 sqrt(tau)，供 GPU kernel 线性插值查询
        ///
        /// z 的 ±8 范围覆盖了正态分布约 6.2×10⁻¹⁶ 的尾概率，足够精度。
        ///
        /// @param shape_nu Gamma 分布形状参数 ν，E[tau] = 1
        /// @param lut_size 查找表长度（越大插值精度越高）
        std::vector<Scalar> build_k_sqrt_tau_lut(Scalar shape_nu, int lut_size)
        {
            lut_size = std::max(16, lut_size);
            const double nu = std::max(static_cast<double>(shape_nu), 1.0e-9);
            boost::math::normal_distribution<double> normal(0.0, 1.0);
            boost::math::gamma_distribution<double> gamma(nu, 1.0 / nu);

            std::vector<Scalar> lut(static_cast<std::size_t>(lut_size));
            for (int i = 0; i < lut_size; ++i)
            {
                // z 在 [-8, 8] 均匀采样
                const double z = -8.0 + 16.0 * static_cast<double>(i) /
                                           static_cast<double>(lut_size - 1);
                // 正态 CDF 映射：z → u ∈ (0, 1)
                const double u = std::clamp(boost::math::cdf(normal, z),
                                            1.0e-15,
                                            1.0 - 1.0e-15);
                // Gamma CDF 逆函数映射：u → tau
                const double tau = std::max(0.0, boost::math::quantile(gamma, u));
                // 存储 sqrt(tau)
                lut[static_cast<std::size_t>(i)] = static_cast<Scalar>(std::sqrt(tau));
            }
            return lut;
        }
    } // namespace

    // =========================================================================
    // 初始化
    // =========================================================================

    SeaClutterEngine::~SeaClutterEngine()
    {
        // 释放 GPU 资源（安全可重复调用）
        cuda::release_clutter_gpu_state(gpu_state_);
    }

    bool SeaClutterEngine::initialize(const RadarSystemParams &system)
    {
        if (initialized_)
        {
            return true; 
        }

        // ---- 步骤 1: 验证配置参数 ----
        std::string error;
        if (!cfg_.validate(error))
        {
            last_error_ = error;
            return false;
        }

        // ---- 步骤 2: 注入配置到内部模型 ----
        sys_ = system;
        model_.set_config(cfg_);
        model_.set_system_params(sys_);
        model_.set_antenna_config(ant_cfg_);

        // 未启用时仅设置基本几何参数，不分配 GPU 资源
        if (!cfg_.enabled)
        {
            num_range_cells_ = std::max(1, sys_.samples_per_pulse);
            delta_r_m_ = math::clamp_positive_eps(sys_.range_bin_size_m);
            initialized_ = true;
            last_error_.clear();
            return true;
        }

        // ---- 步骤 3: 多普勒谱宽（直接使用配置值） ----
        sigma_f_hz_ = cfg_.doppler_sigma_hz;

        // ---- 步骤 4: 设计高斯谱 FIR 滤波器 ----
        // 调用 SeaClutterModel 在 CPU 端预计算 FIR 系数 h[k]
        fir_coeffs_ = model_.get_fir_coefficients(sigma_f_hz_,cfg_.doppler_center_hz,
                                                   sys_.prf_hz,cfg_.fir_length);

        if (fir_coeffs_.empty())
        {
            last_error_ = "FIR filter design failed";
            return false;
        }

        // ---- 步骤 5: K 分布参数预计算 ----
        k_sqrt_tau_lut_.clear();
        k_texture_rho_ = 0.0f;
        k_texture_innovation_scale_ = 0.0f;
        if (cfg_.distribution == ClutterDistribution::K)
        {
            // 5a. 构建 sqrt(tau) 查找表（CPU 端一次性预计算，后续 GPU 直接使用）
            k_sqrt_tau_lut_ = build_k_sqrt_tau_lut(cfg_.k_shape_nu, cfg_.k_lut_size);

            // 5b. 预计算 AR(1) 纹理过程参数
            // ρ = exp(-2π × BW_texture / PRF)，单 PRT 相关系数
            const Scalar safe_prf = math::clamp_positive_eps(sys_.prf_hz);
            const Scalar safe_bw = math::clamp_positive_eps(cfg_.k_texture_bandwidth_hz);
            k_texture_rho_ = std::exp(-2.0f * PI * safe_bw / safe_prf);

            // σ_innov = sqrt(1 - ρ²)，连续 PRT 的创新项标准差
            // 在 GPU fast 路径中直接使用，避免重复计算
            k_texture_innovation_scale_ =
                std::sqrt(std::max(1.0f - k_texture_rho_ * k_texture_rho_, 0.0f));
        }

        // ---- 步骤 6: 初始化静态二维 patch 幅度图 ----
        // 每个 (az, range) patch 的 sqrt(接收功率)，不含天线方向图权重
        if (!initialize_patch_grid())
        {
            return false;
        }

        // ---- 步骤 7: 初始化 GPU 状态 ----
        // 上传 FIR 系数、patch 幅度图、俯仰权重、K 分布 LUT 等到 GPU 显存
        std::string gpu_error;
        if (!cuda::initialize_clutter_gpu_state(gpu_state_,
                                                fir_coeffs_.data(),
                                                (fir_coeffs_.size()),
                                                patch_amplitude_map_.data(),
                                                el_amplitude_weight_.data(),
                                                num_range_cells_,
                                                num_az_cells_,
                                                num_az_cells_,
                                                k_sqrt_tau_lut_.empty() ? nullptr : k_sqrt_tau_lut_.data(),
                                                k_sqrt_tau_lut_.size(),
                                                &gpu_error))
        {
            last_error_ = "SeaClutter GPU initialization failed: " + gpu_error;
            return false;
        }

        initialized_ = true;
        last_error_.clear();

        SPDLOG_INFO("SeaClutterEngine initialized: {} range cells × {} az cells, "
                    "FIR length={}, sigma_f={:.1f} Hz",
                    num_range_cells_, num_az_cells_,
                    fir_coeffs_.size(), sigma_f_hz_);

        return true;
    }

    // =========================================================================
    // 核心接口：逐 PRT 获取杂波回波
    // =========================================================================

    bool SeaClutterEngine::get_clutter_for_pulse(const BeamPoint &beam,
                                                  std::uint64_t pulse_index,
                                                  PulseEcho &out_echo)
    {
        // 委托给带计时版本，不收集性能数据
        return get_clutter_for_pulse_timed(beam, pulse_index, out_echo, nullptr);
    }

    bool SeaClutterEngine::get_clutter_for_pulse_timed(const BeamPoint &beam,
                                                       std::uint64_t pulse_index,
                                                       PulseEcho &out_echo,
                                                       SeaClutterTiming *timing)
    {
        if (timing != nullptr)
        {
            *timing = SeaClutterTiming{};
        }

        // 未启用时返回零向量
        if (!cfg_.enabled)
        {
            out_echo.assign(num_range_cells_, Complex(0.0, 0.0));
            return true;
        }

        if (!initialized_)
        {
            last_error_ = "SeaClutterEngine not initialized";
            return false;
        }
        // 确保俯仰幅度权重与当前波束俯仰角匹配
        if (!ensure_elevation_amplitude_weight(beam.elevation_deg))
        {
            return false;
        }

        // 初始化输出为全零
        out_echo.assign(num_range_cells_, Complex(0.0, 0.0));

        // ---- 构建活动方位 patch 列表 ----
        const auto active_start = Clock::now();
        const int active_count = build_active_patch_list(beam.azimuth_deg, active_patches_);
        const auto active_finish = Clock::now();
        last_active_count_ = active_count;
        if (timing != nullptr)
        {
            timing->active_build_ms = elapsed_ms(active_start, active_finish);
        }

        // 无活跃 patch 时直接返回
        if (active_count <= 0 || num_range_cells_ <= 0 || num_az_cells_ <= 0)
        {
            return true;
        }

        // ---- 组装 GPU 参数 ----

        cuda::ClutterPrtParams gpu_params{};
        gpu_params.pulse_index = pulse_index > 0ULL ? pulse_index : 0;
        gpu_params.seed = cfg_.seed;
        gpu_params.peak_amplitude_weight = model_.peak_amplitude_weight();
        gpu_params.weibull_shape = cfg_.weibull_shape;
        gpu_params.weibull_scale = cfg_.weibull_scale;
        gpu_params.lognormal_mu = cfg_.lognormal_mu;
        gpu_params.lognormal_sigma = cfg_.lognormal_sigma;
        gpu_params.k_texture_rho = k_texture_rho_;
        gpu_params.k_texture_innovation_scale = k_texture_innovation_scale_;
        gpu_params.num_range_cells = num_range_cells_;
        gpu_params.active_count = active_count;
        gpu_params.k_lut_size = k_sqrt_tau_lut_.size();
        gpu_params.distribution = cfg_.distribution;

        // ---- 调用 GPU 一维距离并行 kernel ----
        std::string gpu_error;
        if (!cuda::generate_clutter_prt_gpu(gpu_state_,
                                            gpu_params,
                                            active_patches_.data(),
                                            out_echo.data(),
                                            timing != nullptr ? &timing->gpu : nullptr,
                                            &gpu_error))
        {
            last_error_ = "SeaClutter GPU generation failed: " + gpu_error;
            return false;
        }

        return true;
    }

    bool SeaClutterEngine::get_clutter_for_pulse_timed_2d(const BeamPoint &beam,
                                                          std::uint64_t pulse_index,
                                                          PulseEcho &out_echo,
                                                          SeaClutterTiming *timing)
    {
        // ---- 重置计时结构 ----
        if (timing != nullptr)
        {
            *timing = SeaClutterTiming{};
        }

        // 未启用时返回零向量
        if (!cfg_.enabled)
        {
            out_echo.assign(num_range_cells_, Complex(0.0, 0.0));
            return true;
        }

        if (!initialized_)
        {
            last_error_ = "SeaClutterEngine not initialized";
            return false;
        }
        // 确保俯仰幅度权重与当前波束俯仰角匹配
        if (!ensure_elevation_amplitude_weight(beam.elevation_deg))
        {
            return false;
        }

        // 初始化输出为全零
        out_echo.assign(num_range_cells_, Complex(0.0, 0.0));

        // ---- 构建活动方位 patch 列表 ----
        const auto active_start = Clock::now();
        const int active_count = build_active_patch_list(beam.azimuth_deg, active_patches_);
        const auto active_finish = Clock::now();
        last_active_count_ = active_count;
        if (timing != nullptr)
        {
            timing->active_build_ms = elapsed_ms(active_start, active_finish);
        }

        // 无活跃 patch 时直接返回（零向量）
        if (active_count <= 0 || num_range_cells_ <= 0 || num_az_cells_ <= 0)
        {
            return true;
        }

        // ---- 组装 GPU 参数 ----

        cuda::ClutterPrtParams gpu_params{};
        gpu_params.pulse_index =  pulse_index > 0ULL ? pulse_index : 0;
        gpu_params.seed = cfg_.seed;
        gpu_params.peak_amplitude_weight = model_.peak_amplitude_weight();
        gpu_params.weibull_shape = cfg_.weibull_shape;
        gpu_params.weibull_scale = cfg_.weibull_scale;
        gpu_params.lognormal_mu = cfg_.lognormal_mu;
        gpu_params.lognormal_sigma = cfg_.lognormal_sigma;
        gpu_params.k_texture_rho = k_texture_rho_;
        gpu_params.k_texture_innovation_scale = k_texture_innovation_scale_;
        gpu_params.num_range_cells = num_range_cells_;
        gpu_params.active_count = active_count;
        gpu_params.k_lut_size = k_sqrt_tau_lut_.size();
        gpu_params.distribution = cfg_.distribution;

        // ---- 调用 GPU 二维 range×patch 并行 kernel ----
        // 每个线程处理一个 (range, patch) 对，通过原子加跨 patch 归约到距离单元
        std::string gpu_error;
        if (!cuda::generate_clutter_prt_gpu_2d(gpu_state_,
                                               gpu_params,
                                               active_patches_.data(),
                                               out_echo.data(),
                                               timing != nullptr ? &timing->gpu : nullptr,
                                               &gpu_error))
        {
            last_error_ = "SeaClutter GPU 2D generation failed: " + gpu_error;
            return false;
        }

        return true;
    }

    bool SeaClutterEngine::get_clutter_for_pulse_timed_2d_fast(const BeamPoint &beam,
                                                               std::uint64_t pulse_index,
                                                               PulseEcho &out_echo,
                                                               SeaClutterTiming *timing)
    {
        // ---- 重置计时结构 ----
        if (timing != nullptr)
        {
            *timing = SeaClutterTiming{};
        }

        // 未启用时返回零向量
        if (!cfg_.enabled)
        {
            out_echo.assign(num_range_cells_, Complex(0.0, 0.0));
            return true;
        }

        if (!initialized_)
        {
            last_error_ = "SeaClutterEngine not initialized";
            return false;
        }
        // 确保俯仰幅度权重与当前波束俯仰角匹配
        if (!ensure_elevation_amplitude_weight(beam.elevation_deg))
        {
            return false;
        }

        // 初始化输出为全零
        out_echo.assign(num_range_cells_, Complex(0.0, 0.0));

        // ---- 构建活动方位 patch 列表 ----
        const auto active_start = Clock::now();
        const int active_count = build_active_patch_list(beam.azimuth_deg, active_patches_);
        const auto active_finish = Clock::now();
        last_active_count_ = active_count;
        if (timing != nullptr)
        {
            timing->active_build_ms = elapsed_ms(active_start, active_finish);
        }

        // 无活跃 patch 时直接返回（零向量）
        if (active_count <= 0 || num_range_cells_ <= 0 || num_az_cells_ <= 0)
        {
            return true;
        }

        // ---- 组装 GPU 参数 ----
        
        cuda::ClutterPrtParams gpu_params{};
        gpu_params.pulse_index =  pulse_index > 0ULL ? pulse_index : 0;
        gpu_params.seed = cfg_.seed;
        gpu_params.peak_amplitude_weight = model_.peak_amplitude_weight();
        gpu_params.weibull_shape = cfg_.weibull_shape;
        gpu_params.weibull_scale = cfg_.weibull_scale;
        gpu_params.lognormal_mu = cfg_.lognormal_mu;
        gpu_params.lognormal_sigma = cfg_.lognormal_sigma;
        gpu_params.k_texture_rho = k_texture_rho_;
        gpu_params.k_texture_innovation_scale = k_texture_innovation_scale_;
        gpu_params.num_range_cells = num_range_cells_;
        gpu_params.active_count = active_count;
        gpu_params.k_lut_size = k_sqrt_tau_lut_.size();
        gpu_params.distribution = cfg_.distribution;

        // ---- 调用 GPU 二维快速 kernel ----
        // 在 2D 基础上增加 K 分布连续 PRT texture 快路径：
        // ZMNL方法跳过 AR(1) 纹理递推；K 分布时 delta==1 直接用缓存的 texture 递推
        std::string gpu_error;
        if (!cuda::generate_clutter_prt_gpu_2d_fast(gpu_state_,
                                                    gpu_params,
                                                    active_patches_.data(),
                                                    out_echo.data(),
                                                    timing != nullptr ? &timing->gpu : nullptr,
                                                    &gpu_error))
        {
            last_error_ = "SeaClutter GPU 2D fast generation failed: " + gpu_error;
            return false;
        }

        return true;
    }

    // =========================================================================
    // 内部方法
    // =========================================================================

    bool SeaClutterEngine::initialize_patch_grid()
    {
        // ---- 委托 SeaClutterModel 构建二维 patch 幅度图 ----
        // 包括 Morchin σ0 → 雷达方程功率 → sqrt(Pc) 的完整计算链
        SeaClutterPatchGrid grid = model_.build_patch_grid();
        num_range_cells_ = grid.num_range_cells;
        num_az_cells_ = grid.num_az_cells;
        delta_r_m_ = grid.delta_r_m;
        delta_az_deg_ = grid.delta_az_deg;
        delta_az_rad_ = grid.delta_az_rad;
        patch_amplitude_map_ = std::move(grid.patch_amplitude_map);
        range_elevation_deg_ = std::move(grid.range_elevation_deg);
        el_amplitude_weight_ = std::move(grid.el_amplitude_weight);
        const std::size_t range_count = to_size(num_range_cells_);
        const std::size_t az_count = to_size(num_az_cells_);

        // ---- 校验数组维度一致性 ----
        // patch_amplitude_map_: [az][range] 布局，总元素数应为 az_count × range_count
        // range_elevation_deg_ / el_amplitude_weight_: [range] 布局，长度应为 range_count
        if (patch_amplitude_map_.size() != range_count * az_count ||
            range_elevation_deg_.size() != range_count ||
            el_amplitude_weight_.size() != range_count)
        {
            last_error_ = "SeaClutterModel returned inconsistent patch grid sizes";
            return false;
        }

        // 记录初始俯仰角（使用天线配置中的默认波束俯仰角），标记权重缓存为有效
        cached_beam_el_deg_ = ant_cfg_.el_beam_point_deg;
        el_amplitude_weight_valid_ = true;
        // 预分配活跃 patch 缓冲区，避免逐 PRT 重复分配
        active_patches_.reserve(az_count);

        return true;
    }
    /// @brief 波束俯仰角改变时重新计算俯仰幅度权重
    ///
    /// 委托 SeaClutterModel 根据新的波束俯仰角计算每个距离单元的俯仰方向图归一化增益，
    /// 并更新 cached_beam_el_deg_ 缓存。
    void SeaClutterEngine::build_elevation_amplitude_weight(Scalar beam_el_deg)
    {
        // 委托 SeaClutterModel 计算俯仰方向图权重，并更新缓存
        el_amplitude_weight_ =
            model_.build_elevation_amplitude_weight(range_elevation_deg_, beam_el_deg);
        cached_beam_el_deg_ = beam_el_deg;
        el_amplitude_weight_valid_ = true;
    }

    /// @brief 确保俯仰幅度权重与当前波束俯仰角匹配
    ///
    /// 若缓存的俯仰权重与当前波束俯仰角相同（误差 ≤ EPSILON），直接复用；
    /// 否则重新计算并上传到 GPU 显存。
    /// 此缓存策略避免连续 PRT 俯仰角不变时重复计算和 GPU 上传。
    bool SeaClutterEngine::ensure_elevation_amplitude_weight(Scalar beam_el_deg)
    {
        // 缓存命中：俯仰角未变化，直接复用已有权重
        if (el_amplitude_weight_valid_ &&
            std::abs(cached_beam_el_deg_ - beam_el_deg) <= EPSILON)
        {
            return true;
        }

        // 缓存未命中：重新计算俯仰权重并上传到 GPU
        build_elevation_amplitude_weight(beam_el_deg);
        std::string gpu_error;
        if (!cuda::update_clutter_gpu_elevation_amplitude_weight(
                gpu_state_,
                el_amplitude_weight_.data(),
                num_range_cells_,
                &gpu_error))
        {
            last_error_ = "SeaClutter elevation gain update failed: " + gpu_error;
            return false;
        }
        return true;
    }

    /// @brief 确定当前波束覆盖的活动方位 patch 窗口
    ///
    /// 流程：
    /// 1. 将波束方位角归一化到 [0, 360) 度
    /// 2. 计算波束中心对应的方位单元索引
    /// 3. 根据 active_gain_floor_db 门限反解最大偏移角：
    ///    G(offset) = exp(-4·ln2 × (offset/BW)²) ≥ gain_floor_linear
    ///    → max_offset = BW × sqrt(-ln(gain_floor) / (4·ln2))
    /// 4. 窗口内逐个 patch 计算方位方向图归一化权重
    /// 5. 仅保留权重 ≥ gain_floor 的 patch 作为活跃 patch
    ///
    /// 特殊处理：当窗口覆盖全周（波束极宽或门限极低）时，直接遍历所有方位单元。
    ///
    /// @param beam_az_deg 波束方位角（度）
    /// @param[out] active_patches 输出活跃 patch 列表（复用缓冲区）
    /// @return 活跃 patch 数量
    int SeaClutterEngine::build_active_patch_list(
        Scalar beam_az_deg,
        std::vector<cuda::ClutterActivePatch> &active_patches) const
    {
        active_patches.clear();
        if (num_az_cells_ <= 0 || delta_az_deg_ <= 0.0f)
        {
            return 0;
        }

        // ---- 1. 确定波束中心方位单元 ----
        const Scalar beam_az_norm = normalize_azimuth_360(beam_az_deg);
        const int center_cell = wrap_index(
            static_cast<int>(std::lround(beam_az_norm / delta_az_deg_)),
            num_az_cells_);

        const Scalar az_beamwidth_deg = math::clamp_positive_eps(ant_cfg_.az_beamwidth_deg);
        const Scalar gain_floor_linear = math::db_to_linear(cfg_.active_gain_floor_db);

        // 由高斯方向图反解最大偏移角：
        // exp(-4·ln2 × (offset/BW)²) = gain_floor → offset = BW × sqrt(-ln(gain_floor) / (4·ln2))
        const Scalar max_offset_deg =
            az_beamwidth_deg * std::sqrt(std::max(0.0f, -std::log(gain_floor_linear) / (4.0f * kLog2)));
        int half_window = static_cast<int>(std::ceil(max_offset_deg / delta_az_deg_));
        half_window = std::min(half_window, num_az_cells_ / 2); // 不超过半圆周

        // 窗口是否覆盖全周（波束极宽 或 门限极低时）
        const bool full_azimuth_window = (2 * half_window + 1) >= num_az_cells_;
        const int first_offset = full_azimuth_window ? 0 : -half_window;
        const int last_offset = full_azimuth_window ? (num_az_cells_ - 1) : half_window;
        active_patches.reserve(static_cast<std::size_t>(last_offset - first_offset + 1));

        // ---- 3. 窗口内逐 patch 筛选活跃补丁 ----
        for (int offset = first_offset; offset <= last_offset; ++offset)
        {
            const int az_idx = full_azimuth_window
                                   ? offset // 全周窗口：直接遍历 0..N-1
                                   : wrap_index(center_cell + offset, num_az_cells_);
            const Scalar patch_az_deg = static_cast<Scalar>(az_idx) * delta_az_deg_;
            const Scalar az_amplitude_weight =
                model_.normalized_az_amplitude(patch_az_deg, beam_az_norm);

            // 仅保留权重 ≥ 门限的 patch
            if (az_amplitude_weight >= gain_floor_linear)
            {
                active_patches.push_back(cuda::ClutterActivePatch{az_idx, az_amplitude_weight});
            }
        }

        return static_cast<int>(active_patches.size());
    }

} // namespace radar::clutter
