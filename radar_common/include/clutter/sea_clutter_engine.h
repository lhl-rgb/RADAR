/**
 * @file sea_clutter_engine.h
 * @brief 海杂波引擎 — 逐 PRT 生成 FIR 高斯谱杂波回波
 *
 * 设计说明（配置-初始化-运行三阶段模式）：
 *
 * 【配置阶段】
 *   通过 set_config() / set_antenna_config() 注入杂波参数和天线方向图配置。
 *
 * 【初始化阶段】
 *   initialize(system) — 验证参数，执行以下预计算：
 *     a. 读取多普勒谱宽 doppler_sigma_hz，设计高斯谱 FIR 滤波器系数 h[k]
 *     b. 构建距离-方位二维 patch 幅度网格：
 *        - 逐距离单元计算掠射角 → Morchin σ0 → patch 面积 → 雷达方程接收功率
 *        - 均匀海面假设下，距离向剖面沿方位复制
 *     c. 初始化天线方向图模型（方位 + 俯仰）
 *     d. 若启用 K 分布，构建 sqrt(tau) LUT + 预计算 AR(1) 参数
 *     e. 上传所有静态数据至 GPU 显存
 *
 * 【每个 PRT 阶段】
 *   get_clutter_for_pulse(beam, pulse_index, out_echo):
 *     1. 根据波束方位角和 active_gain_floor_db 门限，确定活动方位 patch 窗口
 *     2. 对窗口内每个 (range, active_az) patch：
 *        a. counter-based RNG 生成历史白噪声 w[range,az,pulse_index - k]
 *        b. FIR 滤波：c = sqrt(P_patch) × Σ h[k]·w[range,az,pulse_index - k]
 *        c. 按分布类型施加幅度调制（Rayleigh/Weibull/LogNormal/K）
 *     3. 天线方向图方位加权叠加：z[range] = Σ az_weight(θ-θ_beam)·c[range,θ]
 *     4. 俯仰方向图幅度加权：z[range] *= el_weight(range)
 *     5. 写入 out_echo
 *
 * 数据流：
 *   白复高斯噪声 → FIR 高斯谱整形 → patch 级杂波序列
 *       → 幅度分布调制 → 天线方向图方位加权叠加 → 俯仰加权
 *       → 单 PRT 距离向复杂波回波
 *
 * GPU 并行策略（三种路径）：
 *   - 一维距离并行 (get_clutter_for_pulse_timed)：
 *     每个 CUDA 线程处理一个距离单元，沿活动方位累加
 *   - 二维 range×patch 并行 (get_clutter_for_pulse_timed_2d)：
 *     每个线程处理一个 (range, patch) 对，最后跨 patch 归约
 *   - 二维快速路径 (get_clutter_for_pulse_timed_2d_fast)：
 *     在 2D 基础上增加 K 分布连续 PRT texture 快路径优化
 *   - 流式批量路径 (generate_clutter_sequence_2d_fast_stream)：
 *     双缓冲异步 CUDA Stream 流水线，GPU 计算与 CPU 准备重叠
 */

#pragma once

#include "antenna/antenna_config.h"
#include "clutter/sea_clutter_config.h"
#include "clutter/sea_clutter_model.h"
#include "cuda/clutter_gpu.cuh"
#include "core/radar_system_params.h"
#include "core/types.h"

#include <cstdint>
#include <string>
#include <vector>

namespace radar::clutter
{
    /**
     * @brief 单次 PRT 杂波生成性能计时
     *
     * 记录 CPU 端 active patch 筛选耗时和 GPU 端各阶段耗时。
     * 由 get_clutter_for_pulse_timed 系列接口填充。
     */
    struct SeaClutterTiming
    {
        double active_build_ms = 0.0;  ///< CPU 端构建 active patch 列表耗时 (ms)
        cuda::ClutterGpuTiming gpu;    ///< GPU 端各阶段耗时
    };

    /**
     * @brief 流式批量 PRT 杂波生成性能计时
     *
     * 汇总一轮 generate_clutter_sequence_2d_fast_stream 中各环节累计耗时，
     * 包括整轮总耗时、active patch 构建累计、GPU wait 累计、格式转换累计，
     * 以及 active patch 数量的统计值（总和、最小值、最大值）。
     */
    struct SeaClutterStreamTiming
    {
        double run_ms = 0.0;               ///< 整轮运行总耗时 (ms)
        double active_build_ms_sum = 0.0;  ///< 各 PRT active patch 构建累计耗时 (ms)
        double gpu_wait_ms_sum = 0.0;      ///< 各 slot GPU wait 累计耗时 (ms)
        double host_convert_ms_sum = 0.0;  ///< 各 slot 主机端格式转换累计耗时 (ms)
        double active_count_sum = 0.0;     ///< 各 PRT active patch 数量累计
        int active_count_min = 0;          ///< 本轮 active patch 数最小值
        int active_count_max = 0;          ///< 本轮 active patch 数最大值
    };

    /**
     * @brief 海杂波引擎
     *
     * 对外提供逐 PRT 和批量 PRT 的杂波生成接口。
     * 内部持有 patch 幅度网格、FIR 系数、K 分布 LUT 和 GPU 运行状态，
     * 实现 FIR 高斯谱整形 + 幅度分布调制 + 机扫天线方向图加权。
     *
     * 生命周期：
     *   1. 构造 → 2. set_config / set_antenna_config → 3. initialize
     *      → 4. 反复调用 get_clutter_for_pulse (或 stream 接口)
     *      → 5. 析构（自动释放 GPU 资源）
     */
    class SeaClutterEngine
    {
    public:
        SeaClutterEngine() = default;
        ~SeaClutterEngine();

        // 禁止拷贝
        SeaClutterEngine(const SeaClutterEngine &) = delete;
        SeaClutterEngine &operator=(const SeaClutterEngine &) = delete;

        // =====================================================================
        // 配置注入
        // =====================================================================

        /**
         * @brief 设置杂波配置参数
         *
         * 必须在 initialize() 之前调用。配置会以值拷贝方式保存，
         * 后续可通过 config() 查询当前值。
         */
        void set_config(const SeaClutterConfig &cfg) { cfg_ = cfg; }

        /**
         * @brief 设置天线配置（用于方向图加权）
         *
         * 必须在 initialize() 之前调用。
         * 天线方向图模型在 initialize() 中完成初始化和参数固化。
         */
        void set_antenna_config(const antenna::AntennaConfig &cfg) { ant_cfg_ = cfg; }

        // =====================================================================
        // 初始化
        // =====================================================================

        /**
         * @brief 初始化杂波引擎
         * @param system 雷达系统参数（PRF、波长、功率、天线高度等）
         * @return 成功返回 true，失败时可通过 last_error() 获取错误信息
         *
         * 内部完成：
         * - 验证配置参数合法性
         * - 设计高斯谱 FIR 滤波器系数（调用 SeaClutterModel）
         * - 若为 K 分布：构建 sqrt(tau) LUT + 预计算 AR(1) 相关系数
         * - 构建距离-方位二维 patch 幅度网格（Morchin + 雷达方程）
         * - 初始化天线方向图模型 + 俯仰幅度权重
         * - 上传 FIR 系数、patch 幅度图、俯仰权重至 GPU
         *
         * 若 cfg_.enabled = false，仅设置基本几何参数，不分配 GPU 资源。
         */
        bool initialize(const RadarSystemParams &system);

        /**
         * @brief 是否已初始化成功
         */
        bool is_initialized() const { return initialized_; }

        /**
         * @brief 是否启用（已初始化 且 config().enabled = true）
         */
        bool is_enabled() const { return cfg_.enabled && initialized_; }

        /**
         * @brief 最近一次错误的描述信息
         *
         * 在 initialize() 或各 generate 接口返回 false 后调用，
         * 可获取详细的失败原因。
         */
        const std::string &last_error() const { return last_error_; }

        // =====================================================================
        // 核心接口：逐 PRT 获取杂波回波
        // =====================================================================

        /**
         * @brief 获取指定波束方位的单脉冲海杂波
         * @param beam          当前波束指向（方位角 + 俯仰角，度）
         * @param pulse_index   脉冲序号（从 0 开始递增，用于 counter-based RNG）
         * @param out_echo      输出：距离向杂波复回波（长度为 num_range_cells，I/Q 分量）
         * @return 成功返回 true
         *
         * 每个 PRT 调用一次，内部使用 GPU 一维距离并行路径完成：
         * 1. 确定波束窗口内的活动方位 patch（基于 active_gain_floor_db 门限）
         * 2. counter-based RNG 生成历史白噪声 + FIR 高斯谱滤波
         * 3. 按配置的幅度分布类型施加调制（Rayleigh/Weibull/LogNormal/K）
         * 4. 天线方向图方位加权叠加 + 俯仰幅度加权
         * 5. 输出距离向复杂波回波
         *
         * 未启用时直接返回 true，out_echo 填零。
         */
        bool get_clutter_for_pulse(const BeamPoint &beam,
                                   std::uint64_t pulse_index,
                                   PulseEcho &out_echo);

        /**
         * @brief 获取单脉冲海杂波（带性能计时，一维距离并行路径）
         *
         * 与 get_clutter_for_pulse 功能一致，额外通过 timing 输出各阶段耗时。
         * 使用 GPU 一维距离并行 kernel：每个 CUDA 线程处理一个距离单元，
         * 沿活动方位 patch 循环累加。
         */
        bool get_clutter_for_pulse_timed(const BeamPoint &beam,
                                         std::uint64_t pulse_index,
                                         PulseEcho &out_echo,
                                         SeaClutterTiming *timing);

        /**
         * @brief 获取单脉冲海杂波（二维 range×patch 并行路径，带计时）
         *
         * 每个 CUDA 线程处理一个 (range, patch) 对，
         * 各 patch 独立计算贡献，最后通过原子加跨 patch 归约到每个距离单元。
         * 适用于 patch 数量较多时，利用 GPU 大规模并行度。
         */
        bool get_clutter_for_pulse_timed_2d(const BeamPoint &beam,
                                            std::uint64_t pulse_index,
                                            PulseEcho &out_echo,
                                            SeaClutterTiming *timing);

        /**
         * @brief 获取单脉冲海杂波（二维快速路径，带计时）
         *
         * 在二维 range×patch 基础上增加 K 分布连续 PRT texture 快路径优化。
         * 当 distribution == Rayleigh 时，跳过 AR(1) 纹理递推；
         * 当 distribution == K 时，delta == 1（相邻脉冲）跳过完整 AR(1) 递推，
         * 直接使用缓存的上一次 texture 值进行一步递推。
         */
        bool get_clutter_for_pulse_timed_2d_fast(const BeamPoint &beam,
                                                 std::uint64_t pulse_index,
                                                 PulseEcho &out_echo,
                                                 SeaClutterTiming *timing);

        /**
         * @brief 流式批量生成一段连续 PRT 的海杂波序列
         *
         * 使用 CUDA 异步双缓冲流管线（pipeline 深度为 2）：
         * - slot 0 执行 GPU kernel 期间，slot 1 准备下一 PRT 数据
         * - 交替推进，实现 GPU 计算、数据传输与 CPU 准备的流水线重叠
         *
         * 与单 PRT 接口不同，此方法要求每个 PRT 至少有一个活跃 patch，
         * 否则直接返回失败（stream 路径不支持零 patch 情况）。
         *
         * @param beams            各 PRT 对应的波束指向数组（长度 pulse_count）
         * @param start_pulse_index 起始脉冲编号
         * @param pulse_count       脉冲数量
         * @param[out] checksum     输出回波 I²+Q² 校验和（可选，用于回归测试验证确定性）
         * @param[out] timing       性能计时统计（可选）
         * @return 成功返回 true
         */
        bool generate_clutter_sequence_2d_fast_stream(const BeamPoint *beams,
                                                      std::uint64_t start_pulse_index,
                                                      std::uint64_t pulse_count,
                                                      double *checksum,
                                                      SeaClutterStreamTiming *timing);

        // =====================================================================
        // 状态查询
        // =====================================================================

        /// @brief 杂波配置只读引用
        const SeaClutterConfig &config() const { return cfg_; }
        /// @brief 天线配置只读引用
        const antenna::AntennaConfig &antenna_config() const { return ant_cfg_; }

        /// @brief 距离单元数量（等于 samples_per_pulse）
        int num_range_cells() const { return num_range_cells_; }
        /// @brief 方位 patch 数量（360° / az_patch_step_deg）
        int num_az_cells() const { return num_az_cells_; }
        /// @brief 距离分辨率 (m)
        Scalar range_resolution_m() const { return delta_r_m_; }
        /// @brief 方位 patch 间隔 (deg)
        Scalar patch_angular_resolution_deg() const { return delta_az_deg_; }
        /// @brief FIR 滤波器长度（奇数）
        int fir_length() const { return static_cast<int>(fir_coeffs_.size()); }
        /// @brief 实际使用的多普勒谱宽 σf (Hz)
        Scalar doppler_sigma_hz() const { return sigma_f_hz_; }
        /// @brief 最近一次 PRT 的活动方位 patch 数量
        int last_active_count() const { return last_active_count_; }

    private:
        // ---- 配置 ----
        SeaClutterConfig cfg_;
        antenna::AntennaConfig ant_cfg_;
        SeaClutterModel model_;
        RadarSystemParams sys_;

        // ---- 派生几何参数 ----
        int num_range_cells_ = 0;                       ///< 距离单元数
        int num_az_cells_ = 0;                          ///< 方位单元数
        Scalar delta_r_m_ = 0.0;                        ///< 距离分辨率 (m)
        Scalar delta_az_deg_ = 0.0;                     ///< 方位 patch 间隔 (deg)
        Scalar delta_az_rad_ = 0.0;                     ///< 方位 patch 间隔 (rad)
        Scalar sigma_f_hz_ = 0.0;                       ///< 实际使用的多普勒谱宽 σf (Hz)
        Scalar k_texture_rho_ = 0.0;                    ///< K 分布 texture AR(1) 单 PRT 相关系数 ρ
        Scalar k_texture_innovation_scale_ = 0.0;       ///< K 分布 AR(1) 创新项标准差 σ_innov = sqrt(1-ρ²)

        // ---- FIR 滤波器 ----
        ComplexVec fir_coeffs_;                          ///< FIR 滤波器系数 h[k]（归一化，Σ|h[k]|²=1）
        std::vector<Scalar> k_sqrt_tau_lut_;             ///< K 分布 z→sqrt(tau) 查找表（仅 K 分布时非空）

        // ---- 静态 patch 幅度图 ----
        std::vector<Scalar> patch_amplitude_map_;        ///< [az][range] 布局，元素为 sqrt(Pc)，不含天线方向图权重
        std::vector<Scalar> range_elevation_deg_;        ///< [range] 各距离单元对应的俯仰角 (deg)
        std::vector<Scalar> el_amplitude_weight_;        ///< [range] 俯仰方向图归一化幅度权重
        std::vector<cuda::ClutterActivePatch> active_patches_; ///< 复用缓冲区，避免逐 PRT 分配
        Scalar cached_beam_el_deg_ = 0.0f;               ///< 缓存的波束俯仰角，用于判断权重是否需要更新
        bool el_amplitude_weight_valid_ = false;         ///< 俯仰权重缓存是否有效

        // ---- GPU 运行状态 ----
        cuda::ClutterGpuState gpu_state_;
        int last_active_count_ = 0;

        // ---- 状态 ----
        bool initialized_ = false;
        std::string last_error_;

        // =====================================================================
        // 内部方法（实现在 .cpp 中）
        // =====================================================================

        /**
         * @brief 初始化静态 patch 幅度网格
         *
         * 委托 SeaClutterModel 完成 Morchin σ0、patch 面积和雷达方程计算，
         * Engine 接收预计算结果并准备后续 GPU 上传所需的缓存。
         * 同时校验各数组维度一致性。
         */
        bool initialize_patch_grid();

        /**
         * @brief 为指定波束俯仰角构建距离向俯仰幅度权重
         * @param beam_el_deg 波束俯仰角（度）
         *
         * 根据天线俯仰方向图和每个距离单元对应的仰角，
         * 计算归一化增益系数。结果存入 el_amplitude_weight_ 并缓存 beam_el_deg。
         */
        void build_elevation_amplitude_weight(Scalar beam_el_deg);

        /**
         * @brief 确保当前波束俯仰角的俯仰幅度权重已就绪
         * @param beam_el_deg 波束俯仰角（度）
         * @return 成功返回 true
         *
         * 若 beam_el_deg 与缓存值相同（误差 ≤ EPSILON），直接复用缓存；
         * 否则重新计算并上传至 GPU 显存。
         */
        bool ensure_elevation_amplitude_weight(Scalar beam_el_deg);

        /**
         * @brief 确定当前波束覆盖的活动方位 patch 窗口
         * @param beam_az_deg     波束方位角（度）
         * @param[out] active_patches 输出：活动 patch 列表（az_idx + 方位方向图权重）
         * @return 活动 patch 数量
         *
         * 算法：
         * 1. 将波束方位角归一化到 [0, 360) 度
         * 2. 计算波束中心对应的方位单元索引
         * 3. 由 active_gain_floor_db 门限反解最大偏移角：
         *    G(offset) = exp(-4·ln2 × (offset/BW)²) ≥ gain_floor
         *    → max_offset = BW × sqrt(-ln(gain_floor) / (4·ln2))
         * 4. 窗口内逐 patch 计算方位方向图权重
         * 5. 仅保留权重 ≥ gain_floor 的 patch
         */
        int build_active_patch_list(Scalar beam_az_deg,
                                    std::vector<cuda::ClutterActivePatch> &active_patches) const;
    };

} // namespace radar::clutter
