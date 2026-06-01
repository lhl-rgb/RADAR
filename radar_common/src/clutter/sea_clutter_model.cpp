/**
 * @file sea_clutter_model.cpp
 * @brief 海杂波内部计算模型实现
 *
 * 实现五个核心计算模块：
 * 1. Morchin 后向散射系数 σ0 模型（漫反射 + 镜面反射分量）
 * 2. 高斯多普勒谱 FIR 滤波器系数设计（频域谱 → 时域包络采样）
 * 3. patch 照射面积计算（距离分辨率 × 方位弧长）
 * 4. 距离-方位二维 patch 幅度网格构建（均匀海面假设）
 * 5. 雷达方程接收功率计算（不含天线方向图增益 G²）
 */

#include "clutter/sea_clutter_model.h"
#include <utility>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace radar::clutter
{
    namespace
    {
        /// @brief 安全转换 int → size_t，负数映射为 0
        std::size_t to_size(int value)
        {
            return static_cast<std::size_t>(std::max(value, 0));
        }

        /// @brief 方位优先的二维索引：[az][range] 布局中的线性偏移
        /// @param az_idx 方位单元索引
        /// @param range_idx 距离单元索引
        /// @param range_count 距离单元总数
        /// @return 线性索引 = az_idx * range_count + range_idx
        std::size_t az_major_index(int az_idx, int range_idx, int range_count)
        {
            return to_size(az_idx) * to_size(range_count) + to_size(range_idx);
        }
    } // namespace

    // =========================================================================
    // 1. Morchin 后向散射系数模型
    // =========================================================================

    /// @brief 设置天线配置并初始化天线方向图模型
    ///
    /// 将配置注入 AntennaModel 并调用 initialize() 完成方位/俯仰方向图参数固化。
    /// 必须在调用 normalized_az_amplitude()、build_elevation_amplitude_weight() 等
    /// 依赖天线模型的接口之前调用。
    void SeaClutterModel::set_antenna_config(const antenna::AntennaConfig &cfg)
    {
        ant_cfg_ = cfg;
        antenna_model_.set_config(ant_cfg_);
        antenna_model_.initialize(); // 天线方向图模型完成参数固化
    }

    /// @brief Morchin 模型 σ0 计算实现（线性值）
    ///
    /// 将后向散射系数分解为漫反射分量和镜面反射分量，两部分分别在小掠射角和大掠射角主导。
    /// 通过临界角 φ_c = arcsin(λ/(4π·h_e)) 区分两个区域，使用 σ0_c 过渡函数平滑切换。
    ///
    /// 公式详见头文件注释。
    Scalar SeaClutterModel::morchin_sigma0_linear(Scalar grazing_rad) const{
        // ---- 参数准备与安全钳位 ----
        const Scalar phi = math::clamp_positive_eps(grazing_rad);        // 掠射角 ψ
        const Scalar lambda_m = math::clamp_positive_eps(sys_.wavelength_m); // 波长 λ
        const Scalar ss = math::clamp_nonnegative(cfg_.sea_state);      // 海况等级 ss
        const Scalar ss_plus_one = ss + 1.0;

        // ---- Morchin 模型参数 β（海态相关粗糙度角） ----
        // β = kMorchinBetaCoeff × (ss + 1)^kMorchinBetaPower
        const Scalar beta = kMorchinBetaCoeff * std::pow(ss_plus_one, kMorchinBetaPower);
        const Scalar tan_beta = std::max(std::tan(beta), EPSILON);
        const Scalar tan_beta_sq = tan_beta * tan_beta;
        const Scalar cot_beta_sq = 1.0 / tan_beta_sq; // cot²β

        // ---- 粗糙度参数 h_e（有效波高相关） ----
        // h_e = kMorchinRoughnessBase + kMorchinRoughnessCoeff × ss^kMorchinRoughnessPower
        const Scalar h_e =
            kMorchinRoughnessBase + kMorchinRoughnessCoeff * std::pow(ss, kMorchinRoughnessPower);

        // ---- 临界角 φ_c（区分镜面反射区与漫反射区） ----
        // φ_c = arcsin(λ / (4π × h_e))
        const Scalar phi_c_arg =
            std::clamp(lambda_m / std::max(static_cast<Scalar>(EPSILON), (4.0f * PI * h_e)), 0.0f, 1.0f);
        const Scalar phi_c = std::max(std::asin(phi_c_arg), EPSILON);

        // ---- 过渡函数 σ0_c（入射角 < 临界角时平滑过渡到镜面反射区） ----
        // σ0_c = (φ / φ_c)^kMorchinTransitionPower
        const Scalar sigma0_c =
            (phi < phi_c) ? std::pow(phi / phi_c, kMorchinTransitionPower) : 1.0;

        // ---- 漫反射分量（小掠射角主导） ----
        // diffuse = c0 × 10^(k_ss × ss) × σ0_c × sin(φ) / λ
        const Scalar sin_phi = std::max(std::sin(phi), EPSILON);
        const Scalar cot_phi = std::cos(phi) / sin_phi; // cot(φ)，用于镜面反射项
        const Scalar diffuse_term =
            (kMorchinBaseCoeff *
            std::pow(10.0, kMorchinSeaStateRate * ss_plus_one) *
            sigma0_c * sin_phi) / lambda_m;

        // ---- 镜面反射分量（大掠射角主导） ----
        // specular = cot²β × exp(-cot²φ / tan²β)
        const Scalar specular_term =
            cot_beta_sq * std::exp(-(cot_phi * cot_phi) / tan_beta_sq);

        // 最终 σ0 = 漫反射 + 镜面反射（线性值）
        return math::clamp_positive_eps(diffuse_term + specular_term);

    }

    // =========================================================================
    // 2. 高斯多普勒谱 FIR 滤波器设计
    // =========================================================================

    /// @brief 高斯多普勒谱 FIR 滤波器系数设计实现
    ///
    /// 从频域高斯谱 S(f) = exp(-f²/(2·σf²)) 出发，通过连续时间傅里叶逆变换得到
    /// 时域高斯包络，在离散时间 t = n/PRF 处采样并施加中心频率调制。
    /// 最后进行能量归一化使 Σ|h[k]|² = 1，保证滤波后输出功率不变。
    ///
    /// 设计原理详见头文件注释。
    ComplexVec SeaClutterModel::get_fir_coefficients(Scalar sigma_f_hz, Scalar f_dc_hz,Scalar prf_hz, int fir_len )  {

        // ---- 参数安全钳位 ----
        sigma_f_hz = math::clamp_positive_eps(sigma_f_hz);
        prf_hz = math::clamp_positive_eps(prf_hz);

        // 如果未指定阶数，默认使用 13（即 order = 6）
        if(fir_len <= 0 ){
            fir_len = 13;
        }

        // 确保 fir_len 为奇数，保证有中心点 (n=0)，滤波器对称
        if (fir_len % 2 == 0) {
            fir_len += 1;
        }     
        int order = (fir_len - 1) / 2; // 半边长度，n ∈ [-order, +order]
        ComplexVec coeffs(fir_len); 
        
        // ---- 高斯 FIR 系数计算 ----
        // 频域高斯谱 S(f) = exp(-f²/(2·σf²)) 的连续时间傅里叶逆变换：
        //   s(t) = sqrt(2π)·σf·exp(-2π²·σf²·t²)
        // 在离散时间 t = n/PRF 处采样，并施加中心频率偏移调制：
        //   h[n] = 2·σf·√π / PRF × exp(-4·σf²·π²·n² / PRF²) × exp(j·2π·f_dc·n / PRF)
        for (int i = 0; i < fir_len; ++i) {
            // 索引映射: i → n
            // i = 0          → n = -order
            // i = order      → n = 0  (中心，最大权重)
            // i = 2*order    → n = +order
            int n = i - order;
            
            // ---- Step 1: 高斯包络 ----
            // 频域高斯谱 → 时域高斯包络 (傅里叶变换对)
            double n_sq = static_cast<double>(n * n);
            double sigma_sq = sigma_f_hz * sigma_f_hz;
            double prf_sq = prf_hz * prf_hz;
            
            double gaussian = 2.0 * sigma_f_hz * std::sqrt(M_PI) *
                             std::exp(-4.0 * sigma_sq * M_PI * M_PI * n_sq / prf_sq) / prf_hz;
            
            // ---- Step 2: 复数调制（多普勒中心频率偏移） ----
            // modulation = exp(j × 2π × f_dc × n / PRF)
            // 系数 0.5: 将双边谱转为单边（h[n] 对应采样率 PRF 的一半有效带宽）
            double modulation_arg = 2.0 * M_PI * f_dc_hz * n / prf_hz;

            coeffs[i].real(static_cast<Scalar>(0.5 * gaussian * std::cos(modulation_arg)));
            coeffs[i].imag(static_cast<Scalar>(0.5 * gaussian * std::sin(modulation_arg)));
        }

        // ---- Step 3: 能量归一化 ----
        // 使 Σ|h[k]|² = 1，保证 FIR 滤波后输出功率与输入白噪声功率一致
        Scalar energy = 0.0;
        for (const auto &coeff : coeffs) {
            energy += std::norm(coeff);
        }
        if (energy > EPSILON) {
            const Scalar scale = 1.0f / std::sqrt(energy);
            for (auto &coeff : coeffs) {
                coeff *= scale;
            }
        }

        // 缓存系数（供外部查询）
        fir_coeffs_.assign(coeffs.begin(),coeffs.end());  
        return coeffs;
    }

    // =========================================================================
    // 3. patch 面积与距离-方位二维网格构建
    // =========================================================================

    /// @brief 杂波 patch 照射面积计算
    ///
    /// 公式：A = ΔR × R × Δθ
    /// - ΔR：距离分辨率 (m)
    /// - R：斜距 (m)
    /// - Δθ：方位角分辨率 (rad)
    /// 面积等于距离分辨率 × 斜距处的方位弧长。
    Scalar SeaClutterModel::compute_patch_area(Scalar slant_range_m,Scalar delta_r_m,Scalar delta_az_rad){
       // A = ΔR × R × Δθ（距离分辨率 × 斜距方位弧长）
       return  slant_range_m * delta_r_m * delta_az_rad;                               
    }

    /// @brief 构建距离-方位二维 patch 幅度图实现
    ///
    /// 流程：
    /// 1. 确定距离和方位网格参数（range_count, az_count, ΔR, Δθ）
    /// 2. 逐距离单元计算幅度剖面：
    ///    斜距 → 地距（勾股定理）→ 掠射角 → Morchin σ0 → patch 面积 → 雷达方程功率 → sqrt(功率)
    /// 3. 均匀海面假设：距离向幅度剖面沿方位复制到 [az][range] 二维图
    /// 4. 构建默认波束俯仰角下的俯仰幅度权重
    SeaClutterPatchGrid SeaClutterModel::build_patch_grid() const
    {
        SeaClutterPatchGrid grid;
        // ---- 网格参数初始化 ----
        grid.num_range_cells = std::max(1, sys_.samples_per_pulse);
        grid.delta_r_m = math::clamp_positive_eps(sys_.range_bin_size_m);
        grid.delta_az_deg = math::clamp_positive_eps(cfg_.az_patch_step_deg);
        grid.num_az_cells = std::max(1, static_cast<int>(std::lround(360.0f / grid.delta_az_deg)));
        grid.delta_az_rad = math::deg_to_rad(grid.delta_az_deg);

        const std::size_t range_count = to_size(grid.num_range_cells);
        const std::size_t az_count = to_size(grid.num_az_cells);

        // ---- 逐距离单元计算幅度剖面 ----
        std::vector<Scalar> range_amplitude(range_count, 0.0f);
        grid.patch_amplitude_map.assign(range_count * az_count, 0.0f);
        grid.range_elevation_deg.assign(range_count, 0.0f);
        grid.el_amplitude_weight.assign(range_count, 1.0f);

        // 确定杂波生成的地距范围：配置值优先，负值则使用系统参数默认值
        const Scalar clutter_min_m = cfg_.ground_range_min_m >= 0.0f ? cfg_.ground_range_min_m : sys_.min_range_m;
        const Scalar clutter_max_m = cfg_.ground_range_max_m >= 0.0f ? cfg_.ground_range_max_m : sys_.max_range_m;

        for (int range_idx = 0; range_idx < grid.num_range_cells; ++range_idx)
        {
            // 计算当前距离单元的斜距
            const Scalar slant_range_m = sys_.min_range_m + static_cast<Scalar>(range_idx) * grid.delta_r_m;
            const Scalar height_m = math::clamp_nonnegative(sys_.antenna_height_m);
            const Scalar slant_sq = slant_range_m * slant_range_m;
            const Scalar height_sq = height_m * height_m;
            // 斜距必须大于天线高度（地距为正）
            if (slant_sq <= height_sq)
            {
                continue;
            }

            // 斜距 → 地距（勾股定理）
            const Scalar ground_range_m = std::sqrt(slant_sq - height_sq);
            const std::size_t r = to_size(range_idx);
            // 计算俯仰角（负值表示向下看）
            grid.range_elevation_deg[r] =
                -math::rad_to_deg(std::atan2(height_m, math::clamp_positive_eps(ground_range_m)));
            // 检查是否在配置的地距范围内
            if (ground_range_m < clutter_min_m || ground_range_m > clutter_max_m)
            {
                continue;
            }

            // 掠射角 → Morchin σ0 → patch 面积 → 雷达方程接收功率 → sqrt(功率)
            const Scalar grazing_rad = std::atan2(height_m, math::clamp_positive_eps(ground_range_m));
            const Scalar sigma0 = morchin_sigma0_linear(grazing_rad);
            const Scalar area = compute_patch_area(slant_range_m, grid.delta_r_m, grid.delta_az_rad);
            const Scalar patch_power = std::max(0.0f, compute_patch_power(slant_range_m,
                                                                          sigma0,
                                                                          area));
            range_amplitude[r] = std::sqrt(patch_power);
        }

        // ---- 构建默认俯仰幅度权重（波束俯仰角 = 0°） ----
        grid.el_amplitude_weight = build_elevation_amplitude_weight(grid.range_elevation_deg, ant_cfg_.el_beam_point_deg);

        // ---- 均匀海面假设：距离向剖面沿方位复制到 [az][range] 二维图 ----
        for (int az_idx = 0; az_idx < grid.num_az_cells; ++az_idx)
        {
            const std::size_t az_offset = az_major_index(az_idx, 0, grid.num_range_cells);
            std::copy(range_amplitude.begin(),
                      range_amplitude.end(),
                      grid.patch_amplitude_map.begin() + static_cast<std::ptrdiff_t>(az_offset));
        }

        return grid;
    }

    // =========================================================================
    // 4. 天线方向图幅度权重
    // =========================================================================

    /// @brief 构建指定波束俯仰角下的距离向俯仰幅度权重实现
    ///
    /// 对每个距离单元，调用天线俯仰方向图模型计算归一化增益。
    /// 权重用于 GPU 端对杂波回波进行俯仰维度的幅度调制。
    std::vector<Scalar> SeaClutterModel::build_elevation_amplitude_weight(
        const std::vector<Scalar> &range_elevation_deg,
        Scalar beam_el_deg) const
    {
        std::vector<Scalar> weight(range_elevation_deg.size(), 1.0f);
        for (std::size_t range_idx = 0; range_idx < range_elevation_deg.size(); ++range_idx)
        {
            // 对每个距离单元，计算其仰角与波束俯仰角之间的天线增益
            weight[range_idx] =
                antenna_model_.normalized_el_amplitude(range_elevation_deg[range_idx], beam_el_deg);
        }
        return weight;
    }

    /// @brief 方位方向图归一化幅度权重实现
    ///
    /// 计算 patch 方位角与波束中心方位角之间的天线增益（归一化，波束中心 = 1.0）。
    Scalar SeaClutterModel::normalized_az_amplitude(Scalar patch_az_deg, Scalar beam_az_deg) const
    {
        // 方位方向图归一化幅度权重：patch 方位角与波束中心方位角的增益
        return antenna_model_.normalized_az_amplitude(patch_az_deg, beam_az_deg);
    }

    /// @brief 天线峰值增益实现
    ///
    /// 返回天线峰值增益（线性值），用于 GPU 端幅度归一化。
    /// 波束中心方向的增益归一化因子 = 1 / peak_gain_linear。
    Scalar SeaClutterModel::peak_amplitude_weight() const
    {
        // 天线峰值增益（线性值），用于 GPU 端幅度归一化
        return antenna_model_.peak_gain_linear();
    }

    // =========================================================================
    // 5. 地距-斜距几何转换
    // =========================================================================

    /// @brief 地距 → 斜距 + 掠射角（平面地球几何模型）实现
    ///
    /// 公式：
    /// - 斜距 R = sqrt(ground_range² + h²)（勾股定理）
    /// - 掠射角 ψ = atan2(h, ground_range)
    ///
    /// 适用条件：地距 << 地球半径（岸基雷达典型场景）。
    void SeaClutterModel::ground_to_slant(Scalar ground_range_m,Scalar antenna_height_m,Scalar &slant_range_m,Scalar &grazing_rad){
        // ---- 输入钳位 ----
        ground_range_m = math::clamp_positive_eps(ground_range_m);
        antenna_height_m = math::clamp_nonnegative(antenna_height_m);

        // ---- 斜距计算（勾股定理，平面地球近似） ----
        // R = sqrt(ground_range² + h²)
        slant_range_m = std::sqrt(ground_range_m * ground_range_m + antenna_height_m * antenna_height_m);

        // ---- 掠射角计算 ----
        // ψ = atan2(h, ground_range)
        grazing_rad = std::atan2(antenna_height_m,ground_range_m);
    }
    
    // =========================================================================
    // 6. patch 接收功率（雷达方程）
    // =========================================================================

    /// @brief patch 接收功率计算实现（雷达方程，不含天线增益 G²）
    ///
    /// 雷达方程（不含天线方向图增益 G²，该增益在 GPU 端按方向图实时施加）：
    ///   P_c = Pt × λ² × σ0 × A / ((4π)³ × R⁴ × L)
    /// 其中：
    ///   Pt  = 峰值发射功率 (W)
    ///   λ   = 波长 (m)
    ///   σ0  = 后向散射系数（线性值，由 Morchin 模型计算）
    ///   A   = patch 照射面积 (m²)
    ///   R   = 斜距 (m)
    ///   L   = 系统损耗（线性值）
    Scalar SeaClutterModel::compute_patch_power(Scalar slant_range_m,Scalar sigma0,Scalar patch_area) const {
        const Scalar range2 = slant_range_m * slant_range_m;
        const Scalar range4 = range2 * range2;
        return sys_.peak_power_w * sys_.wavelength_m * sys_.wavelength_m * sigma0 * patch_area /
               (std::pow(4.0f * PI, 3.0f) * range4 * math::clamp_positive_eps(sys_.system_loss_linear));
    }
    

}
