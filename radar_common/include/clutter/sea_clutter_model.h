/**
 * @file sea_clutter_model.h
 * @brief 海杂波内部计算模型
 *
 * 设计说明：
 * - 此文件负责海杂波的物理模型、几何关系和初始化阶段预计算
 * - 由 SeaClutterEngine 在初始化和运行调度阶段调用
 * - 不保存任何 PRT 级别的随机状态、delay line 或 GPU buffer
 *
 * 包含五个计算模块：
 * 1. Morchin 海面后向散射系数模型（σ0 计算）
 * 2. 高斯多普勒谱 FIR 滤波器设计（CPU 端预计算系数）
 * 3. 雷达方程 patch 接收功率计算
 * 4. patch 照射面积与距离-方位二维网格构建
 * 5. 天线方向图幅度权重计算（方位 + 俯仰）
 */

#pragma once

#include "antenna/antenna_config.h"
#include "antenna/antenna_model.h"
#include "core/types.h"
#include "core/radar_system_params.h"
#include "clutter/sea_clutter_config.h"

#include <vector>

namespace radar::clutter
{
    /**
     * @brief 海杂波静态 patch 网格预计算结果
     *
     * 由 SeaClutterModel::build_patch_grid() 一次性计算，结果传递给 Engine 使用。
     *
     * 布局说明：
     * - patch_amplitude_map: [az][range] 布局，元素为 sqrt(Pc)，不含天线方向图权重
     * - range_elevation_deg: [range] 布局，每个距离单元对应的俯仰角（度）
     * - el_amplitude_weight: [range] 布局，默认波束俯仰角下的俯仰方向图幅度权重
     */
    struct SeaClutterPatchGrid
    {
        int num_range_cells = 0;
        int num_az_cells = 0;
        Scalar delta_r_m = 0.0f;
        Scalar delta_az_deg = 0.0f;
        Scalar delta_az_rad = 0.0f;
        std::vector<Scalar> patch_amplitude_map;
        std::vector<Scalar> range_elevation_deg;
        std::vector<Scalar> el_amplitude_weight;
    };

    /**
     * @brief 海杂波内部计算模型
     *
     * 保存配置和系统参数的副本，提供 Morchin、FIR、patch 网格等计算接口。
     * 所有计算均为无状态（const 方法），不保存每个 PRT 的随机状态或 GPU buffer。
     *
     * 使用方式：
     *   1. set_config() → 2. set_system_params() → 3. set_antenna_config()
     *   → 4. 调用各计算接口
     */
    class SeaClutterModel
    {
    public:
        SeaClutterModel() = default;

        /// @brief 设置杂波配置（使用前必须调用）
        void set_config(const SeaClutterConfig &cfg) { cfg_ = cfg; }
        
        /// @brief 设置雷达系统参数（使用前必须调用）
        void set_system_params(const RadarSystemParams &sys) { sys_ = sys; }
        
        /// @brief 设置天线配置并初始化天线方向图模型
        void set_antenna_config(const antenna::AntennaConfig &cfg);

        /// @brief 获取当前杂波配置副本
        const SeaClutterConfig config() const { return cfg_; }
        /// @brief 获取当前系统参数副本
        const RadarSystemParams system_params() const { return sys_; }
        /// @brief 获取当前天线配置只读引用
        const antenna::AntennaConfig &antenna_config() const { return ant_cfg_; }

        // =====================================================================
        // 1. Morchin 海面后向散射系数模型
        // =====================================================================

        /**
         * @brief Morchin 模型 σ0 计算（线性值）
         * @param grazing_rad  掠射角（弧度）
         * @return σ0 线性值（始终 > 0）
         *
         * Morchin 模型将后向散射系数分解为漫反射分量和镜面反射分量：
         *   σ0 = 漫反射项 + 镜面反射项
         *
         * 漫反射项（小掠射角主导）：
         *   diffuse = c0 × 10^(k_ss × ss) × (sin ψ / cos² ψ) × σ0_c
         *   其中 c0 为基础系数，ss 为海况等级，σ0_c 为临界角过渡函数
         *
         * 镜面反射项（大掠射角主导）：
         *   specular = cot²β × exp(-tan²(β-φ) / tan²β)
         *   其中 β 为海态相关粗糙度角，φ 为掠射角
         *
         * 参考文献：Morchin, W.C., "Airborne Early Warning Radar", 1990
         */
        Scalar morchin_sigma0_linear(Scalar grazing_rad) const;

        // =====================================================================
        // 2. 高斯谱 FIR 滤波器设计（CPU 端预计算）
        // =====================================================================

        /**
         * @brief 设计高斯多普勒谱 FIR 整形滤波器
         * @param sigma_f_hz  多普勒谱宽 σf (Hz)
         * @param f_dc_hz     多普勒中心频率 f_dc (Hz)
         * @param prf_hz      脉冲重复频率 PRF (Hz)
         * @param fir_len     滤波器长度（0 或负数使用默认值 13）
         * @return FIR 系数 h[k]（归一化，满足 Σ|h[k]|² = 1）
         *
         * 设计原理：
         * 频域高斯谱 S(f) = exp(-f²/(2·σf²)) 的傅里叶逆变换得到时域高斯包络，
         * 在离散时间域对包络采样并施加中心频率调制，得到 FIR 系数：
         *   h[n] ∝ exp(-4·σf²·π²·n² / PRF²) × exp(j·2π·f_dc·n / PRF)
         *
         * 归一化确保白噪声输入经过 FIR 后输出功率不变。
         */
        ComplexVec get_fir_coefficients(Scalar sigma_f_hz, Scalar f_dc_hz,
                                            Scalar prf_hz, int fir_len = 0) ;

        /**
         * @brief 构建距离-方位二维 patch 幅度图
         *
         * 计算步骤：
         * 1. 确定距离和方位网格参数
         * 2. 逐距离单元：地距 → 斜距 + 掠射角 → Morchin σ0 → patch 面积 → 雷达方程功率
         * 3. 均匀海面假设下，距离向幅度剖面沿方位复制到 [az][range] 二维图
         * 4. 构建默认波束俯仰角下的俯仰幅度权重
         *
         * @return 包含完整二维网格的 SeaClutterPatchGrid 结构
         */
        SeaClutterPatchGrid build_patch_grid() const;

        /**
         * @brief 构建指定波束俯仰角下的距离向俯仰幅度权重
         * @param range_elevation_deg 每个距离单元对应的俯仰角（度）
         * @param beam_el_deg 当前波束俯仰角（度）
         * @return [range] 俯仰方向图归一化幅度权重
         *
         * 对每个距离单元，调用天线俯仰方向图模型计算归一化增益。
         * 权重用于对杂波回波进行俯仰维度的幅度调制。
         */
        std::vector<Scalar> build_elevation_amplitude_weight(
            const std::vector<Scalar> &range_elevation_deg,
            Scalar beam_el_deg) const;

        /**
         * @brief 计算方位方向图归一化幅度权重
         * @param patch_az_deg patch 方位角（度）
         * @param beam_az_deg  波束中心方位角（度）
         * @return 归一化幅度权重（0~1，波束中心为 1.0）
         */
        Scalar normalized_az_amplitude(Scalar patch_az_deg, Scalar beam_az_deg) const;

        /**
         * @brief 获取天线峰值幅度权重（线性值，用于 GPU 端增益归一化）
         */
        Scalar peak_amplitude_weight() const;

        // =====================================================================
        // 3. 雷达方程 — patch 接收功率（距离相关）
        // =====================================================================

        /**
         * @brief 计算单个杂波 patch 的接收功率（不含天线增益 G²）
         * @param slant_range_m   斜距 (m)
         * @param sigma0          后向散射系数 σ0（线性值）
         * @param patch_area      patch 照射面积 (m²)
         * @return patch 接收功率 P_c（线性值，W）
         *
         * 雷达方程（不含天线方向图增益 G²，该增益在 GPU 端施加）：
         *   P_c = Pt × λ² × σ0 × A / ((4π)³ × R⁴ × L)
         * 其中：
         *   Pt = 峰值发射功率 (W)
         *   λ  = 波长 (m)
         *   A  = patch 照射面积 (m²)
         *   R  = 斜距 (m)
         *   L  = 系统损耗（线性值）
         */
        Scalar compute_patch_power(Scalar slant_range_m,
                                   Scalar sigma0,
                                   Scalar patch_area) const;

        /**
         * @brief 计算杂波 patch 照射面积
         * @param slant_range_m  斜距 (m)
         * @param delta_r_m      距离分辨率 (m)
         * @param delta_az_rad   方位角分辨率 (rad)
         * @return patch 面积 (m²)
         *
         * 公式：A = ΔR × R × Δθ
         * 即距离分辨率 × 方位弧长（斜距 × 方位角分辨率）
         */
        static Scalar compute_patch_area(Scalar slant_range_m,
                                         Scalar delta_r_m,
                                         Scalar delta_az_rad);


        // =====================================================================
        // 5. 几何工具
        // =====================================================================

        /**
         * @brief 地距 → 斜距 + 掠射角（平面地球几何模型）
         * @param ground_range_m    地距 (m)
         * @param antenna_height_m  天线高度 (m)
         * @param[out] slant_range_m 斜距 (m)，R = sqrt(ground_range² + h²)
         * @param[out] grazing_rad   掠射角 (弧度)，ψ = atan2(h, ground_range)
         */
        static void ground_to_slant(Scalar ground_range_m,
                                    Scalar antenna_height_m,
                                    Scalar &slant_range_m,
                                    Scalar &grazing_rad);


    private:
        // ---- Morchin 模型内部常数 ----
        constexpr static Scalar kMorchinBaseCoeff = 4.0e-7;       ///< σ0 基础系数 c0
        constexpr static Scalar kMorchinSeaStateRate = 0.6;       ///< 海况指数增长因子
        constexpr static Scalar kMorchinBetaCoeff = 2.44 / 57.29; ///< β 粗糙度系数（rad）
        constexpr static Scalar kMorchinBetaPower = 1.08;         ///< β 中海况指数幂次
        constexpr static Scalar kMorchinRoughnessBase = 0.025;    ///< 粗糙度基础参数
        constexpr static Scalar kMorchinRoughnessCoeff = 0.046;   ///< 粗糙度系数
        constexpr static Scalar kMorchinRoughnessPower = 1.72;    ///< 粗糙度中海况指数幂次
        constexpr static Scalar kMorchinTransitionPower = 1.9;    ///< 临界角过渡区指数

        SeaClutterConfig cfg_;                     ///< 杂波配置副本
        antenna::AntennaConfig ant_cfg_;           ///< 天线配置副本
        antenna::AntennaModel antenna_model_;      ///< 天线方向图模型（含方位和俯仰方向图）
        RadarSystemParams sys_;                    ///< 雷达系统参数副本
        ComplexVec fir_coeffs_;                    ///< 预计算的 FIR 滤波器系数缓存
    };

} // namespace radar::clutter
