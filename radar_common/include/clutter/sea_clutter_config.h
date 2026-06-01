/**
 * @file sea_clutter_config.h
 * @brief 海杂波模块配置参数
 *
 * 设计说明：
 * - SeaClutterConfig: 扁平化配置容器，包含行为开关、物理参数、幅度分布参数等
 * - 与 noise::NoiseConfig 采用相同的配置透传模式，由仿真引擎统一注入
 *
 * 海杂波生成总体流程（FIR 高斯谱整形方案，详见 FIR.md）：
 *   白复高斯噪声 w[n] → 高斯谱 FIR 滤波器 h[k] → 距离-方位二维杂波序列 c[r,θ,n]
 *   c[r,θ,n] = sqrt(P_patch(r,θ)) × Σ h[k]·w[r,θ,n-k]
 *   其中 P_patch 由 Morchin 后向散射系数模型 + 雷达方程计算得出
 *
 * 支持的幅度分布类型：
 *   - Rayleigh：默认分布，I/Q 正交分量独立复高斯
 *   - Weibull：p=2 时退化为 Rayleigh
 *   - LogNormal：对 I/Q 施加对数正态幅度调制
 *   - K 分布：通过 Gamma texture AR(1) 过程引入慢变起伏
 */

#pragma once

#include "core/types.h"

#include <cstdint>
#include <string>

namespace radar::clutter
{

    /**
     * @brief 海杂波执行后端
     *
     * 当前引擎仅实现 GPU 后端，CPU 路径保留供后续扩展。
     */
    enum class SeaClutterBackend
    {
        CPU, ///< CPU 端逐 PRT 计算（未实现）
        GPU  ///< GPU 端并行加速（当前唯一可用后端）
    };


    /**
     * @brief 海杂波配置参数
     *
     * 包含：行为开关与后端选择、海况与极化、多普勒谱参数、
     * FIR 滤波参数、方位 patch 采样网格、幅度分布参数、距离范围等。
     * 配置通过 SeaClutterConfig::validate() 进行合法性校验。
     */
    struct SeaClutterConfig
    {
        // =====================================================================
        // 行为选项
        // =====================================================================

        bool enabled = true;                                 ///< 是否启用杂波生成
        SeaClutterBackend backend = SeaClutterBackend::GPU; ///< 执行后端（仅 GPU 可用）
        uint64_t seed = 2026;                                ///< 随机种子（用于 counter-based RNG）
        ClutterDistribution distribution = ClutterDistribution::Rayleigh; ///< 杂波幅度分布类型

        // =====================================================================
        // 海况与极化
        // =====================================================================

        Scalar sea_state = 3.0;                    ///< 海况等级（1-5，Morchin 模型）
        PolarizationType polar = PolarizationType::HH; ///< 极化方式

        // =====================================================================
        // 多普勒谱参数（高斯谱模型）
        // S(f) = exp[-(f - f_dc)² / (2·σf²)]
        // 频域高斯谱 ↔ 时域高斯包络（傅里叶变换对），用于 FIR 滤波器设计
        // =====================================================================

        Scalar doppler_center_hz = 0.0;            ///< 多普勒中心频率 f_dc (Hz)，通常设为 0 表示零频
        Scalar doppler_sigma_hz = 20.0;            ///< 多普勒谱标准差 σf (Hz)，决定杂波时间相关性

        // =====================================================================
        // FIR 滤波参数
        // =====================================================================

        int fir_length = 0;   ///< FIR 滤波器长度（奇数），0=自动推导（默认13）
                              ///< 推荐值参考（谱宽越窄，FIR 越长）：
                              ///<   50 Hz → 128~256,  30 Hz → 256~512,  10 Hz → 512~1024

        // =====================================================================
        // 方位 patch 采样与活动窗口
        // =====================================================================

        Scalar az_patch_step_deg = 0.25;           ///< 海面方位 patch 网格步长（度），固定值不随波束宽度变化
        Scalar active_gain_floor_db = -40.0;       ///< 活动方位窗口门限（dB），低于该值的天线增益 patch 被剔除

        // =====================================================================
        // 幅度分布参数（仅在 distribution 不为 Rayleigh 时生效）
        // =====================================================================

        Scalar weibull_shape = 2.0;                ///< Weibull 形状参数 p，p=2 且 q=√2 时退化为 Rayleigh
        Scalar weibull_scale = 1.41421356;         ///< Weibull 尺度参数 q（默认 √2，配合 p=2 退化为 Rayleigh）
        Scalar lognormal_mu = -1.0;                ///< LogNormal: ln(幅度) 的均值，默认使 E[A²] ≈ 1
        Scalar lognormal_sigma = 1.0;              ///< LogNormal: ln(幅度) 的标准差
        Scalar k_shape_nu = 1.0;                   ///< K 分布 Gamma texture 形状参数 ν，控制起伏程度（ν 越小起伏越大）
        Scalar k_texture_bandwidth_hz = 2.0;       ///< K 分布 texture AR(1) 过程的带宽 (Hz)，控制纹理慢变速度
        int k_lut_size = 4096;                     ///< K 分布 z → sqrt(tau) 查找表长度（越大精度越高）

        // =====================================================================
        // 距离范围（地距，用于限制杂波生成区域）
        // =====================================================================

        Scalar ground_range_min_m = -1.0;  ///< 地距下限 (m)，负值表示使用系统参数 sys.min_range_m 推算
        Scalar ground_range_max_m = -1.0;  ///< 地距上限 (m)，负值表示使用系统参数 sys.max_range_m 推算

        // =====================================================================
        // 验证
        // =====================================================================

        /**
         * @brief 验证配置参数的合法性
         *
         * 检查项包括：后端类型、海况范围、多普勒谱宽、patch 步长、
         * 活动窗口门限、分布类型及对应参数范围、FIR 长度、距离范围等。
         *
         * @param error 错误信息输出，仅在返回 false 时写入
         * @return 参数有效返回 true
         */
        bool validate(std::string &error) const;
    };

} // namespace radar::clutter
