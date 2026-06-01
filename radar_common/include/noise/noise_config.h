/**
 * @file noise_config.hpp
 * @brief 噪声模块配置参数
 *
 * 设计说明：
 * - NoiseConfig: 扁平化配置容器（行为选项 + 物理参数）
 *
 * 噪声模式：
 * 1. ComplexSigma - 直接使用复噪声标准差
 * 2. NoisePower - 使用噪声功率 (W)
 * 3. ThermalKTB - 使用热噪声公式 KTB
 */

#pragma once

#include "core/types.h"

#include <cstdint>
#include <string>

namespace radar::noise
{

    enum class NoiseExecutionBackend
    {
        Auto,
        CPU,
        GPU
    };

    /**
     * @brief 噪声配置
     *
     * 根据不同模式使用不同的参数：
     * - ComplexSigma: 使用 sigma_complex
     * - NoisePower: 使用 noise_power_w
     * - ThermalKTB: 使用 system_temperature_k 和 noise_bandwidth_hz
     */
    struct NoiseConfig
    {
        // 行为选项
        bool enabled = true;                                ///< 是否启用噪声生成
        NoiseLevelMode mode = NoiseLevelMode::ComplexSigma; ///< 噪声强度配置模式
        NoiseExecutionBackend backend = NoiseExecutionBackend::Auto;
        uint64_t seed = 12345; ///< 随机种子

        // 物理参数
        Scalar sigma_complex = 1.0e-3;       ///< 复噪声总 RMS，满足 E{|n|^2}=sigma^2
        Scalar noise_power_w = 1.0e-6;       ///< 噪声功率（W）
        Scalar system_temperature_k = 290.0; ///< 系统噪声温度（K）（ThermalKTB 模式）
        Scalar noise_bandwidth_hz = 20.0e6;  ///< 噪声等效带宽（Hz）（ThermalKTB 模式）

        /**
         * @brief 验证配置参数
         * @param error 错误信息输出
         * @return 有效返回 true
         */
        bool validate(std::string &error) const;
    };

} // namespace radar::noise
