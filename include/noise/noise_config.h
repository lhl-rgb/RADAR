/**
 * @file noise_config.h
 * @brief 噪声模块配置参数
 */

#pragma once

#include "core/types.h"
#include "core/radar_params.h"
#include <cstdint>
#include <string>

namespace radar::noise {

/**
 * @brief 噪声配置参数
 * @details
 * 本结构采用"显式模式"：
 * - mode = ComplexSigma：使用 sigma_complex 作为主输入；
 * - mode = NoisePower：使用 noise_power_w 作为主输入；
 * - mode = ThermalKTB：使用 k*T*B*F 推导噪声功率。
 *
 * 注意：noise_figure_db 从 RadarSystemParams 获取，不在此处定义。
 * NoiseLevelMode 枚举定义在 radar 命名空间中。
 */
struct NoiseConfig {
    NoiseLevelMode mode = NoiseLevelMode::ComplexSigma;  ///< 噪声强度配置模式

    Scalar sigma_complex = 1.0e-3;      ///< 复噪声总 RMS，满足 E{|n|^2}=sigma^2
    Scalar noise_power_w = 1.0e-6;      ///< 噪声功率（W）

    Scalar system_temperature_k = 290.0; ///< 系统噪声温度（K）（ThermalKTB 模式）
    Scalar noise_bandwidth_hz = 20.0e6;  ///< 噪声等效带宽（Hz）（ThermalKTB 模式）

    uint64_t seed = 12345;              ///< 随机种子

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

}  // namespace radar::noise