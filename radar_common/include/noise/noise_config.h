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
#include <nlohmann/json.hpp>

namespace radar::noise {

/**
 * @brief 噪声配置（扁平化）
 *
 * 根据不同模式使用不同的参数：
 * - ComplexSigma: 使用 sigma_complex
 * - NoisePower: 使用 noise_power_w
 * - ThermalKTB: 使用 system_temperature_k 和 noise_bandwidth_hz
 */
struct NoiseConfig {
    // 行为选项
    bool enabled = true;                                ///< 是否启用噪声生成
    NoiseLevelMode mode = NoiseLevelMode::ComplexSigma; ///< 噪声强度配置模式
    uint64_t seed = 12345;                              ///< 随机种子

    // 物理参数
    Scalar sigma_complex = 1.0e-3;      ///< 复噪声总 RMS，满足 E{|n|^2}=sigma^2
    Scalar noise_power_w = 1.0e-6;      ///< 噪声功率（W）
    Scalar system_temperature_k = 290.0; ///< 系统噪声温度（K）（ThermalKTB 模式）
    Scalar noise_bandwidth_hz = 20.0e6;  ///< 噪声等效带宽（Hz）（ThermalKTB 模式）

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

// JSON 序列化支持
inline void from_json(const nlohmann::json& j, NoiseConfig& cfg) {
    if (j.contains("enabled")) j.at("enabled").get_to(cfg.enabled);
    if (j.contains("mode")) j.at("mode").get_to(cfg.mode);
    if (j.contains("seed")) j.at("seed").get_to(cfg.seed);
    if (j.contains("sigma_complex")) j.at("sigma_complex").get_to(cfg.sigma_complex);
    if (j.contains("noise_power_w")) j.at("noise_power_w").get_to(cfg.noise_power_w);
    if (j.contains("system_temperature_k")) j.at("system_temperature_k").get_to(cfg.system_temperature_k);
    if (j.contains("noise_bandwidth_hz")) j.at("noise_bandwidth_hz").get_to(cfg.noise_bandwidth_hz);
}

inline void to_json(nlohmann::json& j, const NoiseConfig& cfg) {
    j = nlohmann::json{
        {"enabled", cfg.enabled},
        {"mode", cfg.mode},
        {"seed", cfg.seed},
        {"sigma_complex", cfg.sigma_complex},
        {"noise_power_w", cfg.noise_power_w},
        {"system_temperature_k", cfg.system_temperature_k},
        {"noise_bandwidth_hz", cfg.noise_bandwidth_hz}
    };
}

} // namespace radar::noise
