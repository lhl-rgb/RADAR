/**
 * @file noise_config.cpp
 * @brief 噪声配置参数实现
 */

#include "noise/noise_config.h"

namespace radar::noise {

bool NoiseConfig::validate(std::string& error) const {
    if (mode == NoiseLevelMode::ComplexSigma) {
        if (sigma_complex <= 0.0) {
            error = "sigma_complex must be positive in ComplexSigma mode";
            return false;
        }
    } else if (mode == NoiseLevelMode::NoisePower) {
        if (noise_power_w <= 0.0) {
            error = "noise_power_w must be positive in NoisePower mode";
            return false;
        }
    } else if (mode == NoiseLevelMode::ThermalKTB) {
        if (system_temperature_k <= 0.0) {
            error = "system_temperature_k must be positive in ThermalKTB mode";
            return false;
        }
        if (noise_bandwidth_hz <= 0.0) {
            error = "noise_bandwidth_hz must be positive in ThermalKTB mode";
            return false;
        }
    }

    return true;
}

}  // namespace radar::noise