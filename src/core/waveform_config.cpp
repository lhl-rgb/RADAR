/**
 * @file waveform_config.cpp
 * @brief 波形配置参数实现
 */

#include "core/waveform_config.h"

namespace radar::waveform {

bool WaveformConfig::validate(std::string& error) const {
    // WaveformType validation - all enum values are valid
    // PhaseCodeType validation - all enum values are valid
    // WindowType validation - all enum values are valid
    // PolarizationType validation - all enum values are valid

    (void)error;  // No validation errors possible for enum-only config
    return true;
}

}  // namespace radar::waveform