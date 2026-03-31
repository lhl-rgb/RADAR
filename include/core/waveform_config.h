/**
 * @file waveform_config.h
 * @brief 波形模块配置参数
 */

#pragma once

#include "types.h"
#include <string>

namespace radar::waveform {

/**
 * @brief 波形配置参数
 * @details 控制波形生成的配置项
 */
struct WaveformConfig {
    WaveformType waveform_type = WaveformType::LFM;         ///< 波形类型
    PhaseCodeType phase_code_type = PhaseCodeType::Barker13;///< 相位编码类型
    WindowType nlfm_window_type = WindowType::Hamming;      ///< NLFM 窗函数类型
    PolarizationType polarization = PolarizationType::HH;   ///< 极化方式

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

}  // namespace radar::waveform