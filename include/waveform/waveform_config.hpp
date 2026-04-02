/**
 * @file waveform_config.hpp
 * @brief 波形模块配置参数
 *
 * 设计说明：
 * - WaveformOptions: 波形行为选项（类型、编码方式、极化方式等）
 * - WaveformParams: 波形物理参数（由 RadarSystemParams 统一管理，此处不再重复）
 * - WaveformConfig: 完整配置容器
 */

#pragma once

#include "core/types.h"

#include <string>

namespace radar::waveform {

/**
 * @brief 波形行为选项
 *
 * 只包含行为选项，不包含物理参数。
 */
struct WaveformOptions {
    WaveformType waveform_type = WaveformType::LFM;         ///< 波形类型
    PhaseCodeType phase_code_type = PhaseCodeType::Barker13;///< 相位编码类型（PHASE_CODED 时使用）
    WindowType nlfm_window_type = WindowType::Hamming;      ///< NLFM 窗函数类型
    PolarizationType polarization = PolarizationType::HH;   ///< 极化方式
};

/**
 * @brief 波形物理参数
 *
 * 波形本身没有独立的物理参数，所有参数（带宽、脉宽等）都来自 RadarSystemParams。
 * 此结构体预留扩展点。
 */
struct WaveformParams {
    // 波形参数统一由 RadarSystemParams 管理：
    // - bandwidth_hz (bw_hz)
    // - pulse_width_s
    // - sample_rate_hz (fs_hz)
    // 此处无需重复定义
};

/**
 * @brief 波形完整配置容器
 */
struct WaveformConfig {
    WaveformOptions options;    ///< 波形行为选项
    WaveformParams params;      ///< 波形物理参数（预留）

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

inline bool WaveformConfig::validate(std::string& error) const {
    // 波形类型检查
    if (options.waveform_type != WaveformType::LFM &&
        options.waveform_type != WaveformType::NLFM &&
        options.waveform_type != WaveformType::PHASE_CODED &&
        options.waveform_type != WaveformType::CW) {
        error = "Invalid waveform type";
        return false;
    }

    // 相位编码类型检查
    if (options.waveform_type == WaveformType::PHASE_CODED) {
        if (options.phase_code_type != PhaseCodeType::Barker2 &&
            options.phase_code_type != PhaseCodeType::Barker3 &&
            options.phase_code_type != PhaseCodeType::Barker4 &&
            options.phase_code_type != PhaseCodeType::Barker5 &&
            options.phase_code_type != PhaseCodeType::Barker7 &&
            options.phase_code_type != PhaseCodeType::Barker11 &&
            options.phase_code_type != PhaseCodeType::Barker13) {
            error = "Invalid phase code type";
            return false;
        }
    }

    return true;
}

} // namespace radar::waveform
