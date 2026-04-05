/**
 * @file waveform_config.hpp
 * @brief 波形模块配置参数
 *
 * 设计说明：
 * - WaveformConfig: 波形行为选项（类型、编码方式、极化方式等）
 * - : 波形物理参数（由 RadarSystemParams 统一管理，此处不再重复）
 * - WaveformConfig: 完整配置容器
 */

#pragma once

#include "core/types.h"

#include <string>
#include <nlohmann/json.hpp>

namespace radar::waveform {


/**
 * @brief 波形配置器
 */
struct WaveformConfig {
    WaveformType waveform_type = WaveformType::LFM;         ///< 波形类型
    PhaseCodeType phase_code_type = PhaseCodeType::Barker13;///< 相位编码类型（PHASE_CODED 时使用）
    WindowType nlfm_window_type = WindowType::Hamming;      ///< NLFM 窗函数类型
    PolarizationType polarization = PolarizationType::HH;   ///< 极化方式

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

// JSON 序列化支持
inline void from_json(const nlohmann::json& j, WaveformConfig& cfg) {
    if (j.contains("waveform_type")) j.at("waveform_type").get_to(cfg.waveform_type);
    if (j.contains("phase_code_type")) j.at("phase_code_type").get_to(cfg.phase_code_type);
    if (j.contains("nlfm_window_type")) j.at("nlfm_window_type").get_to(cfg.nlfm_window_type);
    if (j.contains("polarization")) j.at("polarization").get_to(cfg.polarization);
}

inline void to_json(nlohmann::json& j, const WaveformConfig& cfg) {
    j = nlohmann::json{
        {"waveform_type", cfg.waveform_type},
        {"phase_code_type", cfg.phase_code_type},
        {"nlfm_window_type", cfg.nlfm_window_type},
        {"polarization", cfg.polarization}
    };
}

} // namespace radar::waveform
