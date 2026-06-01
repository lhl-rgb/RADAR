/**
 * @file waveform_config.cpp
 * @brief 波形模块配置参数实现
 */

#include "waveform/waveform_config.h"

namespace radar::waveform
{

    bool WaveformConfig::validate(std::string &error) const
    {
        // 波形类型检查
        if (waveform_type != WaveformType::LFM &&
            waveform_type != WaveformType::NLFM &&
            waveform_type != WaveformType::PHASE_CODED &&
            waveform_type != WaveformType::CW)
        {
            error = "Invalid waveform type";
            return false;
        }

        // 相位编码类型检查
        if (waveform_type == WaveformType::PHASE_CODED)
        {
            if (phase_code_type != PhaseCodeType::Barker2 &&
                phase_code_type != PhaseCodeType::Barker3 &&
                phase_code_type != PhaseCodeType::Barker4 &&
                phase_code_type != PhaseCodeType::Barker5 &&
                phase_code_type != PhaseCodeType::Barker7 &&
                phase_code_type != PhaseCodeType::Barker11 &&
                phase_code_type != PhaseCodeType::Barker13)
            {
                error = "Invalid phase code type";
                return false;
            }
        }

        return true;
    }

} // namespace radar::waveform
