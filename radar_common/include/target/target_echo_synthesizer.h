/**
 * @file target_echo_synthesizer.h
 * @brief 目标回波合成（Layer 2） 单目标
 */

#pragma once

#include "core/radar_system_params.h"
#include "target/target_config.h"
#include "target/target_kinematics.h"

#include <string>

namespace radar::target {

/**
 * @brief 目标回波合成器（Layer 2）
 */
class TargetEchoSynthesizer {
public:
    /**
     * @brief 将单个目标快照合成为当前脉冲回波并叠加写入 PulseEcho
     */
    static bool generate_target_echo(const TargetSnapshot& snapshot,
                                     const RadarSystemParams& system,
                                     const target::TargetConfig& config,
                                     const ComplexVec& tx_waveform,
                                     PulseEcho& out_echo,
                                     std::string& error);
};

}  // namespace radar
