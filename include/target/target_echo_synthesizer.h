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
     * @brief 将目标轨迹合成为回波并写入CPI矩阵
     */
    static bool generate_target_echo(const TargetTrajectory& trajectory,
                                     const RadarSystemParams& system,
                                     const target::TargetConfig& config,
                                     const ComplexVec& tx_waveform,
                                     CpiEcho& out_echo,
                                     std::string& error);

    /**
     * @brief 将多个目标轨迹批量合成为回波并叠加写入CPI矩阵
     */
    static bool generate_target_echoes(const TrajectoryBatch& trajectories,
                                       const RadarSystemParams& system,
                                       const target::TargetConfig& config,
                                       const ComplexVec& tx_waveform,
                                       CpiEcho& out_echo,
                                       std::string& error);
};

}  // namespace radar