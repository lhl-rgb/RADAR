/**
 * @file target_kinematics.h
 * @brief 目标运动学与几何量计算（Layer 1）
 */

#pragma once

#include "antenna/antenna_model.h"
#include "core/radar_system_params.h"
#include "target/target_config.h"
#include <string>

namespace radar::target {

using radar::antenna::AntennaModel;

/**
 * @brief 波位视图
 * @details 包含波束指向角度、波位索引和天线指针
 */
struct BeamView {
    BeamPoint pointing{};                         ///< 波束指向（方位、仰角）
    int beam_index = 0;                            ///< 波位索引
    const AntennaModel* antenna = nullptr;  ///< 天线指针（用于获取增益等信息）
};

/**
 * @brief 目标运动学计算器（Layer 1）
 */
class TargetKinematics {
public:
    /**
     * @brief 为目标生成一个CPI的完整真值轨迹
     */
    static bool generate_trajectory(const TargetState& target,
                                    const BeamView& beam,
                                    const RadarSystemParams& system,
                                    const target::TargetConfig& config,
                                    TargetTrajectory& out_trajectory,
                                    std::string& error);

    /**
     * @brief 为多个目标批量生成一个CPI的完整真值轨迹
     */
    static bool generate_trajectories(const TargetList& targets,
                                      const BeamView& beam,
                                      const RadarSystemParams& system,
                                      const target::TargetConfig& config,
                                      TrajectoryBatch& out_trajectories,
                                      std::string& error);
};

}  // namespace radar