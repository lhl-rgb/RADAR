/**
 * @file target_kinematics.h
 * @brief 目标运动学与几何量计算（Layer 1）
 */

#pragma once

#include "antenna/antenna_model.h"
#include "core/types.h"
#include "target/target_config.h"

#include <string>

namespace radar::target {

using radar::antenna::AntennaModel;

/**
 * @brief 波束视图
 */
struct BeamView {
    BeamPoint pointing{};
    const AntennaModel* antenna = nullptr;
};

/**
 * @brief 当前脉冲上下文
 */
struct PulseContext {
    int scan_index = 0;
    int pulse_index_in_scan = 0;
    int global_pulse_index = 0;
    Scalar current_time_s = 0.0;
};

/**
 * @brief 目标运动学计算器（Layer 1）
 */
class TargetKinematics {
public:
    /**
     * @brief 为单个目标生成当前脉冲时刻的真值快照
     */
    static bool generate_snapshot(const TargetState& target,
                                  const BeamView& beam,
                                  const PulseContext& pulse,
                                  const target::TargetConfig& config,
                                  TargetSnapshot& out_snapshot,
                                  std::string& error);
};

}  // namespace radar::target
