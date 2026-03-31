/**
 * @file target_manager.cpp
 * @brief 目标状态管理器实现
 */

#include "target/target_manager.h"

namespace radar {

void TargetManager::set_targets(const TargetList& targets) {
    initial_targets_ = targets;  // 深拷贝初始状态
    current_targets_ = targets;  // 初始化当前状态
}

void TargetManager::update_to_time(Scalar current_time_s) {
    for (std::size_t i = 0; i < initial_targets_.size(); ++i) {
        const TargetState& init = initial_targets_[i];
        TargetState& curr = current_targets_[i];

        switch (init.motion_model) {
        case MotionModel::Stationary:
            // 静止目标：位置不变，速度为零
            curr.position_m = init.position_m;
            curr.velocity_mps = Vec3::Zero();
            curr.acceleration_mps2 = Vec3::Zero();
            break;

        case MotionModel::ConstantVelocity:
            // 匀速目标：位置线性变化，速度不变
            curr.position_m = init.position_m + init.velocity_mps * current_time_s;
            curr.velocity_mps = init.velocity_mps;
            curr.acceleration_mps2 = Vec3::Zero();
            break;

        case MotionModel::ConstantAcceleration:
            // 匀加速目标：位置二次变化，速度线性变化
            curr.position_m = init.position_m
                            + init.velocity_mps * current_time_s
                            + 0.5 * init.acceleration_mps2 * current_time_s * current_time_s;
            curr.velocity_mps = init.velocity_mps + init.acceleration_mps2 * current_time_s;
            curr.acceleration_mps2 = init.acceleration_mps2;
            break;

        case MotionModel::VariableAcceleration:
            // 变加速目标：第一版暂按匀加速处理
            curr.position_m = init.position_m
                            + init.velocity_mps * current_time_s
                            + 0.5 * init.acceleration_mps2 * current_time_s * current_time_s;
            curr.velocity_mps = init.velocity_mps + init.acceleration_mps2 * current_time_s;
            curr.acceleration_mps2 = init.acceleration_mps2;
            break;
        }

        // 保持不变的属性
        curr.id = init.id;
        curr.motion_model = init.motion_model;
        curr.rcs_mean_m2 = init.rcs_mean_m2;
        curr.swerling = init.swerling;
        curr.enabled = init.enabled;
    }
}

const std::vector<TargetState>& TargetManager::get_current_targets() const {
    return current_targets_;
}

void TargetManager::clear() {
    initial_targets_.clear();
    current_targets_.clear();
}

std::size_t TargetManager::size() const {
    return initial_targets_.size();
}

}  // namespace radar