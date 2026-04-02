/**
 * @file target_manager.h
 * @brief 目标状态管理器
 * @details
 * 负责目标状态的时间推进，不负责波位筛选。
 * 使用绝对时间方案：从初始状态计算指定时刻的位置/速度。
 */

#pragma once

#include <vector>

#include "core/types.h"

namespace radar::target {

/**
 * @brief 目标状态管理器
 * @details
 * 维护目标初始状态（时刻0），根据运动模型计算指定时刻的位置/速度。
 *
 * 使用方式：
 * 1. set_targets() 设置初始目标列表
 * 2. update_to_time() 更新到当前时刻
 * 3. get_current_targets() 获取当前状态供TargetEngine使用
 *
 * 支持的运动模型：
 * - Stationary：位置不变
 * - ConstantVelocity：匀速运动
 * - ConstantAcceleration：匀加速运动
 */
class TargetManager {
public:
    TargetManager() = default;

    /**
     * @brief 设置初始目标列表（时刻0的状态）
     * @param targets 初始目标状态列表
     */
    void set_targets(const TargetList& targets);

    /**
     * @brief 更新所有目标状态到绝对时刻
     * @param current_time_s 当前时刻（秒）
     */
    void update_to_time(Scalar current_time_s);

    /**
     * @brief 获取当前时刻的目标列表
     * @return 当前目标状态列表（供TargetEngine使用）
     */
    const TargetList& get_current_targets() const;

    /**
     * @brief 清空目标列表
     */
    void clear();

    /**
     * @brief 获取目标数量
     */
    std::size_t size() const;

private:
    TargetList initial_targets_;    ///< 初始状态（时刻0）
    TargetList current_targets_;    ///< 当前状态
};

}  // namespace radar