/**
 * @file target_kinematics.h
 * @brief 目标运动学与几何量计算（Layer 1）
 * @details
 * 负责生成：位置、速度、加速度、距离、径向速度、增益、RCS
 */

#pragma once

#include <string>

#include "target/target_engine.h"

namespace radar {

/**
 * @brief 目标运动学计算器（Layer 1）
 * @details
 * 输入：目标初始状态、波位、雷达参数
 * 输出：TargetTrajectory（逐脉冲真值快照）
 *
 * 计算内容：
 * - 运动模型：Stationary/ConstantVelocity/ConstantAcceleration
 * - 几何量：距离、径向速度、径向加速度
 * - 天线增益：波束方向图
 * - RCS起伏：Swerling模型（0/1/2/3/4）
 */
class TargetKinematics {
public:
    /**
     * @brief 为目标生成一个CPI的完整真值轨迹
     * @details
     * 计算目标在CPI内每个脉冲时刻的运动状态、雷达几何、RCS起伏和天线增益
     *
     * @param target          目标运动模型和RCS参数（参考时刻状态）
     * @param beam            波束指向和天线参数
     * @param radar           雷达系统参数（PRI、脉冲数等）
     * @param out_trajectory  输出的逐脉冲轨迹数据
     * @param error           错误信息（失败时填充）
     * @return                成功返回true
     */
    static bool generate_trajectory(const TargetState& target,
                                    const BeamView& beam,
                                    const RadarParams& radar,
                                    TargetTrajectory& out_trajectory,
                                    std::string& error);
};

}  // namespace radar