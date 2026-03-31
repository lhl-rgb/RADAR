/**
 * @file target_echo_synthesizer.h
 * @brief 目标回波合成（Layer 2） 单目标
 * @details
 * 负责生成：延时、相位、幅度、回波矩阵
 */

#pragma once

#include <string>

#include "target/target_engine.h"

namespace radar {

/**
 * @brief 目标回波合成器（Layer 2）
 * @details
 * 输入：TargetTrajectory（Layer 1输出）、雷达参数、发射波形
 * 输出：写入CpiEcho的距离bin
 *
 * 计算内容：
 * - 延时：τ = 2R/c
 * - 相位：φ = -4πR(t)/λ（多普勒相位）
 * - 幅度：雷达方程 √(Pt·G²·λ²·σ / (4π)³·R⁴·L)
 */
class TargetEchoSynthesizer {
public:
    /**
     * @brief 将目标轨迹合成为回波并写入CPI矩阵
     * @details
     * 逐脉冲计算延时、相位、幅度，叠加到对应距离bin
     *
     * @param trajectory    目标真值轨迹（Layer 1输出）
     * @param radar         雷达参数
     * @param tx_waveform   发射波形
     * @param out_echo      输出的CPI回波矩阵（叠加写入）
     * @param error         错误信息（失败时填充）
     * @return              成功返回true
     */
    static bool generate_target_echo(const TargetTrajectory& trajectory,
                                     const RadarParams& radar,
                                     const ComplexVec& tx_waveform,
                                     CpiEcho& out_echo,
                                     std::string& error);
                                     
};

}  // namespace radar