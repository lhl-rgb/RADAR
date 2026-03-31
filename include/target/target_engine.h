/**
 * @file target_engine.h
 * @brief 点目标回波生成引擎（编排层）
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "core/antenna_set.h"
#include "core/radar_params.h"

namespace radar {

/**
 * @brief 波位视图
 * @details 包含波束指向角度、波位索引和天线指针
 */
struct BeamView {
    AzEl pointing{};                      ///< 波束指向（方位、仰角）
    int beam_index = 0;                   ///< 波位索引
    const PhasedArrayAntenna* antenna = nullptr; ///< 天线指针
};

using TargetList = std::vector<TargetState>;

/**
 * @brief 目标逐脉冲真值样本
 * @details
 * Kinematics层输出：位置/速度/加速度/距离/径向速度/增益/RCS
 * Synthesis层计算：延时/相位/幅度/回波矩阵
 */
struct TargetSnapshot {
    Scalar slow_time_s = 0.0;             ///< 慢时间（相对CPI起始）
    Vec3 position_m = Vec3::Zero();       ///< 位置（雷达本地直角坐标）
    Vec3 velocity_mps = Vec3::Zero();     ///< 速度
    Vec3 acceleration_mps2 = Vec3::Zero();///< 加速度
    Scalar range_m = 0.0;                 ///< 距离（Synthesis层计算延时需要）
    Scalar radial_velocity_mps = 0.0;     ///< 径向速度（Synthesis层计算多普勒需要）
    Scalar radial_acceleration_mps2 = 0.0;///< 径向加速度
    Scalar gain_linear = 1.0;             ///< 波束增益
    Scalar rcs_m2 = 1.0;                  ///< RCS（Swerling采样结果）
};

/**
 * @brief 一个目标在当前CPI的逐脉冲真值轨迹
 */
struct TargetTrajectory {
    uint64_t target_id = 0;               ///< 目标ID
    std::vector<TargetSnapshot> snapshots;///< 逐脉冲真值快照
};
using TrajectoryBatch = std::vector<TargetTrajectory>;

/**
 * @brief 点目标回波生成引擎
 * @details
 * 编排层，负责：
 * - 接收ActiveTargetList（由TargetManager提供）
 * - 编排TargetKinematics（Layer 1）和TargetEchoSynthesizer（Layer 2）
 * - 生成一个CPI的目标回波矩阵
 *
 * 不负责：
 * - 目标状态更新（TargetManager负责）
 * - 目标筛选（波位筛选）
 */
class TargetEngine {
public:
    /**
     * @brief 获取最近一次错误信息
     */
    const std::string& last_error() const { return last_error_; }

    /**
     * @brief 生成一个CPI的目标回波
     * @param active_targets   当前时刻的目标列表
     * @param beam             波位视图
     * @param radar            雷达参数
     * @param tx_waveform      发射波形
     * @param out_echo         输出的CPI回波矩阵
     * @return                 成功返回true
     */
    bool generate(const TargetList& active_targets,
                  const BeamView& beam,
                  const RadarParams& radar,
                  const ComplexVec& tx_waveform,
                  CpiEcho& out_echo);

    /**
     * @brief 生成一个CPI的目标回波（异常版本）
     * @details 失败时抛出std::runtime_error
     */
    CpiEcho generate_or_throw(const TargetList& active_targets,
                              const BeamView& beam,
                              const RadarParams& radar,
                              const ComplexVec& tx_waveform);

private:
    /**
     * @brief 初始化输出回波矩阵
     */
    bool init_output_echo(const BeamView& beam,
                          const RadarParams& radar,
                          CpiEcho& out_echo,
                          std::string& error) const;

private:
    std::string last_error_;              ///< 最近一次错误信息
};

}  // namespace radar