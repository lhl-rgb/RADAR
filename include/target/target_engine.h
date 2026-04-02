/**
 * @file target_engine.h
 * @brief 点目标回波生成引擎（编排层）
 *
 * 负责根据目标状态、波束指向等生成一个 CPI 的目标回波。
 *
 * 工作流程：
 * 1. 设置初始目标状态
 * 2. 更新目标状态到当前时刻
 * 3. 生成目标轨迹（考虑波束增益调制）
 * 4. 合成目标回波（距离徙动、相位调制等）
 * 5. 返回合成的回波数据
 */

#pragma once

#include "core/types.h"
#include "core/radar_system_params.hpp"
#include "target/target_config.hpp"
#include "target/target_echo_synthesizer.h"
#include "target/target_kinematics.h"
#include "target/target_manager.h"
#include "antenna/antenna_model.h"

#include <string>
#include <vector>

namespace radar::target {

/**
 * @brief 点目标回波生成引擎（统一管理器）
 * @details
 * 负责目标的全生命周期管理：
 * - 设置初始目标状态
 * - 更新目标状态到指定时刻
 * - 生成目标回波
 */
class TargetEngine {
public:
    explicit TargetEngine() = default;
    ~TargetEngine() = default;

    // 禁止拷贝
    TargetEngine(const TargetEngine&) = delete;
    TargetEngine& operator=(const TargetEngine&) = delete;

    // ========================================================================
    // 配置注入
    // ========================================================================

    /**
     * @brief 设置目标配置
     * @param cfg 目标配置参数
     */
    void set_config(const TargetConfig& cfg) { cfg_ = cfg; }

    // ========================================================================
    // 初始化
    // ========================================================================

    /**
     * @brief 初始化目标引擎
     * @return 如果初始化成功返回 true
     */
    bool initialize() { initialized_ = true; return true; }

    /**
     * @brief 检查是否已初始化
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief 获取最近一次错误信息
     */
    const std::string& last_error() const { return last_error_; }

    // ========================================================================
    // 目标状态管理（委托给 TargetManager）
    // ========================================================================

    /**
     * @brief 设置初始目标状态
     * @param targets 初始目标状态列表
     */
    void set_initial_targets(const TargetList& targets);

    /**
     * @brief 更新目标状态到指定时刻
     * @param current_time_s 当前时刻（秒）
     */
    void update_targets(Scalar current_time_s);

    /**
     * @brief 获取当前目标状态列表
     * @return 当前目标状态列表（只读引用）
     */
    const TargetList& get_current_targets() const;

    /**
     * @brief 清空目标列表
     */
    void clear_targets();

    /**
     * @brief 获取目标数量
     */
    std::size_t target_count() const;

    // ========================================================================
    // 运行时控制
    // ========================================================================

    /**
     * @brief 检查目标引擎是否启用
     */
    bool is_enabled() const { return cfg_.enabled; }

    // ========================================================================
    // 核心功能
    // ========================================================================

    /**
     * @brief 生成一个 CPI 的目标回波
     * @param beam 波束视图（包含波束指向、索引、天线指针）
     * @param system 雷达系统参数
     * @param tx_waveform 发射波形
     * @param out_echo 输出回波
     * @return 如果生成成功返回 true
     */
    bool generate(const BeamView& beam,
                  const RadarSystemParams& system,
                  const ComplexVec& tx_waveform,
                  CpiEcho& out_echo);

    /**
     * @brief 生成一个 CPI 的目标回波（异常版本）
     * @param beam 波束视图
     * @param system 雷达系统参数
     * @param tx_waveform 发射波形
     * @return 合成的目标回波
     * @throws std::runtime_error 如果生成失败
     */
    CpiEcho generate_or_throw(const BeamView& beam,
                              const RadarSystemParams& system,
                              const ComplexVec& tx_waveform);

    // ========================================================================
    // 状态查询
    // ========================================================================

    /**
     * @brief 获取当前配置
     */
    const TargetConfig& config() const { return cfg_; }

private:
    /**
     * @brief 初始化输出回波结构
     * @param beam 波束视图
     * @param system 雷达系统参数
     * @param out_echo 输出回波
     * @param error 错误信息输出
     * @return 如果初始化成功返回 true
     */
    bool init_output_echo(const BeamView& beam,
                          const RadarSystemParams& system,
                          CpiEcho& out_echo,
                          std::string& error) const;

    TargetConfig cfg_;
    bool initialized_ = false;
    std::string last_error_;

    // 子组件
    TargetManager target_manager_;        ///< 目标状态管理器（被拥有）
};

} // namespace radar::target
