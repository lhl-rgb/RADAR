/**
 * @file clutter_engine.h
 * @brief 杂波管理引擎
 *
 * 当前版本仅管理海杂波模型，后续可扩展陆杂波/雨杂波等。
 *
 * 工作流程：
 * 1. 配置海杂波参数（K 分布、多普勒展宽等）
 * 2. 初始化海杂波模型
 * 3. 根据波束指向和雷达参数生成杂波回波
 * 4. 输出复数 CPI 回波数据
 */

#pragma once

#include "core/types.h"
#include "core/radar_system_params.h"
#include "clutter/sea_clutter_config.h"
#include "clutter/sea_clutter_model.h"
#include "antenna/antenna_model.h"

#include <string>

namespace radar::clutter {

/**
 * @brief 杂波管理引擎
 */
class ClutterEngine {
public:
    ClutterEngine() = default;
    ~ClutterEngine() = default;

    // 禁止拷贝
    ClutterEngine(const ClutterEngine&) = delete;
    ClutterEngine& operator=(const ClutterEngine&) = delete;

    // ========================================================================
    // 配置注入
    // ========================================================================

    /**
     * @brief 设置海杂波配置
     * @param cfg 海杂波配置参数
     */
    void set_config(const SeaClutterConfig& cfg) { sea_cfg_ = cfg; }

    // ========================================================================
    // 初始化
    // ========================================================================

    /**
     * @brief 初始化杂波引擎
     * @return 如果初始化成功返回 true
     *
     * 验证海杂波配置并初始化海杂波模型。
     */
    bool initialize();

    /**
     * @brief 检查是否已初始化
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief 获取最近一次错误信息
     */
    const std::string& last_error() const { return last_error_; }

    // ========================================================================
    // 运行时控制
    // ========================================================================

    /**
     * @brief 检查海杂波是否启用
     */
    bool is_enabled() const { return enabled_ && initialized_; }

    // ========================================================================
    // 核心功能
    // ========================================================================

    /**
     * @brief 生成一个 CPI 的海杂波原始回波
     * @param system 雷达系统参数
     * @param antenna 相控阵天线模型
     * @param beam_pointing 波束指向（方位、仰角）
     * @param beam_index 波位索引
     * @param tx_waveform 发射波形
     * @param out_clutter 输出杂波回波
     * @return 如果生成成功返回 true
     */
    bool generate_sea_clutter_cpi(const RadarSystemParams& system,
                                  const antenna::AntennaModel& antenna,
                                  const BeamPoint& beam_pointing,
                                  int beam_index,
                                  const ComplexVec& tx_waveform,
                                  CpiEcho& out_clutter);

    // ========================================================================
    // 状态查询
    // ========================================================================

    /**
     * @brief 获取当前海杂波配置
     */
    const SeaClutterConfig& config() const { return sea_cfg_; }

    /**
     * @brief 获取海杂波模型只读引用
     */
    const SeaClutterModel& sea_model() const { return sea_model_; }

private:
    SeaClutterModel sea_model_;
    SeaClutterConfig sea_cfg_;
    bool initialized_ = false;
    bool enabled_ = false;
    std::string last_error_;
};

} // namespace radar::clutter
