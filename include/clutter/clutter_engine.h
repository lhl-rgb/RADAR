/**
 * @file clutter_engine.h
 * @brief 杂波管理引擎
 */

#pragma once

#include <string>

#include "clutter/sea_clutter_model.h"

namespace radar {

/**
 * @brief 杂波管理器
 * @details
 * 当前版本仅管理海杂波模型，后续可扩展陆杂波/雨杂波等。
 */
class ClutterEngine {
public:
    ClutterEngine() = default;
    explicit ClutterEngine(const SeaClutterParams& sea_params);

    /**
     * @brief 设置海杂波参数
     */
    bool set_sea_params(const SeaClutterParams& sea_params);

    /**
     * @brief 生成一个 CPI 的海杂波原始回波
     */
    bool generate_sea_clutter_cpi(const RadarParams& radar_params,
                                  const PhasedArrayAntenna& antenna,
                                  const AzEl& beam_pointing,
                                  int beam_index,
                                  const ComplexVec& tx_waveform,
                                  CpiEcho& out_clutter);

    /**
     * @brief 获取最近一次错误信息
     */
    const std::string& last_error() const { return last_error_; }

    /**
     * @brief 获取海杂波模型只读引用
     */
    const SeaClutterModel& sea_model() const { return sea_model_; }

private:
    SeaClutterModel sea_model_;
    std::string last_error_;
};

}  // namespace radar

