/**
 * @file clutter_engine.cpp
 * @brief 杂波管理引擎实现
 */

#include "clutter/clutter_engine.h"

namespace radar {

ClutterEngine::ClutterEngine(const SeaClutterParams& sea_params) {
    // 构造阶段尽早校验参数；失败时错误信息由 last_error_ 暴露。
    (void)set_sea_params(sea_params);
}

bool ClutterEngine::set_sea_params(const SeaClutterParams& sea_params) {
    if (!sea_model_.set_params(sea_params)) {
        last_error_ = sea_model_.last_error();
        return false;
    }
    last_error_.clear();
    return true;
}

bool ClutterEngine::generate_sea_clutter_cpi(const RadarParams& radar_params,
                                             const PhasedArrayAntenna& antenna,
                                             const AzEl& beam_pointing,
                                             int beam_index,
                                             const ComplexVec& tx_waveform,
                                             CpiEcho& out_clutter) {
    // 当前为透传封装：统一错误接口，不改变 SeaClutterModel 的计算语义。
    if (!sea_model_.generate_cpi(radar_params, antenna, beam_pointing, beam_index, tx_waveform,
                                 out_clutter)) {
        last_error_ = sea_model_.last_error();
        return false;
    }
    last_error_.clear();
    return true;
}

}  // namespace radar
