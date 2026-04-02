/**
 * @file clutter_engine.cpp
 * @brief 杂波管理引擎实现
 */

#include "clutter/clutter_engine.h"

namespace radar::clutter {

bool ClutterEngine::initialize() {
    if (initialized_) {
        return true;
    }

    if (!sea_cfg_.options.enabled) {
        enabled_ = false;
        initialized_ = true;
        return true;
    }

    enabled_ = true;
    if (!sea_model_.set_params(sea_cfg_)) {
        last_error_ = sea_model_.last_error();
        return false;
    }

    initialized_ = true;
    last_error_.clear();
    return true;
}

bool ClutterEngine::generate_sea_clutter_cpi(const RadarSystemParams& system,
                                             const antenna::AntennaModel& antenna,
                                             const AzEl& beam_pointing,
                                             int beam_index,
                                             const ComplexVec& tx_waveform,
                                             CpiEcho& out_clutter) {
    if (!enabled_) {
        return true;  // 未启用时直接返回成功，不生成杂波
    }

    if (!initialized_) {
        last_error_ = "ClutterEngine not initialized";
        return false;
    }

    if (!sea_model_.generate_cpi(system, antenna, beam_pointing, beam_index, tx_waveform,
                                 out_clutter)) {
        last_error_ = sea_model_.last_error();
        return false;
    }
    last_error_.clear();
    return true;
}

}  // namespace radar::clutter