/**
 * @file antenna_config.cpp
 * @brief 天线模块配置参数实现（反射面天线）
 */

#include "antenna/antenna_config.h"

#include <cmath>

namespace radar::antenna {

void MechanicalScanConfig::compute_derived_params(Scalar pri_s) {
    if (rotation_rate_dps > 0.0) {
        rotation_period_s = 360.0 / rotation_rate_dps;
    } else {
        rotation_period_s = 0.0;
    }

    if (pri_s > 0.0 && rotation_rate_dps > 0.0) {
        azimuth_step_per_prt_deg = rotation_rate_dps * pri_s;
        pulses_per_rotation = std::max(1, static_cast<int>(std::lround(360.0 / azimuth_step_per_prt_deg)));
    } else {
        azimuth_step_per_prt_deg = 0.0;
        pulses_per_rotation = 0;
    }
}

bool MechanicalScanConfig::validate(std::string& error) const {
    if (rotation_rate_dps <= 0.0) {
        error = "rotation_rate_dps must be positive";
        return false;
    }
    if (az_start_deg < 0.0 || az_start_deg >= 360.0) {
        error = "az_start_deg must be in [0, 360)";
        return false;
    }
    if (az_end_deg <= 0.0 || az_end_deg > 360.0) {
        error = "az_end_deg must be in (0, 360]";
        return false;
    }
    if (elevation_deg < -90.0 || elevation_deg > 90.0) {
        error = "elevation_deg must be in [-90, 90]";
        return false;
    }
    if (rotation_period_s <= 0.0) {
        error = "rotation_period_s must be positive after derived-parameter computation";
        return false;
    }
    if (azimuth_step_per_prt_deg <= 0.0) {
        error = "azimuth_step_per_prt_deg must be positive after derived-parameter computation";
        return false;
    }
    if (pulses_per_rotation <= 0) {
        error = "pulses_per_rotation must be positive after derived-parameter computation";
        return false;
    }
    return true;
}

bool AntennaConfig::validate(std::string& error) const {
    if (az_beamwidth_deg <= 0.0) {
        error = "az_beamwidth_deg must be positive";
        return false;
    }
    if (el_beamwidth_deg <= 0.0) {
        error = "el_beamwidth_deg must be positive";
        return false;
    }
    if (peak_gain_db < 0.0) {
        error = "peak_gain_db cannot be negative";
        return false;
    }
    return true;
}

}  // namespace radar::antenna
