/**
 * @file sea_clutter_config.cpp
 * @brief 海杂波模块配置参数实现
 */

#include "clutter/sea_clutter_config.h"

namespace radar::clutter {

bool SeaClutterConfig::validate(std::string& error) const {
    if (ground_range_min_m < 0.0 && ground_range_min_m != -1.0) {
        error = "ground_range_min_m must be -1 or >= 0";
        return false;
    }
    if (ground_range_max_m < 0.0 && ground_range_max_m != -1.0) {
        error = "ground_range_max_m must be -1 or >= 0";
        return false;
    }
    if (ground_range_min_m >= 0.0 && ground_range_max_m >= 0.0 &&
        ground_range_max_m <= ground_range_min_m) {
        error = "ground_range_max_m must be greater than ground_range_min_m";
        return false;
    }

    if (!math::is_finite_positive(az_grid_step_deg)) {
        error = "az_grid_step_deg must be positive and finite";
        return false;
    }
    if (!math::is_finite_positive(k_shape_nu)) {
        error = "k_shape_nu must be positive and finite";
        return false;
    }
    if (!math::is_finite_positive(doppler_sigma_hz)) {
        error = "doppler_sigma_hz must be positive and finite";
        return false;
    }
    if (!math::is_finite(doppler_center_hz)) {
        error = "doppler_center_hz must be finite";
        return false;
    }
    if (pool_length_factor <= 0) {
        error = "pool_length_factor must be positive";
        return false;
    }

    if (!std::isfinite(morchin_a0_db) || !std::isfinite(morchin_a_g) ||
        !std::isfinite(morchin_a_f) || !std::isfinite(morchin_a_s) ||
        !std::isfinite(morchin_sea_state) ||
        !math::is_finite_positive(morchin_sin_psi_floor)) {
        error = "Morchin params are invalid";
        return false;
    }

    return true;
}

}  // namespace radar::clutter
