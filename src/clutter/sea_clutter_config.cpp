/**
 * @file sea_clutter_config.cpp
 * @brief 海杂波配置参数实现
 */

#include "clutter/sea_clutter_config.h"
#include "core/math_utils.h"

namespace radar::clutter {

bool SeaClutterConfig::validate(std::string& error) const {
    if (!math::is_finite(ground_range_min_m) ||
        !math::is_finite(ground_range_max_m) ||
        !math::is_finite(range_step_m) ||
        !math::is_finite(beam_az_width_deg) ||
        !math::is_finite(az_step_deg) ||
        !math::is_finite(k_shape_nu) ||
        !math::is_finite(doppler_center_hz) ||
        !math::is_finite(doppler_sigma_hz) ||
        !math::is_finite(morchin.a0_db) ||
        !math::is_finite(morchin.a_g) ||
        !math::is_finite(morchin.a_f) ||
        !math::is_finite(morchin.a_s) ||
        !math::is_finite(morchin.sea_state) ||
        !math::is_finite(morchin.sin_psi_floor)) {
        error = "all parameters must be finite";
        return false;
    }

    if (range_step_m <= 0.0 ||
        beam_az_width_deg <= 0.0 ||
        az_step_deg <= 0.0 ||
        k_shape_nu <= 0.0 ||
        doppler_sigma_hz <= 0.0 ||
        pool_length_factor <= 0 ||
        morchin.sin_psi_floor <= 0.0) {
        error = "range_step, beam_az_width, az_step, k_shape_nu, doppler_sigma, pool_length_factor, and sin_psi_floor must be positive";
        return false;
    }

    return true;
}

}  // namespace radar::clutter