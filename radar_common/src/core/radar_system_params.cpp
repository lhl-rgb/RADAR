/**
 * @file radar_system_params.cpp
 * @brief 雷达系统参数实现
 */

#include "core/radar_system_params.h"
#include "core/tools/math_utils.h"

#include <spdlog/spdlog.h>

namespace radar {

RadarSystemParams::RadarSystemParams() {
    compute_derived_params();
}

void RadarSystemParams::compute_derived_params() {
    const Scalar safe_fc = math::clamp_positive_eps(fc_hz);
    const Scalar safe_prf = math::clamp_positive_eps(prf_hz);
    const Scalar safe_fs = math::clamp_positive_eps(fs_hz);
    const Scalar safe_bw = math::clamp_positive_eps(bw_hz);
    const Scalar safe_pulse_width = math::clamp_nonnegative(pulse_width_s);
    const Scalar safe_peak_power = math::clamp_nonnegative(peak_power_w);

    wavelength_m = C / safe_fc;
    pri_s = 1.0 / safe_prf;
    duty_cycle = safe_pulse_width / pri_s;

    average_power_w = safe_peak_power * duty_cycle;
    energy_per_pulse_j = safe_peak_power * safe_pulse_width;
    bandwidth_time_product = safe_bw * safe_pulse_width;

    range_resolution_m = C / (2.0 * safe_bw);
    range_bin_size_m = C / (2.0 * safe_fs);

    max_unambiguous_range_m = C / (2.0 * safe_prf);
    max_unambiguous_velocity_mps = wavelength_m * safe_prf / 4.0;
    max_doppler_hz = safe_prf / 2.0;

    const Scalar range_span_m = math::clamp_nonnegative(max_range_m - min_range_m);
    const Scalar fast_time_window_s = (2.0 * range_span_m / C) + safe_pulse_width;
    samples_per_pulse = std::max(1, static_cast<int>(std::ceil(fast_time_window_s * safe_fs)));
    samples_per_tx = std::max(1, static_cast<int>(std::round(safe_pulse_width * safe_fs)));

    noise_figure_linear = math::db_to_linear(noise_figure_db);
    system_loss_linear = math::db_to_linear(system_loss_db);
}

bool RadarSystemParams::validate(std::string& error) const {
    if (fc_hz <= 0.0) {
        error = "fc_hz must be positive";
        return false;
    }
    if (prf_hz <= 0.0) {
        error = "prf_hz must be positive";
        return false;
    }
    if (fs_hz <= 0.0) {
        error = "fs_hz must be positive";
        return false;
    }
    if (bw_hz <= 0.0) {
        error = "bw_hz must be positive";
        return false;
    }
    if (pulse_width_s <= 0.0) {
        error = "pulse_width_s must be positive";
        return false;
    }
    if (peak_power_w <= 0.0) {
        error = "peak_power_w must be positive";
        return false;
    }

    if (noise_figure_db < 0.0) {
        error = "noise_figure_db cannot be negative";
        return false;
    }
    if (system_loss_db < 0.0) {
        error = "system_loss_db cannot be negative";
        return false;
    }

    if (min_range_m < 0.0) {
        error = "min_range_m cannot be negative";
        return false;
    }
    if (max_range_m <= min_range_m) {
        error = "max_range_m must be greater than min_range_m";
        return false;
    }
    if (antenna_height_m < 0.0) {
        error = "antenna_height_m cannot be negative";
        return false;
    }

    if (!math::is_finite(radar_location.latitude) ||
        !math::is_finite(radar_location.longitude) ||
        !math::is_finite(radar_location.altitude)) {
        error = "radar_location contains non-finite values";
        return false;
    }

    if (pulse_width_s > 1.0 / prf_hz) {
        error = "pulse_width_s exceeds PRI (duty cycle > 100%)";
        return false;
    }

    if (fs_hz < bw_hz) {
        error = "fs_hz should be >= bw_hz for complex baseband sampling";
        return false;
    }

    return true;
}

void RadarSystemParams::print() const {
    SPDLOG_INFO("RadarSystemParams:");
    SPDLOG_INFO("  ===== Basic Physical Parameters =====");
    SPDLOG_INFO("  fc_hz: {}", fc_hz);
    SPDLOG_INFO("  prf_hz: {}", prf_hz);
    SPDLOG_INFO("  fs_hz: {}", fs_hz);
    SPDLOG_INFO("  bw_hz: {}", bw_hz);
    SPDLOG_INFO("  pulse_width_s: {}", pulse_width_s);
    SPDLOG_INFO("  peak_power_w: {}", peak_power_w);
    SPDLOG_INFO("");
    SPDLOG_INFO("  ===== Scene Parameters =====");
    SPDLOG_INFO("  min_range_m: {}", min_range_m);
    SPDLOG_INFO("  max_range_m: {}", max_range_m);
    SPDLOG_INFO("  radar_location: ({}, {}, {})", radar_location.latitude, radar_location.longitude, radar_location.altitude);
    SPDLOG_INFO("  antenna_height_m: {}", antenna_height_m);
    SPDLOG_INFO("");
    SPDLOG_INFO("  ===== System Losses =====");
    SPDLOG_INFO("  noise_figure_db: {}", noise_figure_db);
    SPDLOG_INFO("  system_loss_db: {}", system_loss_db);
    SPDLOG_INFO("");
    SPDLOG_INFO("  ===== Derived Parameters =====");
    SPDLOG_INFO("  wavelength_m: {}", wavelength_m);
    SPDLOG_INFO("  pri_s: {}", pri_s);
    SPDLOG_INFO("  duty_cycle: {}", duty_cycle);
    SPDLOG_INFO("  average_power_w: {}", average_power_w);
    SPDLOG_INFO("  energy_per_pulse_j: {}", energy_per_pulse_j);
    SPDLOG_INFO("  bandwidth_time_product: {}", bandwidth_time_product);
    SPDLOG_INFO("  range_resolution_m: {}", range_resolution_m);
    SPDLOG_INFO("  range_bin_size_m: {}", range_bin_size_m);
    SPDLOG_INFO("  max_unambiguous_range_m: {}", max_unambiguous_range_m);
    SPDLOG_INFO("  max_unambiguous_velocity_mps: {}", max_unambiguous_velocity_mps);
    SPDLOG_INFO("  max_doppler_hz: {}", max_doppler_hz);
    SPDLOG_INFO("  samples_per_pulse: {}", samples_per_pulse);
    SPDLOG_INFO("  samples_per_tx: {}", samples_per_tx);
    SPDLOG_INFO("  noise_figure_linear: {}", noise_figure_linear);
    SPDLOG_INFO("  system_loss_linear: {}", system_loss_linear);
}

}  // namespace radar
