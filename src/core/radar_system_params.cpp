/**
 * @file radar_system_params.cpp
 * @brief RadarSystemParams implementation - global shared radar parameters
 */

#include "core/radar_system_params.h"
#include "core/math_utils.h"

#include <algorithm>
#include <cmath>
#include <iostream>

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
    const int safe_pulses_per_cpi = std::max(1, pulses_per_cpi);

    // Wavelength and timing
    wavelength_m = C / safe_fc;
    pri_s = 1.0 / safe_prf;
    duty_cycle = safe_pulse_width / pri_s;
    cpi_duration_s = static_cast<Scalar>(safe_pulses_per_cpi) / safe_prf;

    // Power and energy
    average_power_w = safe_peak_power * duty_cycle;
    energy_per_pulse_j = safe_peak_power * safe_pulse_width;
    bandwidth_time_product = safe_bw * safe_pulse_width;

    // Resolution
    range_resolution_m = C / (2.0 * safe_bw);
    range_bin_size_m = C / (2.0 * safe_fs);
    velocity_resolution_mps = wavelength_m * safe_prf / (2.0 * static_cast<Scalar>(safe_pulses_per_cpi));
    velocity_bin_size_mps = wavelength_m / (2.0 * cpi_duration_s);

    // Unambiguous limits
    max_unambiguous_range_m = C / (2.0 * safe_prf);
    max_unambiguous_velocity_mps = wavelength_m * safe_prf / 4.0;
    max_doppler_hz = safe_prf / 2.0;

    // Samples per pulse (fast time window)
    const Scalar range_span_m = math::clamp_nonnegative(max_range_m - min_range_m);
    const Scalar fast_time_window_s = (2.0 * range_span_m / C) + safe_pulse_width;
    samples_per_pulse = std::max(1, static_cast<int>(std::ceil(fast_time_window_s * safe_fs)));

    // Linear values
    noise_figure_linear = math::db_to_linear(noise_figure_db);
    system_loss_linear = math::db_to_linear(system_loss_db);
}

bool RadarSystemParams::validate(std::string& error) const {
    // Basic physical parameters
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
    if (pulses_per_cpi <= 0) {
        error = "pulses_per_cpi must be positive";
        return false;
    }

    // System losses
    if (noise_figure_db < 0.0) {
        error = "noise_figure_db cannot be negative";
        return false;
    }
    if (system_loss_db < 0.0) {
        error = "system_loss_db cannot be negative";
        return false;
    }

    // Scene parameters
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

    // Radar location
    if (!math::is_finite(radar_location.latitude) ||
        !math::is_finite(radar_location.longitude) ||
        !math::is_finite(radar_location.altitude)) {
        error = "radar_location contains non-finite values";
        return false;
    }

    // Pulse width vs PRI constraint
    if (pulse_width_s > 1.0 / prf_hz) {
        error = "pulse_width_s exceeds PRI (duty cycle > 100%)";
        return false;
    }

    // Sampling constraint
    if (fs_hz < bw_hz) {
        error = "fs_hz should be >= bw_hz for complex baseband sampling";
        return false;
    }

    return true;
}

void RadarSystemParams::print() const {
    std::cout
        << "RadarSystemParams:\n"
        << "  ===== Basic Physical Parameters =====\n"
        << "  fc_hz: " << fc_hz << "\n"
        << "  prf_hz: " << prf_hz << "\n"
        << "  fs_hz: " << fs_hz << "\n"
        << "  bw_hz: " << bw_hz << "\n"
        << "  pulse_width_s: " << pulse_width_s << "\n"
        << "  peak_power_w: " << peak_power_w << "\n"
        << "  pulses_per_cpi: " << pulses_per_cpi << "\n"
        << "\n"
        << "  ===== Scene Parameters =====\n"
        << "  min_range_m: " << min_range_m << "\n"
        << "  max_range_m: " << max_range_m << "\n"
        << "  radar_location: (" << radar_location.latitude << ", "
        << radar_location.longitude << ", " << radar_location.altitude << ")\n"
        << "  antenna_height_m: " << antenna_height_m << "\n"
        << "\n"
        << "  ===== System Losses =====\n"
        << "  noise_figure_db: " << noise_figure_db << "\n"
        << "  system_loss_db: " << system_loss_db << "\n"
        << "\n"
        << "  ===== Derived Parameters =====\n"
        << "  wavelength_m: " << wavelength_m << "\n"
        << "  pri_s: " << pri_s << "\n"
        << "  duty_cycle: " << duty_cycle << "\n"
        << "  cpi_duration_s: " << cpi_duration_s << "\n"
        << "  average_power_w: " << average_power_w << "\n"
        << "  energy_per_pulse_j: " << energy_per_pulse_j << "\n"
        << "  bandwidth_time_product: " << bandwidth_time_product << "\n"
        << "  range_resolution_m: " << range_resolution_m << "\n"
        << "  range_bin_size_m: " << range_bin_size_m << "\n"
        << "  velocity_resolution_mps: " << velocity_resolution_mps << "\n"
        << "  velocity_bin_size_mps: " << velocity_bin_size_mps << "\n"
        << "  max_unambiguous_range_m: " << max_unambiguous_range_m << "\n"
        << "  max_unambiguous_velocity_mps: " << max_unambiguous_velocity_mps << "\n"
        << "  max_doppler_hz: " << max_doppler_hz << "\n"
        << "  samples_per_pulse: " << samples_per_pulse << "\n"
        << "  noise_figure_linear: " << noise_figure_linear << "\n"
        << "  system_loss_linear: " << system_loss_linear << "\n";
}

}  // namespace radar