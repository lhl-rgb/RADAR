// include/core/radar_system_params.h
#pragma once

#include <spdlog/spdlog.h>
#include <string>

#include "types.h"
#include "math_utils.h"
#include "core/udp/udp_config.h"

namespace radar {

/**
 * @brief 全局共享雷达系统参数
 * @details 所有模块都依赖的基础物理参数和派生参数
 */
struct RadarSystemParams {
    // ===== 基础物理参数 =====
    Scalar fc_hz = 10.0e9;              ///< 载频 (Hz)
    Scalar prf_hz = 1600.0;             ///< 脉冲重复频率 (Hz)
    Scalar fs_hz = 40.0e6;              ///< ADC采样率 (Hz)
    Scalar bw_hz = 20.0e6;              ///< 信号带宽 (Hz)
    Scalar pulse_width_s = 20.0e-6;     ///< 脉冲宽度 (s)
    Scalar peak_power_w = 5000.0;       ///< 峰值发射功率 (W)
    int pulses_per_cpi = 32;            ///< 每 CPI 脉冲数

    // ===== 场景参数 =====
    Scalar min_range_m = 1000.0;        ///< 最小作用距离 (m)
    Scalar max_range_m = 120000.0;      ///< 最大作用距离 (m)
    Scalar antenna_height_m = 15.0;     ///< 天线安装高度 (m)
    GeoCoord radar_location{0.0, 0.0, antenna_height_m}; ///< 雷达地理位置

    // ===== 系统损耗 =====
    Scalar noise_figure_db = 4.0;       ///< 接收机噪声系数 (dB)
    Scalar system_loss_db = 6.0;        ///< 系统总损耗 (dB)

    // ===== 派生参数（由 compute_derived_params 计算） =====
    Scalar wavelength_m;                ///< 波长 = C / fc
    Scalar pri_s;                       ///< 脉冲重复间隔 = 1 / prf
    Scalar duty_cycle;                  ///< 占空比 = pulse_width / pri
    Scalar cpi_duration_s;              ///< CPI 时长 = pulses_per_cpi / prf
    Scalar average_power_w;             ///< 平均功率 = peak_power * duty_cycle
    Scalar energy_per_pulse_j;          ///< 每脉冲能量 = peak_power * pulse_width
    Scalar bandwidth_time_product;      ///< 时带宽积 = bw * pulse_width

    Scalar range_resolution_m;          ///< 距离分辨率 = C / (2 * bw)
    Scalar range_bin_size_m;            ///< 距离 bin = C / (2 * fs)
    Scalar velocity_resolution_mps;     ///< 速度分辨率 = λ * prf / (2 * N)
    Scalar velocity_bin_size_mps;       ///< 速度 bin = λ / (2 * cpi_duration)

    Scalar max_unambiguous_range_m;     ///< 最大非模糊距离 = C / (2 * prf)
    Scalar max_unambiguous_velocity_mps;///< 最大非模糊速度 = λ * prf / 4
    Scalar max_doppler_hz;              ///< 最大非模糊多普勒 = prf / 2

    int samples_per_pulse;              ///< 每脉冲采样点数

    Scalar noise_figure_linear;         ///< 噪声系数线性值
    Scalar system_loss_linear;          ///< 系统损耗线性值

    RadarSystemParams();

    void compute_derived_params();
    bool validate(std::string& error) const;
    void print() const;
};

inline RadarSystemParams::RadarSystemParams() {
    compute_derived_params();
}

inline void RadarSystemParams::compute_derived_params() {
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

inline bool RadarSystemParams::validate(std::string& error) const {
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

inline void RadarSystemParams::print() const {
    SPDLOG_INFO("RadarSystemParams:");
    SPDLOG_INFO("  ===== Basic Physical Parameters =====");
    SPDLOG_INFO("  fc_hz: {}", fc_hz);
    SPDLOG_INFO("  prf_hz: {}", prf_hz);
    SPDLOG_INFO("  fs_hz: {}", fs_hz);
    SPDLOG_INFO("  bw_hz: {}", bw_hz);
    SPDLOG_INFO("  pulse_width_s: {}", pulse_width_s);
    SPDLOG_INFO("  peak_power_w: {}", peak_power_w);
    SPDLOG_INFO("  pulses_per_cpi: {}", pulses_per_cpi);
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
    SPDLOG_INFO("  cpi_duration_s: {}", cpi_duration_s);
    SPDLOG_INFO("  average_power_w: {}", average_power_w);
    SPDLOG_INFO("  energy_per_pulse_j: {}", energy_per_pulse_j);
    SPDLOG_INFO("  bandwidth_time_product: {}", bandwidth_time_product);
    SPDLOG_INFO("  range_resolution_m: {}", range_resolution_m);
    SPDLOG_INFO("  range_bin_size_m: {}", range_bin_size_m);
    SPDLOG_INFO("  velocity_resolution_mps: {}", velocity_resolution_mps);
    SPDLOG_INFO("  velocity_bin_size_mps: {}", velocity_bin_size_mps);
    SPDLOG_INFO("  max_unambiguous_range_m: {}", max_unambiguous_range_m);
    SPDLOG_INFO("  max_unambiguous_velocity_mps: {}", max_unambiguous_velocity_mps);
    SPDLOG_INFO("  max_doppler_hz: {}", max_doppler_hz);
    SPDLOG_INFO("  samples_per_pulse: {}", samples_per_pulse);
    SPDLOG_INFO("  noise_figure_linear: {}", noise_figure_linear);
    SPDLOG_INFO("  system_loss_linear: {}", system_loss_linear);
}


}  // namespace radar