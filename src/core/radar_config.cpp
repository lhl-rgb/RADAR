/**
 * @file radar_config.cpp
 * @brief 雷达配置组合容器实现
 */

#include "core/radar_config.h"

#include <iostream>

namespace radar {

RadarConfig::RadarConfig() {
    compute_derived_params();
}

void RadarConfig::compute_derived_params() {
    system.compute_derived_params();
}

bool RadarConfig::validate(std::string& error) const {
    // Validate each module config
    if (!system.validate(error)) {
        return false;
    }
    if (!waveform.validate(error)) {
        return false;
    }
    if (!antenna.validate(error)) {
        return false;
    }
    if (!noise.validate(error)) {
        return false;
    }
    if (!clutter.validate(error)) {
        return false;
    }
    if (!target.validate(error)) {
        return false;
    }

    // Cross-module validation
    return validate_full(error);
}

bool RadarConfig::validate_full(std::string& error) const {
    // Pulse width < PRI
    if (system.pulse_width_s > system.pri_s) {
        error = "pulse_width exceeds PRI";
        return false;
    }

    // Sampling rate >= bandwidth
    if (system.fs_hz < system.bw_hz) {
        error = "fs must be >= bw for complex baseband modeling";
        return false;
    }

    // Clutter doppler center within unambiguous range
    const Scalar clutter_nyquist_hz = 0.5 * system.prf_hz;
    if (clutter.doppler_center_hz < -clutter_nyquist_hz ||
        clutter.doppler_center_hz >= clutter_nyquist_hz) {
        error = "clutter doppler_center must be within [-prf/2, prf/2)";
        return false;
    }

    // Clutter range within radar range
    const Scalar clutter_range_min =
        (clutter.ground_range_min_m < 0.0) ? system.min_range_m : clutter.ground_range_min_m;
    const Scalar clutter_range_max =
        (clutter.ground_range_max_m < 0.0) ? system.max_range_m : clutter.ground_range_max_m;
    if (clutter_range_min < 0.0 || clutter_range_max <= clutter_range_min) {
        error = "clutter range invalid (min >= 0, max > min)";
        return false;
    }

    return true;
}

void RadarConfig::print() const {
    std::cout << "=== RadarConfig ===\n\n";
    system.print();
    std::cout << "\n--- WaveformConfig ---\n"
              << "  waveform_type: " << static_cast<int>(waveform.waveform_type) << "\n"
              << "  phase_code_type: " << static_cast<int>(waveform.phase_code_type) << "\n"
              << "  nlfm_window_type: " << static_cast<int>(waveform.nlfm_window_type) << "\n"
              << "  polarization: " << static_cast<int>(waveform.polarization) << "\n";
    std::cout << "\n--- AntennaConfig ---\n"
              << "  model_type: " << static_cast<int>(antenna.model_type) << "\n"
              << "  num_elements_az: " << antenna.num_elements_az << "\n"
              << "  num_elements_el: " << antenna.num_elements_el << "\n"
              << "  peak_gain_db: " << antenna.peak_gain_db << "\n";
    std::cout << "\n--- NoiseConfig ---\n"
              << "  mode: " << static_cast<int>(noise.mode) << "\n"
              << "  sigma_complex: " << noise.sigma_complex << "\n"
              << "  seed: " << noise.seed << "\n";
    std::cout << "\n--- SeaClutterConfig ---\n"
              << "  enabled: " << (clutter.enabled ? "true" : "false") << "\n"
              << "  k_shape_nu: " << clutter.k_shape_nu << "\n"
              << "  morchin.sea_state: " << clutter.morchin.sea_state << "\n";
    std::cout << "\n--- TargetConfig ---\n"
              << "  enabled: " << (target.enabled ? "true" : "false") << "\n"
              << "  enable_swerling: " << (target.enable_swerling ? "true" : "false") << "\n";
}

}  // namespace radar