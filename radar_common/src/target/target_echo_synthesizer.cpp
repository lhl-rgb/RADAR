/**
 * @file target_echo_synthesizer.cpp
 * @brief 目标真值到 CPI 回波写入与叠加实现
 */

#include "target/target_echo_synthesizer.h"
#include "core/tools/math_utils.h"

#include <cmath>

namespace radar::target {

bool TargetEchoSynthesizer::generate_target_echo(const TargetTrajectory& trajectory,
                                                  const RadarSystemParams& system,
                                                  const target::TargetConfig& config,
                                                  const ComplexVec& tx_waveform,
                                                  CpiEcho& out_echo,
                                                  std::string& error) {
    error.clear();

    if (system.pulses_per_cpi <= 0 || system.samples_per_pulse <= 0) {
        error = "RadarSystemParams invalid: pulses_per_cpi and samples_per_pulse must be positive.";
        return false;
    }
    if (tx_waveform.empty()) {
        error = "TargetEchoSynthesizer invalid input: tx_waveform must not be empty.";
        return false;
    }
    if (trajectory.snapshots.size() != static_cast<std::size_t>(system.pulses_per_cpi)) {
        error = "TargetTrajectory size mismatch with system.pulses_per_cpi.";
        return false;
    }
    if (out_echo.pulses.size() != static_cast<std::size_t>(system.pulses_per_cpi)) {
        error = "Output CpiEcho pulse count mismatch with system.pulses_per_cpi.";
        return false;
    }

    const Scalar tau_ref_s = 2.0 * system.min_range_m / C;
    const Scalar fs_hz = math::clamp_positive_eps(system.fs_hz);
    const Scalar lambda_m = math::clamp_positive_eps(system.wavelength_m);

    for (int pulse = 0; pulse < system.pulses_per_cpi; ++pulse) {
        const TargetSnapshot& sample = trajectory.snapshots[static_cast<std::size_t>(pulse)];

        if (sample.gain_linear <= 0.0 || sample.rcs_m2 <= 0.0) {
            continue;
        }

        const Scalar delay_s = (2.0 * sample.range_m) / C;
        const Scalar fast_time_s = delay_s - tau_ref_s;
        const int delay_idx = static_cast<int>(std::llround(fast_time_s * fs_hz));

        const Scalar gain_linear = math::clamp_nonnegative(sample.gain_linear);
        const Scalar rcs_m2 = math::clamp_positive_eps(sample.rcs_m2);
        const Scalar current_range_m = sample.position_m.norm();

        Scalar amp = std::sqrt(gain_linear * rcs_m2);

        if (config.enable_two_way_propagation_loss) {
            const Scalar numerator =
                system.peak_power_w * gain_linear * gain_linear * lambda_m * lambda_m * rcs_m2;
            const Scalar denominator =
                std::pow(4.0 * PI, 3.0) * std::pow(current_range_m, 4.0) *
                math::clamp_positive_eps(system.system_loss_linear);
            amp = std::sqrt(math::safe_div(numerator, denominator));
        }

        Scalar phase = 0.0;
        if (config.enable_phase) {
            phase = -4.0 * PI * current_range_m / lambda_m;
        }

        const Complex phasor(std::cos(phase), std::sin(phase));
        const Complex coeff = amp * phasor;

        PulseEcho& pulse_echo = out_echo.pulses[static_cast<std::size_t>(pulse)];
        for (std::size_t n = 0; n < tx_waveform.size(); ++n) {
            const int sample_idx = delay_idx + static_cast<int>(n);
            if (sample_idx < 0 || sample_idx >= system.samples_per_pulse) {
                continue;
            }
            pulse_echo[static_cast<std::size_t>(sample_idx)] += coeff * tx_waveform[n];
        }
    }

    return true;
}

bool TargetEchoSynthesizer::generate_target_echoes(const TrajectoryBatch& trajectories,
                                                    const RadarSystemParams& system,
                                                    const target::TargetConfig& config,
                                                    const ComplexVec& tx_waveform,
                                                    CpiEcho& out_echo,
                                                    std::string& error) {
    error.clear();
    for (const TargetTrajectory& trajectory : trajectories) {
        if (!generate_target_echo(trajectory, system, config, tx_waveform, out_echo, error)) {
            error = "TargetEchoSynthesizer::generate_target_echoes failed for target_id=" +
                    std::to_string(trajectory.target_id) + ": " + error;
            return false;
        }
    }
    return true;
}

}  // namespace radar