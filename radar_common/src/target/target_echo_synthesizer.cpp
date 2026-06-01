/**
 * @file target_echo_synthesizer.cpp
 * @brief 目标真值到单脉冲回波写入与叠加实现
 */

#include "target/target_echo_synthesizer.h"
#include "core/tools/math_utils.h"

#include <cmath>

namespace radar::target
{

    bool TargetEchoSynthesizer::generate_target_echo(const TargetSnapshot &snapshot,
                                                     const RadarSystemParams &system,
                                                     const target::TargetConfig &config,
                                                     const ComplexVec &tx_waveform,
                                                     PulseEcho &out_echo,
                                                     std::string &error)
    {
        error.clear();

        if (system.samples_per_pulse <= 0)
        {
            error = "RadarSystemParams invalid: samples_per_pulse must be positive.";
            return false;
        }
        if (tx_waveform.empty())
        {
            error = "TargetEchoSynthesizer invalid input: tx_waveform must not be empty.";
            return false;
        }
        if (out_echo.size() != static_cast<std::size_t>(system.samples_per_pulse))
        {
            error = "Output PulseEcho size mismatch with system.samples_per_pulse.";
            return false;
        }

        if (snapshot.gain_linear <= 0.0 || snapshot.rcs_m2 <= 0.0)
        {
            return true;
        }

        const Scalar tau_ref_s = 2.0 * system.min_range_m / C;
        const Scalar fs_hz = math::clamp_positive_eps(system.fs_hz);
        const Scalar lambda_m = math::clamp_positive_eps(system.wavelength_m);
        const Scalar delay_s = (2.0 * snapshot.range_m) / C;
        const Scalar fast_time_s = delay_s - tau_ref_s;
        const int delay_idx = static_cast<int>(std::llround(fast_time_s * fs_hz));

        const Scalar gain_linear = math::clamp_nonnegative(snapshot.gain_linear);
        const Scalar rcs_m2 = math::clamp_positive_eps(snapshot.rcs_m2);
        const Scalar current_range_m = snapshot.position_m.norm();

        Scalar amp = std::sqrt(gain_linear * rcs_m2);
        if (config.enable_two_way_propagation_loss)
        {
            const Scalar numerator =
                system.peak_power_w * gain_linear * gain_linear * lambda_m * lambda_m * rcs_m2;
            const Scalar denominator =
                std::pow(4.0 * PI, 3.0) * std::pow(current_range_m, 4.0) *
                math::clamp_positive_eps(system.system_loss_linear);
            amp = std::sqrt(math::safe_div(numerator, denominator));
        }

        Scalar phase = 0.0;
        if (config.enable_phase)
        {
            phase = -4.0 * PI * current_range_m / lambda_m;
        }

        const Complex phasor(std::cos(phase), std::sin(phase));
        const Complex coeff = amp * phasor;

        for (std::size_t n = 0; n < tx_waveform.size(); ++n)
        {
            const int sample_idx = delay_idx + static_cast<int>(n);
            if (sample_idx < 0 || sample_idx >= system.samples_per_pulse)
            {
                continue;
            }
            out_echo[static_cast<std::size_t>(sample_idx)] += coeff * tx_waveform[n];
        }

        return true;
    }

} // namespace radar::target
