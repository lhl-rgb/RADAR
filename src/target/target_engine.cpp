/**
 * @file target_engine.cpp
 * @brief 点目标回波生成引擎实现
 */

#include "target/target_engine.h"
#include "target/target_echo_synthesizer.h"
#include "target/target_kinematics.h"
#include "core/math_utils.h"

#include <stdexcept>

namespace radar {

bool TargetEngine::init_output_echo(const BeamView& beam,
                                    const RadarParams& radar,
                                    CpiEcho& out_echo,
                                    std::string& error) const {
    error.clear();

    if (radar.pulses_per_cpi <= 0 || radar.samples_per_pulse <= 0) {
        error = "RadarParams invalid: pulses_per_cpi and samples_per_pulse must be positive.";
        return false;
    }

    out_echo.beam_index = beam.beam_index;
    out_echo.azimuth_deg = beam.pointing.azimuth;
    out_echo.elevation_deg = beam.pointing.elevation;
    out_echo.pulses.assign(static_cast<std::size_t>(radar.pulses_per_cpi),
                           PulseEcho(static_cast<std::size_t>(radar.samples_per_pulse), Complex(0.0, 0.0)));
    return true;
}

bool TargetEngine::generate(const TargetList& active_targets,
                            const BeamView& beam,
                            const RadarParams& radar,
                            const ComplexVec& tx_waveform,
                            CpiEcho& out_echo) {
    last_error_.clear();

    if (!radar.validate()) {
        last_error_ = "RadarParams invalid: validate() failed.";
        return false;
    }
    if (tx_waveform.empty()) {
        last_error_ = "TargetEngine invalid input: tx_waveform must not be empty.";
        return false;
    }

    std::string error;
    if (!init_output_echo(beam, radar, out_echo, error)) {
        last_error_ = error;
        return false;
    }

    if (!radar.target_params.enabled) {
        return true;
    }

    for (const TargetState& target : active_targets) {
        if (!target.enabled) {
            continue;
        }

        TargetTrajectory  trajectory;
        if (!TargetKinematics::generate_trajectory(target, beam, radar, trajectory, error)) {
            last_error_ = error;
            return false;
        }

        if (!TargetEchoSynthesizer::generate_target_echo(trajectory,
                                                           radar,
                                                           tx_waveform,
                                                           out_echo,
                                                           error)) {
            last_error_ = error;
            return false;
        }
    }

    return true;
}


CpiEcho TargetEngine::generate_or_throw(const TargetList& active_targets,
                                        const BeamView& beam,
                                        const RadarParams& radar,
                                        const ComplexVec& tx_waveform) {
    CpiEcho echo;
    if (!generate(active_targets, beam, radar, tx_waveform, echo)) {
        throw std::runtime_error(last_error_);
    }
    return echo;
}


}  // namespace radar
