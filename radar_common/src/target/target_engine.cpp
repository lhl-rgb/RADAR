/**
 * @file target_engine.cpp
 * @brief 点目标回波生成引擎实现
 */

#include "target/target_engine.h"
#include "core/tools/math_utils.h"

#include <random>

namespace radar::target {

namespace {

uint64_t splitmix64(uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31U);
}

Scalar draw_visit_rcs(std::mt19937_64& rng, SwerlingType swerling, Scalar mean_rcs_m2) {
    const Scalar mean_rcs = math::clamp_positive_eps(mean_rcs_m2);
    switch (swerling) {
    case SwerlingType::Swerling0:
        return mean_rcs;
    case SwerlingType::Swerling1:
    case SwerlingType::Swerling2: {
        std::exponential_distribution<Scalar> expo(1.0 / mean_rcs);
        return expo(rng);
    }
    case SwerlingType::Swerling3:
    case SwerlingType::Swerling4: {
        std::gamma_distribution<Scalar> gamma(2.0, mean_rcs / 2.0);
        return gamma(rng);
    }
    }
    return mean_rcs;
}

}  // namespace

bool TargetEngine::initialize() {
    if (initial_targets_.empty()) {
        initial_targets_ = generate_default_targets();
    }

    target_manager_.set_targets(initial_targets_);
    reset_fluctuation_states();
    initialized_ = true;
    return true;
}

void TargetEngine::set_initial_targets(const TargetList& targets) {
    initial_targets_ = targets;
    reset_fluctuation_states();
    if (initialized_) {
        target_manager_.set_targets(targets);
    }
}

void TargetEngine::update_targets(Scalar current_time_s) {
    target_manager_.update_to_time(current_time_s);
}

const TargetList& TargetEngine::get_current_targets() const {
    return target_manager_.get_current_targets();
}

void TargetEngine::clear_targets() {
    target_manager_.clear();
    reset_fluctuation_states();
}

std::size_t TargetEngine::target_count() const {
    return target_manager_.size();
}

void TargetEngine::reset_fluctuation_states() {
    fluctuation_states_.clear();
}

TargetEngine::TargetFluctuationState& TargetEngine::ensure_fluctuation_state(const TargetState& target) {
    auto [it, inserted] = fluctuation_states_.try_emplace(target.id);
    TargetFluctuationState& state = it->second;
    if (inserted || !state.rng_seeded) {
        const uint64_t seed = splitmix64(cfg_.seed ^ splitmix64(target.id));
        state.rng.seed(seed);
        state.rng_seeded = true;
        state.current_rcs_m2 = target.rcs_mean_m2;
        state.has_current_rcs = false;
        state.was_illuminated = false;
        state.visit_count = 0;
    }
    return state;
}

bool TargetEngine::is_target_illuminated(const TargetState& target,
                                         const BeamView& beam) const {
    if (beam.antenna == nullptr) {
        return true;
    }

    Scalar az_deg = 0.0;
    Scalar el_deg = 0.0;
    math::to_az_el_deg(target.position_m, az_deg, el_deg);
    return beam.antenna->is_target_in_beam(az_deg, el_deg,
                                           beam.pointing.azimuth_deg,
                                           beam.pointing.elevation_deg,
                                           cfg_.beam_gate_threshold_db);
}

Scalar TargetEngine::resolve_rcs_for_pulse(const TargetState& target,
                                           const BeamView& beam) {
    if (!cfg_.enable_swerling || target.swerling == SwerlingType::Swerling0) {
        return target.rcs_mean_m2;
    }

    TargetFluctuationState& state = ensure_fluctuation_state(target);
    const bool illuminated = is_target_illuminated(target, beam);
    const bool visit_started = illuminated && !state.was_illuminated;

    if (visit_started || !state.has_current_rcs) {
        state.current_rcs_m2 = draw_visit_rcs(state.rng, target.swerling, target.rcs_mean_m2);
        state.has_current_rcs = true;
        if (visit_started) {
            state.visit_count++;
        }
    }

    state.was_illuminated = illuminated;
    return state.current_rcs_m2;
}

bool TargetEngine::generate_pulse(const BeamView& beam,
                                  const PulseContext& pulse,
                                  const RadarSystemParams& system,
                                  const ComplexVec& tx_waveform,
                                  PulseEcho& out_echo) {
    last_error_.clear();

    if (tx_waveform.empty()) {
        last_error_ = "TargetEngine invalid input: tx_waveform must not be empty.";
        return false;
    }
    if (system.samples_per_pulse <= 0) {
        last_error_ = "RadarSystemParams invalid: samples_per_pulse must be positive.";
        return false;
    }

    out_echo.assign(static_cast<std::size_t>(system.samples_per_pulse), Complex(0.0, 0.0));

    if (!cfg_.enabled) {
        return true;
    }

    const TargetList& active_targets = get_current_targets();
    for (const TargetState& target : active_targets) {
        if (!target.enabled) {
            continue;
        }

        std::string error;
        TargetSnapshot snapshot;
        if (!TargetKinematics::generate_snapshot(target, beam, pulse, cfg_, snapshot, error)) {
            last_error_ = "TargetKinematics::generate_snapshot failed for target_id=" +
                          std::to_string(target.id) + ": " + error;
            return false;
        }

        snapshot.rcs_m2 = resolve_rcs_for_pulse(target, beam);

        if (!TargetEchoSynthesizer::generate_target_echo(snapshot, system, cfg_, tx_waveform, out_echo, error)) {
            last_error_ = "TargetEchoSynthesizer::generate_target_echo failed for target_id=" +
                          std::to_string(target.id) + ": " + error;
            return false;
        }
    }

    return true;
}

TargetList TargetEngine::generate_default_targets() const {
    return {
        {1, Vec3(50000.0, 0.0, 5000.0), Vec3::Zero(), Vec3::Zero(),
         MotionModel::Stationary, 10.0, SwerlingType::Swerling1, true},
        {2, Vec3(30000.0, 10000.0, 3000.0), Vec3(100.0, 50.0, 0.0), Vec3::Zero(),
         MotionModel::ConstantVelocity, 5.0, SwerlingType::Swerling1, true}
    };
}

}  // namespace radar::target
