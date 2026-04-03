/**
 * @file target_kinematics.cpp
 * @brief 目标逐脉冲运动学与几何量计算实现
 */

#include "target/target_kinematics.h"
#include "core/math_utils.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace radar::target {

namespace {

bool validate_target_state(const TargetState& target, std::string& error) {
    if (!math::is_vec3_finite(target.position_m)) {
        error = "TargetState invalid: position_m must be finite (contains inf/nan).";
        return false;
    }
    if (!math::is_vec3_finite(target.velocity_mps)) {
        error = "TargetState invalid: velocity_mps must be finite.";
        return false;
    }
    if (!math::is_vec3_finite(target.acceleration_mps2)) {
        error = "TargetState invalid: acceleration_mps2 must be finite.";
        return false;
    }
    if (!math::is_finite_positive(target.rcs_mean_m2)) {
        error = "TargetState invalid: rcs_mean_m2 must be positive and finite.";
        return false;
    }
    return true;
}

Vec3 position_st(const TargetState& target, Scalar t_s) {
    switch (target.motion_model) {
    case MotionModel::Stationary:
        return target.position_m;
    case MotionModel::ConstantVelocity:
        return target.position_m + target.velocity_mps * t_s;
    case MotionModel::ConstantAcceleration:
    case MotionModel::VariableAcceleration:
        return target.position_m + target.velocity_mps * t_s +
               0.5 * target.acceleration_mps2 * t_s * t_s;
    }
    return target.position_m;
}

Vec3 velocity_st(const TargetState& target, Scalar t_s) {
    switch (target.motion_model) {
    case MotionModel::Stationary:
        return Vec3::Zero();
    case MotionModel::ConstantVelocity:
        return target.velocity_mps;
    case MotionModel::ConstantAcceleration:
    case MotionModel::VariableAcceleration:
        return target.velocity_mps + target.acceleration_mps2 * t_s;
    }
    return target.velocity_mps;
}

Vec3 acceleration_st(const TargetState& target, Scalar t_s) {
    switch (target.motion_model) {
    case MotionModel::Stationary:
    case MotionModel::ConstantVelocity:
        return Vec3::Zero();
    case MotionModel::ConstantAcceleration:
    case MotionModel::VariableAcceleration:
        return target.acceleration_mps2;
    }
    return Vec3::Zero();
}

uint64_t splitmix64(uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31U);
}

uint64_t make_target_seed(uint64_t engine_seed, uint64_t target_id, int beam_index) {
    uint64_t seed = splitmix64(engine_seed);
    seed ^= splitmix64(target_id + 0x9e3779b97f4a7c15ULL);
    seed ^= splitmix64(static_cast<uint64_t>(static_cast<uint32_t>(beam_index)));
    return splitmix64(seed);
}

Scalar draw_rcs(std::mt19937_64& rng, SwerlingType swerling, Scalar mean_rcs_m2) {
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

ScalarVector generate_rcs_series(SwerlingType swerling,
                                int pulses_per_cpi,
                                Scalar rcs_mean_m2,
                                uint64_t seed,
                                uint64_t target_id,
                                int beam_index) {
    ScalarVector sigma_series(static_cast<std::size_t>(pulses_per_cpi), rcs_mean_m2);
    if (swerling != SwerlingType::Swerling0) {
        std::mt19937_64 rng(make_target_seed(seed, target_id, beam_index));
        bool slow_fluctuation =
                (swerling == SwerlingType::Swerling1 || swerling == SwerlingType::Swerling3);
        if (slow_fluctuation) {
            const Scalar sigma = draw_rcs(rng, swerling, rcs_mean_m2);
            std::fill(sigma_series.begin(), sigma_series.end(), sigma);
        } else {
            for (std::size_t i = 0; i < sigma_series.size(); ++i) {
                sigma_series[i] = draw_rcs(rng, swerling, rcs_mean_m2);
            }
        }
    }
    return sigma_series;
}

}  // namespace

bool TargetKinematics::generate_trajectory(const TargetState& target,
                                           const BeamView& beam,
                                           const RadarSystemParams& system,
                                           const target::TargetConfig& config,
                                           TargetTrajectory& out_trajectory,
                                           std::string& error) {
    error.clear();

    if (!validate_target_state(target, error)) {
        return false;
    }
    if (system.pulses_per_cpi <= 0) {
        error = "RadarSystemParams invalid: pulses_per_cpi must be positive.";
        return false;
    }

    const Scalar pri_s = 1.0 / math::clamp_positive_eps(system.prf_hz);
    const int num_pulses = system.pulses_per_cpi;

    out_trajectory.target_id = target.id;
    out_trajectory.snapshots.assign(static_cast<std::size_t>(num_pulses), TargetSnapshot{});

    const ScalarVector rcs_series = generate_rcs_series(target.swerling,
                                                        num_pulses,
                                                        target.rcs_mean_m2,
                                                        config.seed,
                                                        target.id,
                                                        beam.beam_index);

    for (int pulse_idx = 0; pulse_idx < num_pulses; ++pulse_idx) {
        TargetSnapshot snapshot;
        const Scalar t_s = static_cast<Scalar>(pulse_idx) * pri_s;

        snapshot.slow_time_s = t_s;
        snapshot.position_m = position_st(target, t_s);
        snapshot.velocity_mps = velocity_st(target, t_s);
        snapshot.acceleration_mps2 = acceleration_st(target, t_s);

        const Scalar range_m = snapshot.position_m.norm();
        if (!math::is_finite_positive(range_m)) {
            error = "Target position invalid: range must be positive and finite.";
            return false;
        }

        snapshot.range_m = range_m;
        snapshot.radial_velocity_mps = snapshot.position_m.dot(snapshot.velocity_mps) / range_m;
        snapshot.radial_acceleration_mps2 = snapshot.position_m.dot(snapshot.acceleration_mps2) / range_m;

        if (beam.antenna != nullptr && config.enable_beam_gain) {
            Scalar az_deg = 0.0;
            Scalar el_deg = 0.0;
            math::to_az_el_deg(snapshot.position_m, az_deg, el_deg);
            if (config.skip_out_of_beam_targets && !beam.antenna->is_target_in_beam(az_deg, el_deg,
                beam.pointing.azimuth_deg, beam.pointing.elevation_deg, config.beam_gate_threshold_db)) {
                snapshot.gain_linear = 0.0;
            } else {
                snapshot.gain_linear = beam.antenna->gain(az_deg, el_deg, beam.pointing.azimuth_deg, beam.pointing.elevation_deg);
            }
        }

        snapshot.rcs_m2 = rcs_series[static_cast<std::size_t>(pulse_idx)];

        out_trajectory.snapshots[static_cast<std::size_t>(pulse_idx)] = std::move(snapshot);
    }

    return true;
}

bool TargetKinematics::generate_trajectories(const TargetList& targets,
                                             const BeamView& beam,
                                             const RadarSystemParams& system,
                                             const target::TargetConfig& config,
                                             TrajectoryBatch& out_trajectories,
                                             std::string& error) {
    error.clear();
    out_trajectories.clear();
    out_trajectories.reserve(targets.size());

    for (const TargetState& target : targets) {
        if (!target.enabled) {
            continue;
        }

        TargetTrajectory trajectory;
        if (!generate_trajectory(target, beam, system, config, trajectory, error)) {
            error = "TargetKinematics::generate_trajectories failed for target_id=" +
                    std::to_string(target.id) + ": " + error;
            return false;
        }
        out_trajectories.push_back(std::move(trajectory));
    }

    return true;
}

}  // namespace radar