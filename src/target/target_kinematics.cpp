/**
 * @file target_kinematics.cpp
 * @brief 目标逐脉冲运动学与几何量计算实现
 */

#include "target/target_kinematics.h"
#include "core/math_utils.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace radar {

namespace {

/**
 * 验证目标状态参数的有效性
 * 
 * 检查运动学状态和RCS参数是否满足物理和数值约束
 * 
 * @param target  待验证的目标状态
 * @param error   错误信息（验证失败时填充）
 * @return        有效返回true，否则false
 */
bool validate_target_state(const TargetState& target, std::string& error) 
{
    // 位置验证
    if (!math::is_vec3_finite(target.position_m)) {
        error = "TargetState invalid: position_m must be finite (contains inf/nan).";
        return false;
    }

    // 速度验证
    if (!math::is_vec3_finite(target.velocity_mps)) {
        error = "TargetState invalid: velocity_mps must be finite.";
        return false;
    }

    // 加速度验证
    if (!math::is_vec3_finite(target.acceleration_mps2)) {
        error = "TargetState invalid: acceleration_mps2 must be finite.";
        return false;
    }

    // RCS验证
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


// 根据 Swerling 类型和平均 RCS 采样一个 RCS 值
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
            // 慢起伏：CPI内RCS不变
            const Scalar sigma = draw_rcs(rng, swerling, rcs_mean_m2);
            std::fill(sigma_series.begin(), sigma_series.end(), sigma);
        } else {
            // 快起伏：每脉冲独立采样
            for (std::size_t i = 0; i < sigma_series.size(); ++i) {
                sigma_series[i] = draw_rcs(rng, swerling, rcs_mean_m2);
            }
        }
    }
    return sigma_series;
}

}  



bool TargetKinematics::generate_trajectory(const TargetState& target,
                                           const BeamView& beam,
                                           const RadarParams& radar,
                                           TargetTrajectory& out_trajectory,
                                           std::string& error) 
{
    error.clear();

    // ========== 输入验证 ==========
    if (!validate_target_state(target, error)) {
        return false;
    }
    if (radar.pulses_per_cpi <= 0) {
        error = "RadarParams invalid: pulses_per_cpi must be positive.";
        return false;
    }

    // ========== 初始化 ==========
    const Scalar pri_s = 1.0 / math::clamp_positive_eps(radar.prf_hz);
    const int num_pulses = radar.pulses_per_cpi;
    const TargetParams& target_params = radar.target_params;

    out_trajectory.target_id = target.id;
    out_trajectory.snapshots.assign(static_cast<std::size_t>(num_pulses), TargetSnapshot{});

    // 预生成RCS起伏序列（Swerling模型）
    const ScalarVector rcs_series = generate_rcs_series(target.swerling,
                                                        num_pulses,
                                                        target.rcs_mean_m2,
                                                        target_params.seed,
                                                        target.id,
                                                        beam.beam_index);

    // ========== 逐脉冲计算 ==========
    for (int pulse_idx = 0; pulse_idx < num_pulses; ++pulse_idx) {
        TargetSnapshot snapshot;
        const Scalar t_s = static_cast<Scalar>(pulse_idx) * pri_s;
        
        // 运动学状态（慢时间求值）
        snapshot.slow_time_s = t_s;
        snapshot.position_m = position_st(target, t_s);
        snapshot.velocity_mps = velocity_st(target, t_s);
        snapshot.acceleration_mps2 = acceleration_st(target, t_s);

        // 雷达几何（派生量）
        const Scalar range_m = snapshot.position_m.norm();
        if (!math::is_finite_positive(range_m)) {
            error = "Target position invalid: range must be positive and finite.";
            return false;
        }
        
        snapshot.range_m = range_m;
        snapshot.radial_velocity_mps = snapshot.position_m.dot(snapshot.velocity_mps) / range_m;
        snapshot.radial_acceleration_mps2 = snapshot.position_m.dot(snapshot.acceleration_mps2) / range_m;

        // 天线增益计算
        if (beam.antenna != nullptr && target_params.enable_beam_gain) {
            Scalar az_deg = 0.0;
            Scalar el_deg = 0.0;
            math::to_az_el_deg(snapshot.position_m, az_deg, el_deg);
            if (target_params.skip_out_of_beam_targets && !beam.antenna->is_target_in_beam(az_deg, el_deg,beam.pointing.azimuth,
                beam.pointing.elevation,target_params.beam_gate_threshold_db)) {
                snapshot.gain_linear = 0.0;
            } else {
                snapshot.gain_linear = beam.antenna->gain(az_deg, el_deg,beam.pointing.azimuth,beam.pointing.elevation);
            }
        }
        // RCS（预生成的起伏序列）
        snapshot.rcs_m2 = rcs_series[static_cast<std::size_t>(pulse_idx)];

        out_trajectory.snapshots[static_cast<std::size_t>(pulse_idx)] = std::move(snapshot);
    }

    return true;
}

}  // namespace radar
