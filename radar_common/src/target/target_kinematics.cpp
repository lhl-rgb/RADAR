/**
 * @file target_kinematics.cpp
 * @brief 目标逐脉冲运动学与几何量计算实现
 */

#include "target/target_kinematics.h"
#include "core/tools/math_utils.h"

namespace radar::target
{

    namespace
    {

        bool validate_target_state(const TargetState &target, std::string &error)
        {
            if (!math::is_vec3_finite(target.position_m))
            {
                error = "TargetState invalid: position_m must be finite (contains inf/nan).";
                return false;
            }
            if (!math::is_vec3_finite(target.velocity_mps))
            {
                error = "TargetState invalid: velocity_mps must be finite.";
                return false;
            }
            if (!math::is_vec3_finite(target.acceleration_mps2))
            {
                error = "TargetState invalid: acceleration_mps2 must be finite.";
                return false;
            }
            if (!math::is_finite_positive(target.rcs_mean_m2))
            {
                error = "TargetState invalid: rcs_mean_m2 must be positive and finite.";
                return false;
            }
            return true;
        }

    } // namespace

    bool TargetKinematics::generate_snapshot(const TargetState &target,
                                             const BeamView &beam,
                                             const PulseContext &pulse,
                                             const target::TargetConfig &config,
                                             TargetSnapshot &out_snapshot,
                                             std::string &error)
    {
        error.clear();

        if (!validate_target_state(target, error))
        {
            return false;
        }

        out_snapshot = TargetSnapshot{};
        out_snapshot.time_s = pulse.current_time_s;
        out_snapshot.position_m = target.position_m;
        out_snapshot.velocity_mps = target.velocity_mps;
        out_snapshot.acceleration_mps2 = target.acceleration_mps2;

        const Scalar range_m = out_snapshot.position_m.norm();
        if (!math::is_finite_positive(range_m))
        {
            error = "Target position invalid: range must be positive and finite.";
            return false;
        }

        out_snapshot.range_m = range_m;
        out_snapshot.radial_velocity_mps = out_snapshot.position_m.dot(out_snapshot.velocity_mps) / range_m;
        out_snapshot.radial_acceleration_mps2 = out_snapshot.position_m.dot(out_snapshot.acceleration_mps2) / range_m;

        if (beam.antenna != nullptr && config.enable_beam_gain)
        {
            Scalar az_deg = 0.0;
            Scalar el_deg = 0.0;
            math::to_az_el_deg(out_snapshot.position_m, az_deg, el_deg);

            if (config.skip_out_of_beam_targets &&
                !beam.antenna->is_target_in_beam(az_deg, el_deg,
                                                 beam.pointing.azimuth_deg,
                                                 beam.pointing.elevation_deg,
                                                 config.beam_gate_threshold_db))
            {
                out_snapshot.gain_linear = 0.0;
            }
            else
            {
                out_snapshot.gain_linear = beam.antenna->gain(az_deg, el_deg,
                                                              beam.pointing.azimuth_deg,
                                                              beam.pointing.elevation_deg);
            }
        }

        return true;
    }

} // namespace radar::target
