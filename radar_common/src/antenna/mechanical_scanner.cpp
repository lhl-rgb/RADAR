/**
 * @file mechanical_scanner.cpp
 * @brief 机械扫描控制器实现
 */

#include "antenna/mechanical_scanner.h"

#include <cmath>
#include <stdexcept>

namespace radar::antenna
{

    namespace
    {

        constexpr const char *kNotInitializedError =
            "MechanicalScanner: not initialized. Call initialize() before use.";

    } // namespace

    void MechanicalScanner::set_config(const AntennaConfig &ant_config, const MechanicalScanConfig &scan_config)
    {
        set_antenna_config(ant_config);
        set_scan_config(scan_config);
    }

    bool MechanicalScanner::initialize()
    {
        if (scan_config_.azimuth_step_per_prt_deg <= 0.0 || scan_config_.pulses_per_rotation <= 0)
        {
            last_error_ = "MechanicalScanConfig derived parameters are invalid";
            return false;
        }

        antenna_model_.set_config(antenna_config_);
        if (!antenna_model_.initialize())
        {
            last_error_ = "AntennaModel initialization failed";
            return false;
        }

        current_azimuth_deg_ = scan_config_.az_start_deg;

        initialized_ = true;
        return true;
    }

    bool MechanicalScanner::advance_one_prt()
    {
        if (!initialized_)
            return false;

        current_azimuth_deg_ += scan_config_.azimuth_step_per_prt_deg;

        current_azimuth_deg_ = std::fmod(current_azimuth_deg_, 360.0);
        if (current_azimuth_deg_ < 0.0)
        {
            current_azimuth_deg_ += 360.0;
        }

        return true;
    }

    void MechanicalScanner::reset()
    {
        current_azimuth_deg_ = scan_config_.az_start_deg;
    }

    BeamPoint MechanicalScanner::get_beam_pointing() const
    {
        return {current_azimuth_deg_, scan_config_.elevation_deg};
    }

    BeamPoint MechanicalScanner::get_beam_at_time(Scalar time_s) const
    {
        Scalar az = scan_config_.az_start_deg + scan_config_.rotation_rate_dps * time_s;
        az = std::fmod(az, 360.0);
        if (az < 0.0)
            az += 360.0;
        return {az, scan_config_.elevation_deg};
    }

    bool MechanicalScanner::is_echo_active() const
    {
        const Scalar az = current_azimuth_deg_;
        const Scalar start = scan_config_.az_start_deg;
        const Scalar end = scan_config_.az_end_deg;

        if (start < end)
        {
            return (az >= start) && (az <= end);
        }

        // wrap-around sector: [start, 360) U [0, end]
        return (az >= start) || (az <= end);
    }

    Scalar MechanicalScanner::get_gain_to_target(Scalar target_az_deg, Scalar target_el_deg) const
    {
        if (!initialized_)
            throw std::runtime_error(kNotInitializedError);
        const BeamPoint beam = get_beam_pointing();
        return antenna_model_.gain(target_az_deg, target_el_deg, beam.azimuth_deg, beam.elevation_deg);
    }

    Scalar MechanicalScanner::get_gain_db_to_target(Scalar target_az_deg, Scalar target_el_deg) const
    {
        if (!initialized_)
            throw std::runtime_error(kNotInitializedError);
        const BeamPoint beam = get_beam_pointing();
        return antenna_model_.gain_db(target_az_deg, target_el_deg, beam.azimuth_deg, beam.elevation_deg);
    }

    Scalar MechanicalScanner::get_normalized_power_to_target(Scalar target_az_deg, Scalar target_el_deg) const
    {
        if (!initialized_)
            throw std::runtime_error(kNotInitializedError);
        const BeamPoint beam = get_beam_pointing();
        return antenna_model_.normalized_power(target_az_deg, target_el_deg,
                                               beam.azimuth_deg, beam.elevation_deg);
    }

    bool MechanicalScanner::is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                                              Scalar threshold_db) const
    {
        if (!initialized_)
            throw std::runtime_error(kNotInitializedError);
        const BeamPoint beam = get_beam_pointing();
        return antenna_model_.is_target_in_beam(target_az_deg, target_el_deg,
                                                beam.azimuth_deg, beam.elevation_deg, threshold_db);
    }

} // namespace radar::antenna
