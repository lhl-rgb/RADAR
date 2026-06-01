/**
 * @file antenna_model.cpp
 * @brief 天线方向图模型实现
 */

#include "antenna/antenna_model.h"
#include "core/tools/math_utils.h"

#include <cmath>

namespace radar::antenna
{

    void AntennaModel::set_config(const AntennaConfig &config)
    {
        config_ = config;
        config_.az_beamwidth_deg = std::max(config_.az_beamwidth_deg, 0.1f);
        config_.el_beamwidth_deg = std::max(config_.el_beamwidth_deg, 0.1f);
        peak_gain_linear_ = math::clamp_positive_eps(math::db_to_linear(config_.peak_gain_db));
    }

    Scalar AntennaModel::gain(Scalar target_az_deg, Scalar target_el_deg,
                              Scalar beam_az_deg, Scalar beam_el_deg) const
    {
        const Scalar p_norm = normalized_power(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg);
        return peak_gain_linear_ * p_norm;
    }

    Scalar AntennaModel::gain_db(Scalar target_az_deg, Scalar target_el_deg,
                                 Scalar beam_az_deg, Scalar beam_el_deg) const
    {
        return math::linear_to_db(gain(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg));
    }

    Scalar AntennaModel::normalized_power(Scalar target_az_deg, Scalar target_el_deg,
                                          Scalar beam_az_deg, Scalar beam_el_deg) const
    {
        return normalized_power_gaussian(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg);
    }

    Scalar AntennaModel::normalized_az_power(Scalar target_az_deg, Scalar beam_az_deg) const
    {
        const Scalar d_az_deg = math::wrap_azimuth_deg(target_az_deg - beam_az_deg);
        const Scalar x_az = d_az_deg / beamwidth_3db_az_deg();
        constexpr Scalar k = 4.0f * 0.6931471805599453f;
        return std::clamp(std::exp(-k * x_az * x_az), 0.0f, 1.0f);
    }

    Scalar AntennaModel::normalized_el_power(Scalar target_el_deg, Scalar beam_el_deg) const
    {
        const Scalar d_el_deg = target_el_deg - beam_el_deg;
        const Scalar x_el = d_el_deg / beamwidth_3db_el_deg();
        constexpr Scalar k = 4.0f * 0.6931471805599453f;
        return std::clamp(std::exp(-k * x_el * x_el), 0.0f, 1.0f);
    }

    Scalar AntennaModel::normalized_az_amplitude(Scalar target_az_deg, Scalar beam_az_deg) const
    {
        return std::sqrt(normalized_az_power(target_az_deg, beam_az_deg));
    }

    Scalar AntennaModel::normalized_el_amplitude(Scalar target_el_deg, Scalar beam_el_deg) const
    {
        return std::sqrt(normalized_el_power(target_el_deg, beam_el_deg));
    }

    bool AntennaModel::is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                                         Scalar beam_az_deg, Scalar beam_el_deg,
                                         Scalar threshold_db) const
    {
        const Scalar p_norm = normalized_power(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg);
        const Scalar threshold_linear = math::db_to_linear(threshold_db);
        return p_norm >= threshold_linear;
    }

    Scalar AntennaModel::normalized_power_gaussian(Scalar target_az_deg, Scalar target_el_deg,
                                                   Scalar beam_az_deg, Scalar beam_el_deg) const
    {
        const Scalar p_norm =
            normalized_az_power(target_az_deg, beam_az_deg) *
            normalized_el_power(target_el_deg, beam_el_deg);
        return std::clamp(p_norm, 0.0f, 1.0f);
    }

    Scalar AntennaModel::beamwidth_3db_az_deg() const
    {
        return config_.az_beamwidth_deg;
    }

    Scalar AntennaModel::beamwidth_3db_el_deg() const
    {
        return config_.el_beamwidth_deg;
    }

} // namespace radar::antenna
