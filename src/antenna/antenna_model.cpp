/**
 * @file antenna_model.cpp
 * @brief 相控阵天线物理模型实现
 */

#include "antenna/antenna_model.h"
#include "core/tools/math_utils.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace radar::antenna {

void AntennaModel::set_config(const AntennaConfig& config) {
    config_ = config;
    // 参数保护，确保阵元数和间距合理
    config_.num_elements_az = std::max(config_.num_elements_az, 1);
    config_.num_elements_el = std::max(config_.num_elements_el, 1);
    config_.spacing_az_lambda = math::clamp_positive_eps(config_.spacing_az_lambda);
    config_.spacing_el_lambda = math::clamp_positive_eps(config_.spacing_el_lambda);

    refresh_internal_cache();// 刷新内部缓存（权重向量、峰值增益等）
}

void AntennaModel::refresh_internal_cache() {
    peak_gain_linear_ = math::clamp_positive_eps(math::db_to_linear(config_.peak_gain_db));
    weights_az_ = make_weights(config_.num_elements_az, config_.weight_type_az);
    weights_el_ = make_weights(config_.num_elements_el, config_.weight_type_el);
}

Scalar AntennaModel::gain(Scalar target_az_deg, Scalar target_el_deg,
                          Scalar beam_az_deg, Scalar beam_el_deg) const {
    const Scalar p_norm =
        normalized_power(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg);
    return peak_gain_linear_ * p_norm;
}

Scalar AntennaModel::gain_db(Scalar target_az_deg, Scalar target_el_deg,
                             Scalar beam_az_deg, Scalar beam_el_deg) const {
    return math::linear_to_db(gain(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg));
}

Scalar AntennaModel::normalized_power(Scalar target_az_deg, Scalar target_el_deg,
                                      Scalar beam_az_deg, Scalar beam_el_deg) const {
    if (config_.model_type == PhasedArrayModelType::ULA_1D) {
        return normalized_power_ula(target_az_deg, beam_az_deg);
    }

    return normalized_power_upa(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg);
}

bool AntennaModel::is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                                     Scalar beam_az_deg, Scalar beam_el_deg,
                                     Scalar threshold_db) const {
    const Scalar p_norm =
        normalized_power(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg);
    const Scalar threshold_linear = math::db_to_linear(threshold_db);
    return p_norm >= threshold_linear;
}

Scalar AntennaModel::normalized_power_ula(Scalar target_az_deg,
                                          Scalar beam_az_deg) const {
    const Scalar theta = target_az_deg * PI / 180.0;
    const Scalar theta0 = beam_az_deg * PI / 180.0;

    const Scalar psi =
        2.0 * PI * config_.spacing_az_lambda * (std::sin(theta) - std::sin(theta0));

    std::complex<Scalar> af(0.0, 0.0);
    for (int n = 0; n < config_.num_elements_az; ++n) {
        const Scalar phase = static_cast<Scalar>(n) * psi;
        af += weights_az_[static_cast<std::size_t>(n)] *
              std::complex<Scalar>(std::cos(phase), std::sin(phase));
    }

    return std::clamp(std::norm(af), 0.0, 1.0);
}

Scalar AntennaModel::normalized_power_upa(Scalar target_az_deg, Scalar target_el_deg,
                                          Scalar beam_az_deg, Scalar beam_el_deg) const {
    const Scalar az = target_az_deg * PI / 180.0;
    const Scalar el = target_el_deg * PI / 180.0;
    const Scalar az0 = beam_az_deg * PI / 180.0;
    const Scalar el0 = beam_el_deg * PI / 180.0;

    const Scalar u = std::cos(el) * std::sin(az);
    const Scalar v = std::sin(el);
    const Scalar u0 = std::cos(el0) * std::sin(az0);
    const Scalar v0 = std::sin(el0);

    const Scalar psix = 2.0 * PI * config_.spacing_az_lambda * (u - u0);
    const Scalar psiy = 2.0 * PI * config_.spacing_el_lambda * (v - v0);

    std::complex<Scalar> af(0.0, 0.0);
    for (int m = 0; m < config_.num_elements_az; ++m) {
        for (int n = 0; n < config_.num_elements_el; ++n) {
            const Scalar phase =
                static_cast<Scalar>(m) * psix + static_cast<Scalar>(n) * psiy;

            af += weights_az_[static_cast<std::size_t>(m)] *
                  weights_el_[static_cast<std::size_t>(n)] *
                  std::complex<Scalar>(std::cos(phase), std::sin(phase));
        }
    }

    return std::clamp(std::norm(af), 0.0, 1.0);
}

std::vector<Scalar> AntennaModel::make_weights(int length,
                                               AntennaWeightType weight_type) {
    length = std::max(length, 1);

    std::vector<Scalar> weights(static_cast<std::size_t>(length), 1.0);

    if (weight_type == AntennaWeightType::Hamming && length > 1) {
        for (int i = 0; i < length; ++i) {
            weights[static_cast<std::size_t>(i)] =
                0.54 - 0.46 * std::cos(2.0 * PI * static_cast<Scalar>(i) /
                                       static_cast<Scalar>(length - 1));
        }
    }

    Scalar sum_weights = 0.0;
    for (const Scalar w : weights) {
        sum_weights += w;
    }
    sum_weights = math::clamp_positive_eps(sum_weights);

    for (Scalar& w : weights) {
        w /= sum_weights;
    }

    return weights;
}

}  // namespace radar::antenna
