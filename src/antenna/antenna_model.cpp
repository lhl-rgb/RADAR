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

    // ULA 线阵的有效扫描范围约为 ±60°（从阵列法线起算）
    // 当 |θ - θ₀| > 90° 时，目标在阵列后半空间，应被抑制
    // 使用相对角度判断：cos(θ - θ₀) > 0 表示目标在波束朝向前半空间
    const Scalar rel_angle = theta - theta0;
    const Scalar front_factor = std::cos(rel_angle);
    if (front_factor < EPSILON) {
        return 0.0;
    }

    // 标准 ULA 阵列因子相位差公式：ψ = kd * (sin(θ) - sin(θ₀))
    const Scalar psi = 2.0 * PI * config_.spacing_az_lambda * (std::sin(theta) - std::sin(theta0));

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

    // 旋转阵面模型：阵面法线随波束指向旋转
    // 使用相对角度计算阵列因子（目标角度相对于阵面法线）
    const Scalar rel_az = az - az0;
    const Scalar rel_el = el - el0;

    // 前半空间约束：目标必须在阵面朝向的前半空间
    const Scalar front_factor = std::cos(rel_el) * std::cos(rel_az);
    if (front_factor <= 0.0) {
        return 0.0;
    }

    // 工程扫描限制：方位±60°，俯仰±30°（超出后增益损失和栅瓣风险过大）
    if (std::abs(rel_az) > 60.0 * PI / 180.0) {
        return 0.0;
    }
    if (std::abs(rel_el) > 30.0 * PI / 180.0) {
        return 0.0;
    }

    // 旋转阵面模型下的方向余弦（在阵列坐标系下）
    const Scalar u = std::cos(rel_el) * std::sin(rel_az);
    const Scalar v = std::sin(rel_el);

    // 相位差：阵面法线对准波束，所以参考相位为 0
    const Scalar psix = 2.0 * PI * config_.spacing_az_lambda * u;
    const Scalar psiy = 2.0 * PI * config_.spacing_el_lambda * v;

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

Scalar AntennaModel::beamwidth_3db_az_deg() const {
    // 3dB 波束宽度近似公式：θ_3dB ≈ 0.886 / (N * d_lambda) 弧度
    // 对于 Hamming 加权，波束宽度约为均匀加权的 1.3 倍
    const Scalar k_factor = (config_.weight_type_az == AntennaWeightType::Hamming) ? 1.3 : 1.0;
    const Scalar beamwidth_rad = k_factor * 0.886 /
        (static_cast<Scalar>(config_.num_elements_az) * config_.spacing_az_lambda);
    return math::rad_to_deg(beamwidth_rad);
}

Scalar AntennaModel::beamwidth_3db_el_deg() const {
    const Scalar k_factor = (config_.weight_type_el == AntennaWeightType::Hamming) ? 1.3 : 1.0;
    const Scalar beamwidth_rad = k_factor * 0.886 /
        (static_cast<Scalar>(config_.num_elements_el) * config_.spacing_el_lambda);
    return math::rad_to_deg(beamwidth_rad);
}

}  // namespace radar::antenna
