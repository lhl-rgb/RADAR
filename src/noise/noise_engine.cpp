/**
 * @file noise_engine.cpp
 * @brief 雷达回波复高斯白噪声生成模块实现
 */

#include "noise/noise_engine.h"

#include <cmath>
#include <limits>

namespace radar {

NoiseEngine::NoiseEngine()
    : params_(),
      rng_(params_.seed),
      standard_normal_(0.0, 1.0) {
    const bool ok = set_params(params_);
    if (!ok) {
        noise_power_w_ = EPSILON;
        sigma_complex_ = std::sqrt(EPSILON);
        sigma_iq_ = std::sqrt(EPSILON / 2.0);
        last_error_ = "NoiseEngine default parameters are invalid.";
    }
}

NoiseEngine::NoiseEngine(const NoiseParams& params)
    : NoiseEngine() {
    (void)set_params(params);
}

bool NoiseEngine::set_params(const NoiseParams& params) {
    Scalar noise_power_w = 0.0;
    Scalar sigma_complex = 0.0;
    Scalar sigma_iq = 0.0;
    std::string error;

    if (!compute_noise_power(params, noise_power_w, error)) {
        last_error_ = error;
        return false;
    }

    if (!build_cache(noise_power_w, sigma_complex, sigma_iq, error)) {
        last_error_ = error;
        return false;
    }

    params_ = params;
    noise_power_w_ = noise_power_w;
    sigma_complex_ = sigma_complex;
    sigma_iq_ = sigma_iq;
    rng_.seed(params_.seed);
    last_error_.clear();

    return true;
}

void NoiseEngine::reseed(uint64_t seed) {
    params_.seed = seed;
    rng_.seed(seed);
}

Complex NoiseEngine::sample() {
    const Scalar i = sigma_iq_ * standard_normal_(rng_);
    const Scalar q = sigma_iq_ * standard_normal_(rng_);
    return Complex(i, q);
}

ComplexVec NoiseEngine::generate(std::size_t n) {
    ComplexVec noise(n, Complex(0.0, 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        noise[i] = sample();
    }
    return noise;
}

void NoiseEngine::add_noise(ComplexVec& signal) {
    for (auto& x : signal) {
        x += sample();
    }
}

void NoiseEngine::add_noise(CpiEcho& cpi_echo) {
    for (auto& pulse : cpi_echo.pulses) {
        add_noise(pulse);
    }
}

bool NoiseEngine::compute_noise_power(const NoiseParams& params,
                                      Scalar& out_noise_power_w,
                                      std::string& error) {
    switch (params.mode) {
    case NoiseLevelMode::ComplexSigma:
        if (!is_positive_finite(params.sigma_complex)) {
            error = "NoiseParams invalid: sigma_complex must be positive and finite.";
            return false;
        }
        out_noise_power_w = params.sigma_complex * params.sigma_complex;
        return true;

    case NoiseLevelMode::NoisePower:
        if (!is_positive_finite(params.noise_power_w)) {
            error = "NoiseParams invalid: noise_power_w must be positive and finite.";
            return false;
        }
        out_noise_power_w = params.noise_power_w;
        return true;

    case NoiseLevelMode::ThermalKTB:
        if (!is_positive_finite(params.system_temperature_k)) {
            error = "NoiseParams invalid: system_temperature_k must be positive and finite.";
            return false;
        }
        if (!is_positive_finite(params.noise_bandwidth_hz)) {
            error = "NoiseParams invalid: noise_bandwidth_hz must be positive and finite.";
            return false;
        }
        if (!std::isfinite(params.noise_figure_db) || params.noise_figure_db < 0.0) {
            error = "NoiseParams invalid: noise_figure_db must be finite and >= 0.";
            return false;
        }

        out_noise_power_w =
            kBoltzmann *
            params.system_temperature_k *
            params.noise_bandwidth_hz *
            db_to_linear(params.noise_figure_db);
        return true;
    }

    error = "NoiseParams invalid: unsupported mode.";
    return false;
}

bool NoiseEngine::build_cache(Scalar noise_power_w,
                              Scalar& out_sigma_complex,
                              Scalar& out_sigma_iq,
                              std::string& error) {
    if (!is_positive_finite(noise_power_w) ||
        noise_power_w <= std::numeric_limits<Scalar>::min()) {
        error = "NoiseParams invalid: resolved noise power must be positive and finite.";
        return false;
    }

    out_sigma_complex = std::sqrt(noise_power_w);
    out_sigma_iq = std::sqrt(noise_power_w / 2.0);

    if (!is_positive_finite(out_sigma_complex) || !is_positive_finite(out_sigma_iq)) {
        error = "NoiseParams invalid: resolved sigma is not finite.";
        return false;
    }

    return true;
}

bool NoiseEngine::is_positive_finite(Scalar value) {
    return std::isfinite(value) && value > 0.0;
}

Scalar NoiseEngine::db_to_linear(Scalar db_value) {
    return std::pow(10.0, db_value / 10.0);
}

}  // namespace radar