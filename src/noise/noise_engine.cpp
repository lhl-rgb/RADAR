/**
 * @file noise_engine.cpp
 * @brief 雷达回波复高斯白噪声生成模块实现
 */

#include "noise/noise_engine.h"
#include "core/radar_system_params.h"
#include "core/math_utils.h"

#include <cmath>
#include <limits>

namespace radar {

NoiseEngine::NoiseEngine()
    : cfg_(),
      rng_(cfg_.seed),
      standard_normal_(0.0, 1.0) {
    RadarSystemParams sys;
    const bool ok = set_params(sys, cfg_);
    if (!ok) {
        noise_power_w_ = math::clamp_positive_eps(EPSILON);
        sigma_complex_ = std::sqrt(noise_power_w_);
        sigma_iq_ = std::sqrt(math::safe_div(noise_power_w_, 2.0));
        last_error_ = "NoiseEngine default parameters are invalid.";
    }
}

NoiseEngine::NoiseEngine(const RadarSystemParams& sys, const noise::NoiseConfig& cfg)
    : NoiseEngine() {
    (void)set_params(sys, cfg);
}

NoiseEngine::NoiseEngine(const NoiseParams& params)
    : NoiseEngine() {
    (void)set_params(params);
}

bool NoiseEngine::set_params(const RadarSystemParams& sys, const noise::NoiseConfig& cfg) {
    std::string error;
    if (!cfg.validate(error)) {
        last_error_ = error;
        return false;
    }

    Scalar noise_power_w = 0.0;
    Scalar sigma_complex = 0.0;
    Scalar sigma_iq = 0.0;

    if (!compute_noise_power(sys, cfg, noise_power_w, error)) {
        last_error_ = error;
        return false;
    }

    if (!build_cache(noise_power_w, sigma_complex, sigma_iq, error)) {
        last_error_ = error;
        return false;
    }

    cfg_ = cfg;
    noise_power_w_ = noise_power_w;
    sigma_complex_ = sigma_complex;
    sigma_iq_ = sigma_iq;
    rng_.seed(cfg_.seed);
    last_error_.clear();

    return true;
}

bool NoiseEngine::set_params(const NoiseParams& params) {
    // 转换为新参数结构
    RadarSystemParams sys;
    sys.noise_figure_db = params.noise_figure_db;
    sys.noise_figure_linear = math::db_to_linear(params.noise_figure_db);

    noise::NoiseConfig cfg;
    cfg.mode = params.mode;
    cfg.sigma_complex = params.sigma_complex;
    cfg.noise_power_w = params.noise_power_w;
    cfg.system_temperature_k = params.system_temperature_k;
    cfg.noise_bandwidth_hz = params.noise_bandwidth_hz;
    cfg.seed = params.seed;

    // 保存旧参数以兼容 params() 方法
    legacy_params_ = params;

    return set_params(sys, cfg);
}

void NoiseEngine::reseed(uint64_t seed) {
    cfg_.seed = seed;
    legacy_params_.seed = seed;
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

bool NoiseEngine::compute_noise_power(const RadarSystemParams& sys,
                                      const noise::NoiseConfig& cfg,
                                      Scalar& out_noise_power_w,
                                      std::string& error) {
    switch (cfg.mode) {
    case NoiseLevelMode::ComplexSigma:
        if (!math::is_finite_positive(cfg.sigma_complex)) {
            error = "NoiseConfig invalid: sigma_complex must be positive and finite.";
            return false;
        }
        out_noise_power_w = cfg.sigma_complex * cfg.sigma_complex;
        return true;

    case NoiseLevelMode::NoisePower:
        if (!math::is_finite_positive(cfg.noise_power_w)) {
            error = "NoiseConfig invalid: noise_power_w must be positive and finite.";
            return false;
        }
        out_noise_power_w = cfg.noise_power_w;
        return true;

    case NoiseLevelMode::ThermalKTB:
        if (!math::is_finite_positive(cfg.system_temperature_k)) {
            error = "NoiseConfig invalid: system_temperature_k must be positive and finite.";
            return false;
        }
        if (!math::is_finite_positive(cfg.noise_bandwidth_hz)) {
            error = "NoiseConfig invalid: noise_bandwidth_hz must be positive and finite.";
            return false;
        }
        // 使用 RadarSystemParams 中的 noise_figure_db
        if (!std::isfinite(sys.noise_figure_db) || sys.noise_figure_db < 0.0) {
            error = "RadarSystemParams invalid: noise_figure_db must be finite and >= 0.";
            return false;
        }

        out_noise_power_w =
            kBoltzmann *
            cfg.system_temperature_k *
            cfg.noise_bandwidth_hz *
            math::db_to_linear(sys.noise_figure_db);
        return true;
    }

    error = "NoiseConfig invalid: unsupported mode.";
    return false;
}

bool NoiseEngine::compute_noise_power_legacy(const NoiseParams& params,
                                             Scalar& out_noise_power_w,
                                             std::string& error) {
    switch (params.mode) {
    case NoiseLevelMode::ComplexSigma:
        if (!math::is_finite_positive(params.sigma_complex)) {
            error = "NoiseParams invalid: sigma_complex must be positive and finite.";
            return false;
        }
        out_noise_power_w = params.sigma_complex * params.sigma_complex;
        return true;

    case NoiseLevelMode::NoisePower:
        if (!math::is_finite_positive(params.noise_power_w)) {
            error = "NoiseParams invalid: noise_power_w must be positive and finite.";
            return false;
        }
        out_noise_power_w = params.noise_power_w;
        return true;

    case NoiseLevelMode::ThermalKTB:
        if (!math::is_finite_positive(params.system_temperature_k)) {
            error = "NoiseParams invalid: system_temperature_k must be positive and finite.";
            return false;
        }
        if (!math::is_finite_positive(params.noise_bandwidth_hz)) {
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
            math::db_to_linear(params.noise_figure_db);
        return true;
    }

    error = "NoiseParams invalid: unsupported mode.";
    return false;
}

bool NoiseEngine::build_cache(Scalar noise_power_w,
                              Scalar& out_sigma_complex,
                              Scalar& out_sigma_iq,
                              std::string& error) {
    if (!math::is_finite_positive(noise_power_w) ||
        noise_power_w <= std::numeric_limits<Scalar>::min()) {
        error = "NoiseParams invalid: resolved noise power must be positive and finite.";
        return false;
    }

    out_sigma_complex = std::sqrt(noise_power_w);
    out_sigma_iq = std::sqrt(math::safe_div(noise_power_w, 2.0));

    if (!math::is_finite_positive(out_sigma_complex) || !math::is_finite_positive(out_sigma_iq)) {
        error = "NoiseParams invalid: resolved sigma is not finite.";
        return false;
    }

    return true;
}

}  // namespace radar