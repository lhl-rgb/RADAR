/**
 * @file noise_engine.cpp
 * @brief 雷达回波复高斯白噪声生成模块实现
 */

#include "noise/noise_engine.h"
#include "core/tools/math_utils.h"
#include "cuda/gaussian.cuh"

#include <cmath>
#include <limits>

namespace radar::noise
{

    void NoiseEngine::reset_sequence_state(uint64_t seed)
    {
        cfg_.seed = seed;
        rng_.seed(seed);
        sample_offset_ = 0;
    }
    // 根据配置和系统参数，确定使用CPU还是GPU进行噪声生成
    void NoiseEngine::resolve_backend()
    {
        backend_ = ResolvedBackend::CPU;

        if (cfg_.backend == NoiseExecutionBackend::CPU)
        {
            return;
        }

#if RADAR_HAS_CUDA
        std::string error;
        if (cuda::is_gaussian_generator_available(&error))
        {
            backend_ = ResolvedBackend::GPU;
            return;
        }

        if (cfg_.backend == NoiseExecutionBackend::GPU)
        {
            last_error_ = error;
        }
#else
        if (cfg_.backend == NoiseExecutionBackend::GPU)
        {
            last_error_ = "NoiseConfig requested GPU backend, but CUDA is not enabled in this build.";
        }
#endif
    }

    bool NoiseEngine::initialize()
    {
        if (initialized_)
        {
            return true;
        }

        std::string error;
        if (!cfg_.validate(error))
        {
            last_error_ = error;
            return false;
        }

        Scalar noise_power_w = 0.0;
        Scalar sigma_complex = 0.0;
        Scalar sigma_iq = 0.0;

        if (!compute_noise_power(sys_, cfg_, noise_power_w, error))
        {
            last_error_ = error;
            return false;
        }

        if (!build_cache(noise_power_w, sigma_complex, sigma_iq, error))
        {
            last_error_ = error;
            return false;
        }

        noise_power_w_ = noise_power_w;
        sigma_complex_ = sigma_complex;
        sigma_iq_ = sigma_iq;
        reset_sequence_state(cfg_.seed);
        last_error_.clear();
        resolve_backend();
        initialized_ = true;

        return true;
    }

    void NoiseEngine::reseed(uint64_t seed)
    {
        reset_sequence_state(seed);
    }

    Complex NoiseEngine::sample()
    {
        const Scalar i = sigma_iq_ * standard_normal_(rng_);
        const Scalar q = sigma_iq_ * standard_normal_(rng_);
        ++sample_offset_;
        return Complex(i, q);
    }

    ComplexVec NoiseEngine::generate_cpu(std::size_t n)
    {
        ComplexVec noise(n, Complex(0.0, 0.0));
        for (std::size_t i = 0; i < n; ++i)
        {
            noise[i] = sample();
        }
        return noise;
    }

    bool NoiseEngine::should_use_gpu(std::size_t n) const
    {
        return backend_ == ResolvedBackend::GPU && n > 0;
    }

    ComplexVec NoiseEngine::generate(std::size_t n)
    {
        if (!should_use_gpu(n))
        {
            return generate_cpu(n);
        }

        ComplexVec noise(n, Complex(0.0, 0.0));
        std::string error;
        if (!cuda::generate_complex_gaussian_host(noise.data(),
                                                  n,
                                                  sigma_iq_,
                                                  cfg_.seed,
                                                  sample_offset_,
                                                  &error))
        {
            last_error_ = error;
            return generate_cpu(n);
        }

        sample_offset_ += static_cast<std::uint64_t>(n);
        return noise;
    }

    void NoiseEngine::add_noise(ComplexVec &signal)
    {
        const ComplexVec noise = generate(signal.size());
        for (std::size_t i = 0; i < signal.size(); ++i)
        {
            signal[i] += noise[i];
        }
    }

    bool NoiseEngine::compute_noise_power(const RadarSystemParams &sys,
                                          const NoiseConfig &cfg,
                                          Scalar &out_noise_power_w,
                                          std::string &error)
    {
        switch (cfg.mode)
        {
        case NoiseLevelMode::ComplexSigma:
            if (!math::is_finite_positive(cfg.sigma_complex))
            {
                error = "NoiseConfig invalid: sigma_complex must be positive and finite.";
                return false;
            }
            out_noise_power_w = cfg.sigma_complex * cfg.sigma_complex;
            return true;

        case NoiseLevelMode::NoisePower:
            if (!math::is_finite_positive(cfg.noise_power_w))
            {
                error = "NoiseConfig invalid: noise_power_w must be positive and finite.";
                return false;
            }
            out_noise_power_w = cfg.noise_power_w;
            return true;

        case NoiseLevelMode::ThermalKTB:
            if (!math::is_finite_positive(cfg.system_temperature_k))
            {
                error = "NoiseConfig invalid: system_temperature_k must be positive and finite.";
                return false;
            }
            if (!math::is_finite_positive(cfg.noise_bandwidth_hz))
            {
                error = "NoiseConfig invalid: noise_bandwidth_hz must be positive and finite.";
                return false;
            }
            if (!std::isfinite(sys.noise_figure_db) || sys.noise_figure_db < 0.0)
            {
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

    bool NoiseEngine::build_cache(Scalar noise_power_w,
                                  Scalar &out_sigma_complex,
                                  Scalar &out_sigma_iq,
                                  std::string &error)
    {
        if (!math::is_finite_positive(noise_power_w) ||
            noise_power_w <= std::numeric_limits<Scalar>::min())
        {
            error = "NoiseParams invalid: resolved noise power must be positive and finite.";
            return false;
        }

        out_sigma_complex = std::sqrt(noise_power_w);
        out_sigma_iq = std::sqrt(math::safe_div(noise_power_w, 2.0));

        if (!math::is_finite_positive(out_sigma_complex) || !math::is_finite_positive(out_sigma_iq))
        {
            error = "NoiseParams invalid: resolved sigma is not finite.";
            return false;
        }

        return true;
    }

} // namespace radar
