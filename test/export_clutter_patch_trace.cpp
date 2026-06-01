#include "clutter/sea_clutter_config.h"
#include "clutter/sea_clutter_model.h"
#include "core/radar_system_params.h"
#include "cuda/clutter_rng.cuh"

#include <boost/math/distributions/gamma.hpp>
#include <boost/math/distributions/normal.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace
{
    struct Options
    {
        int sample_count = 20000;
        int range_idx = 1000;
        int az_idx = 0;
        int fir_length = 13;
        float sigma_f_hz = 40.0f;
        float doppler_center_hz = 0.0f;
        float prf_hz = 1600.0f;
        unsigned long long seed = 2026ULL;
        std::string output_prefix = "out/clutter_patch_trace";
        int distribution = static_cast<int>(radar::ClutterDistribution::Rayleigh);
        float weibull_shape = 2.0f;
        float weibull_scale = 1.41421356f;
        float lognormal_mu = -1.0f;
        float lognormal_sigma = 1.0f;
        float k_shape_nu = 1.0f;
        float k_texture_bandwidth_hz = 2.0f;
        int k_lut_size = 4096;
    };

    int parse_int_arg(char **argv, int argc, int index, int fallback)
    {
        if (index >= argc)
        {
            return fallback;
        }
        return std::max(1, std::atoi(argv[index]));
    }

    int parse_nonnegative_int_arg(char **argv, int argc, int index, int fallback)
    {
        if (index >= argc)
        {
            return fallback;
        }
        return std::max(0, std::atoi(argv[index]));
    }

    float parse_float_arg(char **argv, int argc, int index, float fallback)
    {
        if (index >= argc)
        {
            return fallback;
        }
        return static_cast<float>(std::atof(argv[index]));
    }

    unsigned long long parse_ull_arg(char **argv, int argc, int index, unsigned long long fallback)
    {
        if (index >= argc)
        {
            return fallback;
        }
        return std::strtoull(argv[index], nullptr, 0);
    }

    std::string parse_string_arg(char **argv, int argc, int index, const std::string &fallback)
    {
        if (index >= argc)
        {
            return fallback;
        }
        return argv[index];
    }

    Options parse_options(int argc, char **argv)
    {
        Options options;
        options.sample_count = parse_int_arg(argv, argc, 1, options.sample_count);
        options.range_idx = parse_int_arg(argv, argc, 2, options.range_idx);
        options.az_idx = parse_nonnegative_int_arg(argv, argc, 3, options.az_idx);
        options.fir_length = parse_int_arg(argv, argc, 4, options.fir_length);
        options.sigma_f_hz = parse_float_arg(argv, argc, 5, options.sigma_f_hz);
        options.doppler_center_hz = parse_float_arg(argv, argc, 6, options.doppler_center_hz);
        options.prf_hz = parse_float_arg(argv, argc, 7, options.prf_hz);
        options.seed = parse_ull_arg(argv, argc, 8, options.seed);
        options.output_prefix = parse_string_arg(argv, argc, 9, options.output_prefix);
        options.distribution = parse_nonnegative_int_arg(argv, argc, 10, options.distribution);
        options.weibull_shape = parse_float_arg(argv, argc, 11, options.weibull_shape);
        options.weibull_scale = parse_float_arg(argv, argc, 12, options.weibull_scale);
        options.lognormal_mu = parse_float_arg(argv, argc, 13, options.lognormal_mu);
        options.lognormal_sigma = parse_float_arg(argv, argc, 14, options.lognormal_sigma);
        options.k_shape_nu = parse_float_arg(argv, argc, 15, options.k_shape_nu);
        options.k_texture_bandwidth_hz = parse_float_arg(argv, argc, 16, options.k_texture_bandwidth_hz);
        options.k_lut_size = parse_int_arg(argv, argc, 17, options.k_lut_size);
        return options;
    }

    void print_usage(const char *program)
    {
        std::cout << "Usage: " << program
                  << " [sample_count=20000] [range_idx=1000] [az_idx=0]"
                  << " [fir_length=13] [sigma_f_hz=40] [doppler_center_hz=0]"
                  << " [prf_hz=1600] [seed=2026] [output_prefix=out/clutter_patch_trace]"
                  << " [distribution=0] [weibull_shape=2] [weibull_scale=1.414]"
                  << " [lognormal_mu=-1] [lognormal_sigma=1]"
                  << " [k_shape_nu=1] [k_texture_bandwidth_hz=2] [k_lut_size=4096]\n";
    }

    std::vector<float> build_k_sqrt_tau_lut(float shape_nu, int lut_size)
    {
        lut_size = std::max(16, lut_size);
        const double nu = std::max(static_cast<double>(shape_nu), 1.0e-9);
        boost::math::normal_distribution<double> normal(0.0, 1.0);
        boost::math::gamma_distribution<double> gamma(nu, 1.0 / nu);
        std::vector<float> lut(static_cast<std::size_t>(lut_size));
        for (int i = 0; i < lut_size; ++i)
        {
            const double z = -8.0 + 16.0 * static_cast<double>(i) /
                                       static_cast<double>(lut_size - 1);
            const double u = std::clamp(boost::math::cdf(normal, z), 1.0e-15, 1.0 - 1.0e-15);
            const double tau = std::max(0.0, boost::math::quantile(gamma, u));
            lut[static_cast<std::size_t>(i)] = static_cast<float>(std::sqrt(tau));
        }
        return lut;
    }

    float lookup_k_sqrt_tau(const std::vector<float> &lut, float z)
    {
        if (lut.size() < 2)
        {
            return 1.0f;
        }
        const float z_clamped = std::clamp(z, -8.0f, 8.0f);
        const float x = (z_clamped + 8.0f) *
                        (static_cast<float>(lut.size() - 1) / 16.0f);
        int i = static_cast<int>(std::floor(x));
        i = std::max(0, std::min(i, static_cast<int>(lut.size()) - 2));
        const float frac = x - static_cast<float>(i);
        return lut[static_cast<std::size_t>(i)] * (1.0f - frac) +
               lut[static_cast<std::size_t>(i + 1)] * frac;
    }

    radar::Complex white_sample(const Options &options, unsigned long long pulse_index)
    {
        const auto pair = radar::cuda::counter_complex_gaussian_sample_pair(
            options.seed,
            static_cast<std::uint32_t>(options.range_idx),
            static_cast<std::uint32_t>(options.az_idx),
            pulse_index >> 1U,
            0U);
        const auto sample = ((pulse_index & 1ULL) == 0ULL) ? pair.even : pair.odd;
        return radar::Complex(sample.real, sample.imag);
    }

    float standard_real_gaussian(const Options &options,
                                 unsigned long long pulse_index,
                                 std::uint32_t source_id)
    {
        const auto sample = radar::cuda::counter_complex_gaussian_sample(
            options.seed,
            static_cast<std::uint32_t>(options.range_idx),
            static_cast<std::uint32_t>(options.az_idx),
            pulse_index,
            source_id);
        return 1.4142135623730951f * sample.real;
    }

    std::vector<radar::Complex> generate_patch_trace(const Options &options,
                                                     const radar::ComplexVec &fir)
    {
        std::vector<radar::Complex> trace(static_cast<std::size_t>(options.sample_count));
        const bool use_k = options.distribution == static_cast<int>(radar::ClutterDistribution::K);
        const std::vector<float> k_lut = use_k ? build_k_sqrt_tau_lut(options.k_shape_nu,
                                                                       options.k_lut_size)
                                               : std::vector<float>{};
        const float rho = use_k
                              ? std::exp(-2.0f * radar::PI *
                                         std::max(options.k_texture_bandwidth_hz, radar::EPSILON) /
                                         std::max(options.prf_hz, radar::EPSILON))
                              : 0.0f;
        float texture_state = 0.0f;
        bool texture_initialized = false;
        for (int n = 0; n < options.sample_count; ++n)
        {
            radar::Complex accum(0.0f, 0.0f);
            for (int tap = 0; tap < static_cast<int>(fir.size()); ++tap)
            {
                const unsigned long long pulse_index =
                    static_cast<unsigned long long>(n) - static_cast<unsigned long long>(tap);
                accum += fir[static_cast<std::size_t>(tap)] * white_sample(options, pulse_index);
            }
            const float power = std::norm(accum);
            if (power > 1.1754943508222875e-38f)
            {
                const float amp = std::sqrt(power);
                float target_amp = amp;
                if (options.distribution == static_cast<int>(radar::ClutterDistribution::Weibull))
                {
                    const float shape = std::max(options.weibull_shape, radar::EPSILON);
                    const float scale = std::max(options.weibull_scale, radar::EPSILON);
                    target_amp = (scale / std::pow(2.0f, 1.0f / shape)) *
                                 std::pow(power, 1.0f / shape);
                }
                else if (options.distribution == static_cast<int>(radar::ClutterDistribution::LogNormal))
                {
                    constexpr float kSqrt2 = 1.4142135623730951f;
                    target_amp = std::exp(options.lognormal_mu +
                                          options.lognormal_sigma * kSqrt2 * accum.real());
                }
                accum *= target_amp / amp;
            }
            if (use_k)
            {
                if (!texture_initialized)
                {
                    texture_state = standard_real_gaussian(options, static_cast<unsigned long long>(n), 1U);
                    texture_initialized = true;
                }
                else
                {
                    const float innovation =
                        standard_real_gaussian(options, static_cast<unsigned long long>(n), 2U);
                    texture_state =
                        rho * texture_state + std::sqrt(std::max(0.0f, 1.0f - rho * rho)) * innovation;
                }
                accum *= lookup_k_sqrt_tau(k_lut, texture_state);
            }
            trace[static_cast<std::size_t>(n)] = accum;
        }
        return trace;
    }

    void write_trace_file(const std::string &path,
                          const Options &options,
                          const std::vector<radar::Complex> &trace)
    {
        std::ofstream file(path);
        file << std::setprecision(9);
        file << "# sample_count " << options.sample_count << "\n";
        file << "# prf_hz " << options.prf_hz << "\n";
        file << "# sigma_f_hz " << options.sigma_f_hz << "\n";
        file << "# doppler_center_hz " << options.doppler_center_hz << "\n";
        file << "# range_idx " << options.range_idx << "\n";
        file << "# az_idx " << options.az_idx << "\n";
        file << "# seed " << options.seed << "\n";
        file << "# distribution " << options.distribution << "\n";
        file << "# weibull_shape " << options.weibull_shape << "\n";
        file << "# weibull_scale " << options.weibull_scale << "\n";
        file << "# lognormal_mu " << options.lognormal_mu << "\n";
        file << "# lognormal_sigma " << options.lognormal_sigma << "\n";
        file << "# k_shape_nu " << options.k_shape_nu << "\n";
        file << "# k_texture_bandwidth_hz " << options.k_texture_bandwidth_hz << "\n";
        file << "# k_lut_size " << options.k_lut_size << "\n";
        file << "# columns: n time_s real imag amplitude power\n";

        for (int n = 0; n < static_cast<int>(trace.size()); ++n)
        {
            const auto sample = trace[static_cast<std::size_t>(n)];
            const double real = sample.real();
            const double imag = sample.imag();
            const double power = real * real + imag * imag;
            file << n << ' '
                 << (static_cast<double>(n) / options.prf_hz) << ' '
                 << real << ' '
                 << imag << ' '
                 << std::sqrt(power) << ' '
                 << power << '\n';
        }
    }

    int largest_power_of_two_at_most(int value)
    {
        int out = 1;
        while (out <= value / 2)
        {
            out *= 2;
        }
        return out;
    }

    std::vector<double> compute_welch_periodogram(const std::vector<radar::Complex> &trace,
                                                  int fft_size)
    {
        const int n = static_cast<int>(trace.size());
        fft_size = std::max(16, std::min(fft_size, n));
        const int hop = std::max(1, fft_size / 2);
        std::vector<double> psd(static_cast<std::size_t>(fft_size), 0.0);
        constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
        int segment_count = 0;

        std::vector<double> window(static_cast<std::size_t>(fft_size));
        for (int i = 0; i < fft_size; ++i)
        {
            window[static_cast<std::size_t>(i)] =
                0.5 - 0.5 * std::cos(kTwoPi * static_cast<double>(i) /
                                     static_cast<double>(fft_size - 1));
        }

        for (int start = 0; start + fft_size <= n; start += hop)
        {
            ++segment_count;
            for (int k = 0; k < fft_size; ++k)
            {
                double sum_re = 0.0;
                double sum_im = 0.0;
                for (int t = 0; t < fft_size; ++t)
                {
                    const double angle = -kTwoPi * static_cast<double>(k) * static_cast<double>(t) /
                                         static_cast<double>(fft_size);
                    const double c = std::cos(angle);
                    const double s = std::sin(angle);
                    const auto x = trace[static_cast<std::size_t>(start + t)];
                    const double xr = static_cast<double>(x.real()) * window[static_cast<std::size_t>(t)];
                    const double xi = static_cast<double>(x.imag()) * window[static_cast<std::size_t>(t)];
                    sum_re += xr * c - xi * s;
                    sum_im += xr * s + xi * c;
                }
                psd[static_cast<std::size_t>(k)] += sum_re * sum_re + sum_im * sum_im;
            }
        }

        if (segment_count > 0)
        {
            for (double &value : psd)
            {
                value /= static_cast<double>(segment_count);
            }
        }

        return psd;
    }

    double gaussian_psd_shape(double freq_hz, double center_hz, double sigma_hz)
    {
        const double x = (freq_hz - center_hz) / std::max(1.0e-12, sigma_hz);
        return std::exp(-0.5 * x * x);
    }

    void write_psd_file(const std::string &path,
                        const Options &options,
                        const std::vector<double> &psd)
    {
        double max_psd = 0.0;
        for (double value : psd)
        {
            max_psd = std::max(max_psd, value);
        }
        max_psd = std::max(max_psd, 1.0e-300);

        std::ofstream file(path);
        file << std::setprecision(9);
        file << "# columns: bin freq_hz psd_linear psd_db target_shape target_shape_db\n";
        const int n = static_cast<int>(psd.size());
        for (int k_shift = 0; k_shift < n; ++k_shift)
        {
            const int k = (k_shift + n / 2) % n;
            const double freq_hz =
                (static_cast<double>(k_shift) - static_cast<double>(n / 2)) *
                options.prf_hz / static_cast<double>(n);
            const double value = psd[static_cast<std::size_t>(k)] / max_psd;
            const double target = gaussian_psd_shape(freq_hz,
                                                     options.doppler_center_hz,
                                                     options.sigma_f_hz);
            file << k_shift << ' '
                 << freq_hz << ' '
                 << value << ' '
                 << (10.0 * std::log10(std::max(value, 1.0e-300))) << ' '
                 << target << ' '
                 << (10.0 * std::log10(std::max(target, 1.0e-300))) << '\n';
        }
    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::string(argv[1]) == "--help")
    {
        print_usage(argv[0]);
        return 0;
    }

    const Options options = parse_options(argc, argv);

    radar::RadarSystemParams sys;
    sys.prf_hz = options.prf_hz;
    sys.compute_derived_params();

    radar::clutter::SeaClutterConfig cfg;
    cfg.enabled = true;
    cfg.backend = radar::clutter::SeaClutterBackend::GPU;
    cfg.seed = options.seed;
    cfg.sea_state = 3.0f;
    cfg.doppler_sigma_hz = options.sigma_f_hz;
    cfg.doppler_center_hz = options.doppler_center_hz;
    cfg.fir_length = options.fir_length;
    cfg.distribution = static_cast<radar::ClutterDistribution>(options.distribution);
    cfg.weibull_shape = options.weibull_shape;
    cfg.weibull_scale = options.weibull_scale;
    cfg.lognormal_mu = options.lognormal_mu;
    cfg.lognormal_sigma = options.lognormal_sigma;
    cfg.k_shape_nu = options.k_shape_nu;
    cfg.k_texture_bandwidth_hz = options.k_texture_bandwidth_hz;
    cfg.k_lut_size = options.k_lut_size;

    radar::clutter::SeaClutterModel model;
    model.set_config(cfg);
    model.set_system_params(sys);
    const radar::ComplexVec fir = model.get_fir_coefficients(
        options.sigma_f_hz,
        options.doppler_center_hz,
        options.prf_hz,
        options.fir_length);

    const std::vector<radar::Complex> trace = generate_patch_trace(options, fir);
    const int psd_fft_size = std::min(2048, largest_power_of_two_at_most(options.sample_count));
    const std::vector<double> psd = compute_welch_periodogram(trace, psd_fft_size);

    const std::filesystem::path trace_path = options.output_prefix + "_trace.txt";
    const std::filesystem::path psd_path = options.output_prefix + "_psd.txt";
    std::filesystem::create_directories(trace_path.parent_path());
    write_trace_file(trace_path.string(), options, trace);
    write_psd_file(psd_path.string(), options, psd);

    double mean_power = 0.0;
    double mean_amp = 0.0;
    for (const auto sample : trace)
    {
        const double power = static_cast<double>(std::norm(sample));
        mean_power += power;
        mean_amp += std::sqrt(power);
    }
    mean_power /= static_cast<double>(trace.size());
    mean_amp /= static_cast<double>(trace.size());

    std::cout << "Clutter single-patch trace exported\n";
    std::cout << "  trace:       " << trace_path.string() << "\n";
    std::cout << "  psd:         " << psd_path.string() << "\n";
    std::cout << "  samples:     " << options.sample_count << "\n";
    std::cout << "  range_idx:   " << options.range_idx << "\n";
    std::cout << "  az_idx:      " << options.az_idx << "\n";
    std::cout << "  fir_length:  " << fir.size() << "\n";
    std::cout << "  distribution:" << options.distribution << "\n";
    std::cout << "  k_shape_nu:  " << options.k_shape_nu << "\n";
    std::cout << "  k_texture_bw:" << options.k_texture_bandwidth_hz << "\n";
    std::cout << "  mean_power:  " << mean_power << "\n";
    std::cout << "  mean_amp:    " << mean_amp << "\n";

    return 0;
}
