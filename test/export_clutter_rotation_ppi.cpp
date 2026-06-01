#include "antenna/antenna_config.h"
#include "clutter/sea_clutter_config.h"
#include "clutter/sea_clutter_engine.h"
#include "core/radar_system_params.h"
#include "cuda/clutter_gpu.cuh"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace
{
    struct ExportOptions
    {
        int target_range_cells = 8000;
        float az_patch_step_deg = 0.5f;
        float active_gain_floor_db = -20.0f;
        int fir_length = 13;
        float prf_hz = 1600.0f;
        float rotation_rate_dps = 72.0f;
        std::string output_prefix = "out/clutter_rotation";
        int iq_range_idx_1based = 1000;
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

    float parse_float_arg(char **argv, int argc, int index, float fallback)
    {
        if (index >= argc)
        {
            return fallback;
        }
        return static_cast<float>(std::atof(argv[index]));
    }

    std::string parse_string_arg(char **argv, int argc, int index, const std::string &fallback)
    {
        if (index >= argc)
        {
            return fallback;
        }
        return argv[index];
    }

    ExportOptions parse_options(int argc, char **argv)
    {
        ExportOptions options;
        options.target_range_cells = parse_int_arg(argv, argc, 1, options.target_range_cells);
        options.az_patch_step_deg = parse_float_arg(argv, argc, 2, options.az_patch_step_deg);
        options.active_gain_floor_db = parse_float_arg(argv, argc, 3, options.active_gain_floor_db);
        options.fir_length = parse_int_arg(argv, argc, 4, options.fir_length);
        options.prf_hz = parse_float_arg(argv, argc, 5, options.prf_hz);
        options.rotation_rate_dps = parse_float_arg(argv, argc, 6, options.rotation_rate_dps);
        options.output_prefix = parse_string_arg(argv, argc, 7, options.output_prefix);
        options.iq_range_idx_1based = parse_int_arg(argv, argc, 8, options.iq_range_idx_1based);
        options.distribution = parse_int_arg(argv, argc, 9, options.distribution);
        options.weibull_shape = parse_float_arg(argv, argc, 10, options.weibull_shape);
        options.weibull_scale = parse_float_arg(argv, argc, 11, options.weibull_scale);
        options.lognormal_mu = parse_float_arg(argv, argc, 12, options.lognormal_mu);
        options.lognormal_sigma = parse_float_arg(argv, argc, 13, options.lognormal_sigma);
        options.k_shape_nu = parse_float_arg(argv, argc, 14, options.k_shape_nu);
        options.k_texture_bandwidth_hz = parse_float_arg(argv, argc, 15, options.k_texture_bandwidth_hz);
        options.k_lut_size = parse_int_arg(argv, argc, 16, options.k_lut_size);
        return options;
    }

    void print_usage(const char *program)
    {
        std::cout << "Usage: " << program
                  << " [range_cells=2000] [az_patch_step_deg=0.5]"
                  << " [active_gain_floor_db=-40] [fir_length=13]"
                  << " [prf_hz=1600] [rotation_rate_dps=60]"
                  << " [output_prefix=out/clutter_rotation] [iq_range_idx_1based=1000]"
                  << " [distribution=0] [weibull_shape=2] [weibull_scale=1.414]"
                  << " [lognormal_mu=-1] [lognormal_sigma=1]"
                  << " [k_shape_nu=1] [k_texture_bandwidth_hz=2] [k_lut_size=4096]\n";
    }

    radar::RadarSystemParams make_system_params(const ExportOptions &options)
    {
        radar::RadarSystemParams sys;
        sys.prf_hz = options.prf_hz;
        sys.fs_hz = 20.0e6f;
        sys.bw_hz = 10.0e6f;
        sys.pulse_width_s = 2.0e-6f;
        sys.peak_power_w = 5000.0f;
        sys.min_range_m = 1000.0f;
        sys.antenna_height_m = 15.0f;

        const float fast_time_s =
            static_cast<float>(options.target_range_cells) / sys.fs_hz;
        const float range_span_m =
            std::max(1.0f, (fast_time_s - sys.pulse_width_s) * radar::C * 0.5f);
        sys.max_range_m = sys.min_range_m + range_span_m;
        sys.compute_derived_params();
        return sys;
    }

    radar::antenna::AntennaConfig make_antenna_config()
    {
        radar::antenna::AntennaConfig ant;
        ant.az_beamwidth_deg = 3.0f;
        ant.el_beamwidth_deg = 5.0f;
        ant.peak_gain_db = 35.0f;
        return ant;
    }

    radar::clutter::SeaClutterConfig make_clutter_config(const ExportOptions &options)
    {
        radar::clutter::SeaClutterConfig cfg;
        cfg.enabled = true;
        cfg.backend = radar::clutter::SeaClutterBackend::GPU;
        cfg.seed = 2026;
        cfg.sea_state = 3.0f;
        cfg.doppler_center_hz = 0.0f;
        cfg.doppler_sigma_hz = 40.0f;
        cfg.fir_length = options.fir_length;
        cfg.az_patch_step_deg = options.az_patch_step_deg;
        cfg.active_gain_floor_db = options.active_gain_floor_db;
        cfg.distribution = static_cast<radar::ClutterDistribution>(options.distribution);
        cfg.weibull_shape = options.weibull_shape;
        cfg.weibull_scale = options.weibull_scale;
        cfg.lognormal_mu = options.lognormal_mu;
        cfg.lognormal_sigma = options.lognormal_sigma;
        cfg.k_shape_nu = options.k_shape_nu;
        cfg.k_texture_bandwidth_hz = options.k_texture_bandwidth_hz;
        cfg.k_lut_size = options.k_lut_size;
        cfg.ground_range_min_m = -1.0f;
        cfg.ground_range_max_m = -1.0f;
        return cfg;
    }

    bool ensure_parent_directory(const std::filesystem::path &path)
    {
        const auto parent = path.parent_path();
        if (parent.empty())
        {
            return true;
        }
        std::error_code ec;
        std::filesystem::create_directories(parent, ec);
        return !ec;
    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::string(argv[1]) == "--help")
    {
        print_usage(argv[0]);
        return 0;
    }

    const ExportOptions options = parse_options(argc, argv);

    std::string cuda_error;
    if (!radar::cuda::is_clutter_gpu_available(&cuda_error))
    {
        std::cout << "CUDA clutter rotation export skipped: " << cuda_error << "\n";
        return 0;
    }

    const radar::RadarSystemParams sys = make_system_params(options);
    const radar::antenna::AntennaConfig ant = make_antenna_config();
    const radar::clutter::SeaClutterConfig cfg = make_clutter_config(options);

    radar::clutter::SeaClutterEngine engine;
    engine.set_config(cfg);
    engine.set_antenna_config(ant);
    if (!engine.initialize(sys))
    {
        std::cerr << "SeaClutterEngine initialize failed: " << engine.last_error() << "\n";
        return 1;
    }

    const float az_step_per_prt_deg = options.rotation_rate_dps / sys.prf_hz;
    const int pulses_per_rotation =
        std::max(1, static_cast<int>(std::lround(360.0f / az_step_per_prt_deg)));
    const int range_cells = sys.samples_per_pulse;
    const int iq_range_idx =
        std::max(0, std::min(range_cells - 1, options.iq_range_idx_1based - 1));

    const std::filesystem::path prefix_path(options.output_prefix);
    const std::filesystem::path power_path = options.output_prefix + "_power.bin";
    const std::filesystem::path meta_path = options.output_prefix + "_meta.txt";
    const std::filesystem::path iq_path = options.output_prefix + "_iq.txt";
    if (!ensure_parent_directory(prefix_path))
    {
        std::cerr << "Failed to create output directory for " << options.output_prefix << "\n";
        return 1;
    }

    std::ofstream power_file(power_path, std::ios::binary);
    if (!power_file)
    {
        std::cerr << "Failed to open " << power_path << "\n";
        return 1;
    }

    std::ofstream iq_file(iq_path);
    if (!iq_file)
    {
        std::cerr << "Failed to open " << iq_path << "\n";
        return 1;
    }

    iq_file << std::setprecision(9);
    iq_file << "# columns: pulse az_deg real imag amplitude power\n";

    radar::PulseEcho echo;
    std::vector<float> power_column(static_cast<std::size_t>(range_cells));
    double mean_power_sum = 0.0;
    float min_power = std::numeric_limits<float>::infinity();
    float max_power = 0.0f;

    for (int pulse = 0; pulse < pulses_per_rotation; ++pulse)
    {
        const float az_deg = std::fmod(pulse * az_step_per_prt_deg, 360.0f);
        const radar::BeamPoint beam{az_deg, 0.0f};
        if (!engine.get_clutter_for_pulse(beam, pulse, echo))
        {
            std::cerr << "Pulse " << pulse << " failed: " << engine.last_error() << "\n";
            return 1;
        }
        if (static_cast<int>(echo.size()) != range_cells)
        {
            std::cerr << "Pulse " << pulse << " size mismatch: " << echo.size()
                      << " vs " << range_cells << "\n";
            return 1;
        }

        for (int range_idx = 0; range_idx < range_cells; ++range_idx)
        {
            const float power = static_cast<float>(std::norm(echo[static_cast<std::size_t>(range_idx)]));
            power_column[static_cast<std::size_t>(range_idx)] = power;
            mean_power_sum += static_cast<double>(power);
            min_power = std::min(min_power, power);
            max_power = std::max(max_power, power);
        }

        power_file.write(reinterpret_cast<const char *>(power_column.data()),
                         static_cast<std::streamsize>(power_column.size() * sizeof(float)));
        if (!power_file)
        {
            std::cerr << "Failed writing " << power_path << "\n";
            return 1;
        }

        const auto sample = echo[static_cast<std::size_t>(iq_range_idx)];
        const float sample_power = static_cast<float>(std::norm(sample));
        iq_file << pulse << ' '
                << az_deg << ' '
                << sample.real() << ' '
                << sample.imag() << ' '
                << std::sqrt(std::max(sample_power, 0.0f)) << ' '
                << sample_power << '\n';
    }

    power_file.close();
    iq_file.close();

    if (!std::isfinite(min_power))
    {
        min_power = 0.0f;
    }
    const double mean_power =
        mean_power_sum / static_cast<double>(range_cells) / static_cast<double>(pulses_per_rotation);
    const float iq_range_m =
        sys.min_range_m + static_cast<float>(iq_range_idx) * sys.range_bin_size_m;

    std::ofstream meta_file(meta_path);
    if (!meta_file)
    {
        std::cerr << "Failed to open " << meta_path << "\n";
        return 1;
    }
    meta_file << std::setprecision(10);
    meta_file << "# key value\n";
    meta_file << "range_cells " << range_cells << "\n";
    meta_file << "pulses " << pulses_per_rotation << "\n";
    meta_file << "min_range_m " << sys.min_range_m << "\n";
    meta_file << "range_bin_size_m " << sys.range_bin_size_m << "\n";
    meta_file << "prf_hz " << sys.prf_hz << "\n";
    meta_file << "rotation_rate_dps " << options.rotation_rate_dps << "\n";
    meta_file << "az_step_deg " << az_step_per_prt_deg << "\n";
    meta_file << "az_start_deg 0\n";
    meta_file << "az_patch_step_deg " << cfg.az_patch_step_deg << "\n";
    meta_file << "active_gain_floor_db " << cfg.active_gain_floor_db << "\n";
    meta_file << "fir_length " << engine.fir_length() << "\n";
    meta_file << "distribution " << options.distribution << "\n";
    meta_file << "weibull_shape " << cfg.weibull_shape << "\n";
    meta_file << "weibull_scale " << cfg.weibull_scale << "\n";
    meta_file << "lognormal_mu " << cfg.lognormal_mu << "\n";
    meta_file << "lognormal_sigma " << cfg.lognormal_sigma << "\n";
    meta_file << "k_shape_nu " << cfg.k_shape_nu << "\n";
    meta_file << "k_texture_bandwidth_hz " << cfg.k_texture_bandwidth_hz << "\n";
    meta_file << "k_lut_size " << cfg.k_lut_size << "\n";
    meta_file << "doppler_sigma_hz " << cfg.doppler_sigma_hz << "\n";
    meta_file << "az_beamwidth_deg " << ant.az_beamwidth_deg << "\n";
    meta_file << "el_beamwidth_deg " << ant.el_beamwidth_deg << "\n";
    meta_file << "peak_gain_db " << ant.peak_gain_db << "\n";
    meta_file << "power_file_layout range_by_pulse_float32\n";
    meta_file << "iq_file_layout columns_pulse_az_real_imag_amplitude_power\n";
    meta_file << "iq_range_idx " << (iq_range_idx + 1) << "\n";
    meta_file << "iq_range_m " << iq_range_m << "\n";
    meta_file << "min_power " << min_power << "\n";
    meta_file << "max_power " << max_power << "\n";
    meta_file << "mean_power " << mean_power << "\n";
    meta_file.close();

    std::cout << "CUDA clutter rotation exported\n";
    std::cout << "  power:      " << power_path.string() << "\n";
    std::cout << "  meta:       " << meta_path.string() << "\n";
    std::cout << "  iq:         " << iq_path.string() << "\n";
    std::cout << "  range_cells:" << range_cells << "\n";
    std::cout << "  pulses:     " << pulses_per_rotation << "\n";
    std::cout << "  distribution:" << options.distribution << "\n";
    std::cout << "  mean_power: " << std::scientific << mean_power << "\n";

    return 0;
}
