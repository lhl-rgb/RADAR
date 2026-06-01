#include "antenna/antenna_config.h"
#include "clutter/sea_clutter_config.h"
#include "clutter/sea_clutter_engine.h"
#include "core/radar_system_params.h"
#include "cuda/clutter_gpu.cuh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace
{
    using Clock = std::chrono::steady_clock;

    struct BenchmarkOptions
    {
        int target_range_cells = 2000;
        int rotations = 1;
        float az_patch_step_deg = 0.5f;
        float active_gain_floor_db = -40.0f;
        int fir_length = 13;
        float prf_hz = 1600.0f;
        float rotation_rate_dps = 60.0f;
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

    BenchmarkOptions parse_options(int argc, char **argv)
    {
        BenchmarkOptions options;
        options.target_range_cells = parse_int_arg(argv, argc, 1, options.target_range_cells);
        options.rotations = parse_int_arg(argv, argc, 2, options.rotations);
        options.az_patch_step_deg = parse_float_arg(argv, argc, 3, options.az_patch_step_deg);
        options.active_gain_floor_db = parse_float_arg(argv, argc, 4, options.active_gain_floor_db);
        options.fir_length = parse_int_arg(argv, argc, 5, options.fir_length);
        options.prf_hz = parse_float_arg(argv, argc, 6, options.prf_hz);
        options.rotation_rate_dps = parse_float_arg(argv, argc, 7, options.rotation_rate_dps);
        options.distribution = parse_int_arg(argv, argc, 8, options.distribution);
        options.weibull_shape = parse_float_arg(argv, argc, 9, options.weibull_shape);
        options.weibull_scale = parse_float_arg(argv, argc, 10, options.weibull_scale);
        options.lognormal_mu = parse_float_arg(argv, argc, 11, options.lognormal_mu);
        options.lognormal_sigma = parse_float_arg(argv, argc, 12, options.lognormal_sigma);
        options.k_shape_nu = parse_float_arg(argv, argc, 13, options.k_shape_nu);
        options.k_texture_bandwidth_hz = parse_float_arg(argv, argc, 14, options.k_texture_bandwidth_hz);
        options.k_lut_size = parse_int_arg(argv, argc, 15, options.k_lut_size);
        return options;
    }

    void print_usage(const char *program)
    {
        std::cout << "Usage: " << program
                  << " [range_cells=2000] [rotations=1] [az_patch_step_deg=0.5]"
                  << " [active_gain_floor_db=-40] [fir_length=13]"
                  << " [prf_hz=1600] [rotation_rate_dps=60]"
                  << " [distribution=0] [weibull_shape=2] [weibull_scale=1.414]"
                  << " [lognormal_mu=-1] [lognormal_sigma=1]"
                  << " [k_shape_nu=1] [k_texture_bandwidth_hz=2] [k_lut_size=4096]\n";
    }

    radar::RadarSystemParams make_system_params(const BenchmarkOptions &options)
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
        ant.az_beamwidth_deg = 5.0f;
        ant.el_beamwidth_deg = 5.0f;
        ant.peak_gain_db = 35.0f;
        return ant;
    }

    radar::clutter::SeaClutterConfig make_clutter_config(const BenchmarkOptions &options)
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

    double elapsed_ms(const Clock::time_point &start, const Clock::time_point &finish)
    {
        return std::chrono::duration<double, std::milli>(finish - start).count();
    }
}

int main(int argc, char **argv)
{
    if (argc > 1 && std::string(argv[1]) == "--help")
    {
        print_usage(argv[0]);
        return 0;
    }

    const BenchmarkOptions options = parse_options(argc, argv);

    std::string cuda_error;
    if (!radar::cuda::is_clutter_gpu_available(&cuda_error))
    {
        std::cout << "CUDA clutter benchmark skipped: " << cuda_error << "\n";
        return 0;
    }

    const radar::RadarSystemParams sys = make_system_params(options);
    const radar::antenna::AntennaConfig ant = make_antenna_config();
    const radar::clutter::SeaClutterConfig cfg = make_clutter_config(options);

    radar::clutter::SeaClutterEngine engine;
    engine.set_config(cfg);
    engine.set_antenna_config(ant);

    const auto init_start = Clock::now();
    if (!engine.initialize(sys))
    {
        std::cerr << "SeaClutterEngine initialize failed: " << engine.last_error() << "\n";
        return 1;
    }
    const auto init_finish = Clock::now();

    const float az_step_per_prt_deg = options.rotation_rate_dps / sys.prf_hz;
    const int pulses_per_rotation =
        std::max(1, static_cast<int>(std::lround(360.0f / az_step_per_prt_deg)));
    const int total_pulses = pulses_per_rotation * options.rotations;

    radar::PulseEcho echo;
    for (int i = 0; i < std::min(8, pulses_per_rotation); ++i)
    {
        const radar::BeamPoint beam{std::fmod(i * az_step_per_prt_deg, 360.0f), 0.0f};
        if (!engine.get_clutter_for_pulse(beam, i, echo))
        {
            std::cerr << "Warmup failed: " << engine.last_error() << "\n";
            return 1;
        }
    }

    double checksum = 0.0;
    double active_build_ms_sum = 0.0;
    double active_upload_ms_sum = 0.0;
    double kernel_sync_ms_sum = 0.0;
    double device_to_host_ms_sum = 0.0;
    double host_convert_ms_sum = 0.0;
    double gpu_total_ms_sum = 0.0;
    int active_count_min = 0;
    int active_count_max = 0;
    double active_count_sum = 0.0;
    const auto run_start = Clock::now();
    for (int pulse = 0; pulse < total_pulses; ++pulse)
    {
        const float az_deg = std::fmod(pulse * az_step_per_prt_deg, 360.0f);
        const radar::BeamPoint beam{az_deg, 0.0f};
        radar::clutter::SeaClutterTiming timing;
        if (!engine.get_clutter_for_pulse_timed(beam, pulse, echo, &timing))
        {
            std::cerr << "Pulse " << pulse << " failed: " << engine.last_error() << "\n";
            return 1;
        }

        active_build_ms_sum += timing.active_build_ms;
        active_upload_ms_sum += timing.gpu.active_upload_ms;
        kernel_sync_ms_sum += timing.gpu.kernel_sync_ms;
        device_to_host_ms_sum += timing.gpu.device_to_host_ms;
        host_convert_ms_sum += timing.gpu.host_convert_ms;
        gpu_total_ms_sum += timing.gpu.total_ms;
        const int active_count = engine.last_active_count();
        if (pulse == 0)
        {
            active_count_min = active_count;
            active_count_max = active_count;
        }
        else
        {
            active_count_min = std::min(active_count_min, active_count);
            active_count_max = std::max(active_count_max, active_count);
        }
        active_count_sum += active_count;

        for (const auto &sample : echo)
        {
            checksum += static_cast<double>(sample.real()) * sample.real() +
                        static_cast<double>(sample.imag()) * sample.imag();
        }
    }
    const auto run_finish = Clock::now();

    const double init_ms = elapsed_ms(init_start, init_finish);
    const double run_ms = elapsed_ms(run_start, run_finish);
    const double avg_us_per_prt = (run_ms * 1000.0) / static_cast<double>(total_pulses);
    const double prt_per_sec = 1000.0 * static_cast<double>(total_pulses) / run_ms;
    const double simulated_seconds = static_cast<double>(total_pulses) / sys.prf_hz;
    const double realtime_ratio = simulated_seconds / (run_ms / 1000.0);
    const double pulses_d = static_cast<double>(total_pulses);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "CUDA clutter one-rotation benchmark\n";
    std::cout << "  range_cells_actual:      " << sys.samples_per_pulse << "\n";
    std::cout << "  az_cells:                " << engine.num_az_cells() << "\n";
    std::cout << "  az_patch_step_deg:       " << cfg.az_patch_step_deg << "\n";
    std::cout << "  active_gain_floor_db:    " << cfg.active_gain_floor_db << "\n";
    std::cout << "  fir_length:              " << engine.fir_length() << "\n";
    std::cout << "  distribution:            " << options.distribution << "\n";
    std::cout << "  weibull_shape:           " << cfg.weibull_shape << "\n";
    std::cout << "  weibull_scale:           " << cfg.weibull_scale << "\n";
    std::cout << "  lognormal_mu:            " << cfg.lognormal_mu << "\n";
    std::cout << "  lognormal_sigma:         " << cfg.lognormal_sigma << "\n";
    std::cout << "  k_shape_nu:              " << cfg.k_shape_nu << "\n";
    std::cout << "  k_texture_bandwidth_hz:  " << cfg.k_texture_bandwidth_hz << "\n";
    std::cout << "  k_lut_size:              " << cfg.k_lut_size << "\n";
    std::cout << "  active_count_avg:        " << (active_count_sum / pulses_d) << "\n";
    std::cout << "  active_count_min:        " << active_count_min << "\n";
    std::cout << "  active_count_max:        " << active_count_max << "\n";
    std::cout << "  prf_hz:                  " << sys.prf_hz << "\n";
    std::cout << "  rotation_rate_dps:       " << options.rotation_rate_dps << "\n";
    std::cout << "  pulses_per_rotation:     " << pulses_per_rotation << "\n";
    std::cout << "  rotations_measured:      " << options.rotations << "\n";
    std::cout << "  total_pulses:            " << total_pulses << "\n";
    std::cout << "  init_ms:                 " << init_ms << "\n";
    std::cout << "  run_ms:                  " << run_ms << "\n";
    std::cout << "  avg_us_per_prt:          " << avg_us_per_prt << "\n";
    std::cout << "  generated_prt_per_sec:   " << prt_per_sec << "\n";
    std::cout << "  simulated_seconds:       " << simulated_seconds << "\n";
    std::cout << "  realtime_ratio:          " << realtime_ratio << "x\n";
    std::cout << "  avg_active_build_us:     " << (active_build_ms_sum * 1000.0 / pulses_d) << "\n";
    std::cout << "  avg_active_upload_us:    " << (active_upload_ms_sum * 1000.0 / pulses_d) << "\n";
    std::cout << "  avg_kernel_sync_us:      " << (kernel_sync_ms_sum * 1000.0 / pulses_d) << "\n";
    std::cout << "  avg_d2h_copy_us:         " << (device_to_host_ms_sum * 1000.0 / pulses_d) << "\n";
    std::cout << "  avg_host_convert_us:     " << (host_convert_ms_sum * 1000.0 / pulses_d) << "\n";
    std::cout << "  avg_gpu_call_total_us:   " << (gpu_total_ms_sum * 1000.0 / pulses_d) << "\n";
    std::cout << "  checksum:                " << std::scientific << std::setprecision(6)
              << checksum << "\n";

    return 0;
}
