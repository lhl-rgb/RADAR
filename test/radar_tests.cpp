/**
 * @file radar_tests.cpp
 * @brief 雷达仿真系统单元测试（机械扫描雷达版本）
 */

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <complex>
#include <random>
#include <fstream>
#include <filesystem>
#include <limits>
#include <algorithm>

#include <fftw3.h>

#include "core/types.h"
#include "core/radar_config.h"
#include "core/tools/math_utils.h"
#include "core/radar_system_params.h"
#include "waveform/waveform_generator.h"
#include "waveform/waveform_config.h"
#include "noise/noise_engine.h"
#include "noise/noise_config.h"
#include "clutter/sea_clutter_config.h"
#include "clutter/sea_clutter_engine.h"
#include "cuda/clutter_gpu.cuh"
#include "cuda/clutter_rng.cuh"
#include "antenna/antenna_model.h"
#include "antenna/mechanical_scanner.h"
#include "core/tools/data_exporter.h"
#include "core/simulation_engine.h"
#include "core/echo_frame.h"
#include "core/metrics_collector.h"
#include "output/echo_sink.h"
#include "preview/preview_service.h"
#include "target/target_config.h"
#include "target/target_engine.h"

using namespace radar;

namespace
{

    int tests_passed = 0;
    int tests_failed = 0;

    void assert_true(bool condition, const std::string &test_name, const std::string &detail = "")
    {
        if (condition)
        {
            std::cout << "  [PASS] " << test_name;
            if (!detail.empty())
                std::cout << " (" << detail << ")";
            std::cout << std::endl;
            tests_passed++;
        }
        else
        {
            std::cout << "  [FAIL] " << test_name;
            if (!detail.empty())
                std::cout << " (" << detail << ")";
            std::cout << std::endl;
            tests_failed++;
        }
    }

    void assert_near(Scalar actual, Scalar expected, Scalar tol, const std::string &test_name)
    {
        Scalar diff = std::abs(actual - expected);
        bool pass = diff <= tol;
        std::string detail = "expected=" + std::to_string(expected) +
                             ", actual=" + std::to_string(actual) +
                             ", diff=" + std::to_string(diff);
        assert_true(pass, test_name, detail);
    }

    bool require_clutter_gpu_runtime(const std::string &test_name)
    {
#if RADAR_HAS_CUDA
        std::string error;
        if (radar::cuda::is_clutter_gpu_available(&error))
        {
            return true;
        }
        assert_true(true, test_name + " skipped: CUDA runtime unavailable", error);
        return false;
#else
        assert_true(true, test_name + " skipped: CUDA build disabled");
        return false;
#endif
    }

} // namespace

// ============================================================================
// Math Utils 测试
// ============================================================================
void test_math_utils()
{
    std::cout << "\n=== Math Utils Tests ===" << std::endl;

    assert_near(radar::math::db_to_linear(0.0), 1.0, 1e-10, "db_to_linear(0 dB)");
    assert_near(radar::math::db_to_linear(10.0), 10.0, 1e-10, "db_to_linear(10 dB)");
    assert_near(radar::math::db_to_linear(3.0), 2.0, 0.1, "db_to_linear(3 dB) ≈ 2");

    assert_near(radar::math::linear_to_db(1.0), 0.0, 1e-10, "linear_to_db(1)");
    assert_near(radar::math::linear_to_db(10.0), 10.0, 0.1, "linear_to_db(10)");

    assert_near(radar::math::deg_to_rad(180.0), radar::PI, 1e-10, "deg_to_rad(180)");
    assert_near(radar::math::rad_to_deg(radar::PI), 180.0, 1e-10, "rad_to_deg(PI)");

    assert_near(radar::math::wrap_azimuth_deg(0.0), 0.0, 1e-10, "wrap_azimuth(0)");
    assert_near(radar::math::wrap_azimuth_deg(360.0), 0.0, 1e-10, "wrap_azimuth(360)");
    assert_near(radar::math::wrap_azimuth_deg(190.0), -170.0, 1e-10, "wrap_azimuth(190)");
    assert_near(radar::math::wrap_azimuth_deg(-190.0), 170.0, 1e-10, "wrap_azimuth(-190)");

    assert_near(radar::math::clamp_positive_eps(0.5), 0.5, 1e-10, "clamp_positive(0.5)");
    assert_near(radar::math::clamp_positive_eps(0.0), radar::EPSILON, 1e-10, "clamp_positive(0)");
    assert_near(radar::math::clamp_positive_eps(-1.0), radar::EPSILON, 1e-10, "clamp_positive(-1)");

    assert_near(radar::math::safe_div(10.0, 2.0), 5.0, 1e-10, "safe_div(10/2)");
    assert_near(radar::math::safe_div(10.0, 0.0), 10.0 / radar::EPSILON, 1e-6, "safe_div(10/0)");
}

// ============================================================================
// RadarSystemParams 测试
// ============================================================================
void test_radar_system_params()
{
    std::cout << "\n=== RadarSystemParams Tests ===" << std::endl;

    radar::RadarSystemParams params;

    std::string error;
    bool valid = params.validate(error);
    assert_true(valid, "Default params valid", error);

    assert_near(params.wavelength_m, radar::C / params.fc_hz, 1e-6, "wavelength");
    assert_near(params.pri_s, 1.0 / params.prf_hz, 1e-10, "PRI");
    assert_near(params.range_resolution_m, radar::C / (2.0 * params.bw_hz), 10.0, "range resolution");
    assert_true(params.samples_per_pulse > 0, "samples_per_pulse derived");

    radar::RadarSystemParams invalid_params;
    invalid_params.pulse_width_s = 0.003;
    invalid_params.prf_hz = 500.0;
    std::string error2;
    bool valid2 = invalid_params.validate(error2);
    assert_true(!valid2, "Invalid pulse_width rejected", error2);

    invalid_params = radar::RadarSystemParams();
    invalid_params.fs_hz = 1e6;
    invalid_params.bw_hz = 10e6;
    valid = invalid_params.validate(error);
    assert_true(!valid, "fs < bw rejected", error);
}

// ============================================================================
// RadarConfig Tests
// ============================================================================
void test_radar_config()
{
    std::cout << "\n=== RadarConfig Tests ===" << std::endl;

    {
        radar::RadarConfig cfg;
        cfg.simulation.scan_count = 3;

        std::string error;
        bool valid = cfg.validate(error);
        assert_true(valid, "RadarConfig validates positive scan_count");
        assert_true(cfg.mech_scan.pulses_per_rotation > 0,
                    "RadarConfig computes mechanical scan derived params");
        assert_true(cfg.mech_scan.azimuth_step_per_prt_deg > 0.0,
                    "RadarConfig computes azimuth step per PRT");
    }

    {
        radar::RadarConfig cfg;
        cfg.simulation.scan_count = 0;

        std::string error;
        bool valid = cfg.validate(error);
        assert_true(!valid && error.find("simulation.scan_count") != std::string::npos,
                    "RadarConfig rejects non-positive scan_count", error);
    }
}

// ============================================================================
// WaveformGenerator 测试
// ============================================================================
void test_waveform_generator()
{
    std::cout << "\n=== WaveformGenerator Tests ===" << std::endl;

    radar::RadarSystemParams sys;
    radar::waveform::WaveformConfig cfg;

    // --- LFM 波形测试 ---
    {
        radar::waveform::WaveformGenerator gen;
        cfg.waveform_type = radar::WaveformType::LFM;
        gen.set_config(cfg);
        gen.set_system_params(sys);

        bool init = gen.initialize();
        assert_true(init, "LFM waveform initialize");

        const auto &waveform = gen.get_waveform();
        assert_true(!waveform.empty(), "LFM waveform not empty");
        assert_true(static_cast<int>(waveform.size()) > 0,
                    "LFM waveform size", "size=" + std::to_string(waveform.size()));

        auto mf = gen.generate_matched_filter();
        assert_true(mf.size() == waveform.size(), "Matched filter size");

        bool mf_correct = true;
        for (std::size_t i = 0; i < waveform.size(); ++i)
        {
            if (std::abs(mf[i] - std::conj(waveform[waveform.size() - 1 - i])) > radar::EPSILON)
            {
                mf_correct = false;
                break;
            }
        }
        assert_true(mf_correct, "Matched filter is conjugate reversed");
    }

    // --- NLFM 波形测试 ---
    {
        radar::waveform::WaveformGenerator gen;
        cfg.waveform_type = radar::WaveformType::NLFM;
        cfg.nlfm_window_type = radar::WindowType::Hamming;
        gen.set_config(cfg);
        gen.set_system_params(sys);

        bool init = gen.initialize();
        assert_true(init, "NLFM waveform initialize");

        const auto &waveform = gen.get_waveform();
        assert_true(!waveform.empty(), "NLFM waveform not empty");

        Scalar total_power = 0.0;
        for (const auto &s : waveform)
        {
            total_power += std::norm(s);
        }
        Scalar avg_power = total_power / waveform.size();
        assert_near(avg_power, 1.0, 0.1, "NLFM average power ≈ 1");
    }

    // --- 相位编码波形测试 ---
    {
        radar::waveform::WaveformGenerator gen;
        cfg.waveform_type = radar::WaveformType::PHASE_CODED;
        cfg.phase_code_type = radar::PhaseCodeType::Barker13;
        gen.set_config(cfg);
        gen.set_system_params(sys);

        bool init = gen.initialize();
        assert_true(init, "Barker13 waveform initialize");

        const auto &waveform = gen.get_waveform();
        assert_true(!waveform.empty(), "Barker13 waveform not empty");

        bool constant_envelope = true;
        Scalar ref_mag = std::abs(waveform[0]);
        for (const auto &s : waveform)
        {
            if (std::abs(std::abs(s) - ref_mag) > 0.1)
            {
                constant_envelope = false;
                break;
            }
        }
        assert_true(constant_envelope, "Barker13 constant envelope");
    }

    // --- CW 波形测试 ---
    {
        radar::waveform::WaveformGenerator gen;
        cfg.waveform_type = radar::WaveformType::CW;
        gen.set_config(cfg);
        gen.set_system_params(sys);

        bool init = gen.initialize();
        assert_true(init, "CW waveform initialize");

        const auto &waveform = gen.get_waveform();
        assert_true(!waveform.empty(), "CW waveform not empty");

        bool constant = true;
        for (const auto &s : waveform)
        {
            if (std::abs(std::abs(s) - 1.0) > 0.1)
            {
                constant = false;
                break;
            }
        }
        assert_true(constant, "CW constant envelope");
    }
}

// ============================================================================
// NoiseEngine 测试
// ============================================================================
void test_noise_engine()
{
    std::cout << "\n=== NoiseEngine Tests ===" << std::endl;

    auto vectors_identical = [](const ComplexVec &lhs, const ComplexVec &rhs, Scalar tol)
    {
        if (lhs.size() != rhs.size())
        {
            return false;
        }
        for (std::size_t i = 0; i < lhs.size(); ++i)
        {
            if (std::abs(lhs[i] - rhs[i]) > tol)
            {
                return false;
            }
        }
        return true;
    };

    auto vector_power = [](const ComplexVec &signal)
    {
        Scalar total_power = 0.0;
        for (const auto &sample : signal)
        {
            total_power += std::norm(sample);
        }
        return total_power;
    };

    radar::RadarSystemParams sys;

    // --- ComplexSigma 模式 ---
    {
        radar::noise::NoiseEngine engine;
        radar::noise::NoiseConfig cfg;
        cfg.mode = radar::NoiseLevelMode::ComplexSigma;
        cfg.sigma_complex = 1e-3;
        cfg.seed = 12345;

        engine.set_config(cfg);
        engine.set_system_params(sys);
        bool init = engine.initialize();
        assert_true(init, "NoiseEngine ComplexSigma init");

        const std::size_t N = 100000;
        auto noise = engine.generate(N);

        Scalar avg_power = vector_power(noise) / N;
        Scalar expected_power = cfg.sigma_complex * cfg.sigma_complex;

        assert_near(avg_power, expected_power, expected_power * 0.05,
                    "ComplexSigma power statistics");
    }

    // --- NoisePower 模式 ---
    {
        radar::noise::NoiseEngine engine;
        radar::noise::NoiseConfig cfg;
        cfg.mode = radar::NoiseLevelMode::NoisePower;
        cfg.noise_power_w = 1e-6;
        cfg.seed = 12345;

        engine.set_config(cfg);
        engine.set_system_params(sys);
        bool init = engine.initialize();
        assert_true(init, "NoiseEngine NoisePower init");

        const std::size_t N = 100000;
        auto noise = engine.generate(N);

        Scalar avg_power = vector_power(noise) / N;

        assert_near(avg_power, cfg.noise_power_w, cfg.noise_power_w * 0.05,
                    "NoisePower statistics");
    }

    // --- ThermalKTB 模式 ---
    {
        radar::noise::NoiseEngine engine;
        radar::noise::NoiseConfig cfg;
        cfg.mode = radar::NoiseLevelMode::ThermalKTB;
        cfg.system_temperature_k = 290.0;
        cfg.noise_bandwidth_hz = 20e6;
        cfg.seed = 12345;

        engine.set_config(cfg);
        engine.set_system_params(sys);
        bool init = engine.initialize();
        assert_true(init, "NoiseEngine ThermalKTB init");

        Scalar k = 1.380649e-23;
        Scalar F = std::pow(10.0, sys.noise_figure_db / 10.0);
        Scalar expected_power = k * cfg.system_temperature_k * cfg.noise_bandwidth_hz * F;

        assert_near(engine.noise_power_w(), expected_power, expected_power * 0.01,
                    "ThermalKTB power calculation");
    }

    // --- add_noise 测试 ---
    {
        radar::noise::NoiseEngine engine;
        radar::noise::NoiseConfig cfg;
        cfg.mode = radar::NoiseLevelMode::ComplexSigma;
        cfg.sigma_complex = 0.5;
        cfg.seed = 12345;

        engine.set_config(cfg);
        engine.set_system_params(sys);
        engine.initialize();

        ComplexVec signal(1000, Complex(1.0, 0.0));
        const Scalar signal_power_before = vector_power(signal);

        engine.add_noise(signal);

        const Scalar signal_power_after = vector_power(signal);
        bool power_increased = signal_power_after > signal_power_before * 1.1;
        assert_true(power_increased, "add_noise increases power",
                    "before=" + std::to_string(signal_power_before) +
                        ", after=" + std::to_string(signal_power_after));
    }

    // --- CPU 重复性测试 ---
    {
        radar::noise::NoiseEngine engine1, engine2;
        radar::noise::NoiseConfig cfg;
        cfg.seed = 42;
        cfg.backend = radar::noise::NoiseExecutionBackend::CPU;
        engine1.set_config(cfg);
        engine1.set_system_params(sys);
        engine1.initialize();

        engine2.set_config(cfg);
        engine2.set_system_params(sys);
        engine2.initialize();

        auto noise1 = engine1.generate(100);
        auto noise2 = engine2.generate(100);

        assert_true(vectors_identical(noise1, noise2, 1e-10f),
                    "Same seed produces identical CPU noise");
    }

    // --- CPU 连续调用推进测试 ---
    {
        radar::noise::NoiseEngine engine_split, engine_full;
        radar::noise::NoiseConfig cfg;
        cfg.seed = 314159;
        cfg.backend = radar::noise::NoiseExecutionBackend::CPU;

        engine_split.set_config(cfg);
        engine_split.set_system_params(sys);
        engine_split.initialize();
        auto first = engine_split.generate(100);
        auto second = engine_split.generate(50);

        engine_full.set_config(cfg);
        engine_full.set_system_params(sys);
        engine_full.initialize();
        auto all = engine_full.generate(150);

        ComplexVec stitched;
        stitched.reserve(first.size() + second.size());
        stitched.insert(stitched.end(), first.begin(), first.end());
        stitched.insert(stitched.end(), second.begin(), second.end());

        assert_true(vectors_identical(stitched, all, 1e-10f),
                    "CPU generate advances sequence across calls");
    }

    // --- CPU add_noise 推进测试 ---
    {
        radar::noise::NoiseEngine engine_add, engine_generate;
        radar::noise::NoiseConfig cfg;
        cfg.seed = 271828;
        cfg.backend = radar::noise::NoiseExecutionBackend::CPU;

        engine_add.set_config(cfg);
        engine_add.set_system_params(sys);
        engine_add.initialize();

        ComplexVec first(32, Complex(0.0, 0.0));
        ComplexVec second(16, Complex(0.0, 0.0));
        engine_add.add_noise(first);
        engine_add.add_noise(second);

        engine_generate.set_config(cfg);
        engine_generate.set_system_params(sys);
        engine_generate.initialize();
        auto expected = engine_generate.generate(48);

        ComplexVec stitched;
        stitched.reserve(48);
        stitched.insert(stitched.end(), first.begin(), first.end());
        stitched.insert(stitched.end(), second.begin(), second.end());

        assert_true(vectors_identical(stitched, expected, 1e-10f),
                    "CPU add_noise advances sequence consistently");
    }

#if RADAR_HAS_CUDA
    // --- GPU 统计与确定性测试 ---
    {
        radar::noise::NoiseEngine gpu_engine_1, gpu_engine_2;
        radar::noise::NoiseConfig cfg;
        cfg.seed = 20260521;
        cfg.backend = radar::noise::NoiseExecutionBackend::GPU;
        cfg.sigma_complex = 1e-3;

        gpu_engine_1.set_config(cfg);
        gpu_engine_1.set_system_params(sys);
        const bool init1 = gpu_engine_1.initialize();
        assert_true(init1, "NoiseEngine GPU-request init fallback-safe",
                    gpu_engine_1.last_error());

        gpu_engine_2.set_config(cfg);
        gpu_engine_2.set_system_params(sys);
        const bool init2 = gpu_engine_2.initialize();
        assert_true(init2, "NoiseEngine second GPU-request init fallback-safe",
                    gpu_engine_2.last_error());

        if (gpu_engine_1.using_gpu() && gpu_engine_2.using_gpu())
        {
            const auto small_noise = gpu_engine_1.generate(8);
            assert_true(small_noise.size() == 8, "GPU path handles small noise batches");

            gpu_engine_1.reseed(cfg.seed);
            gpu_engine_2.reseed(cfg.seed);

            const std::size_t N = 8192;
            auto noise1 = gpu_engine_1.generate(N);
            auto noise2 = gpu_engine_2.generate(N);
            assert_true(vectors_identical(noise1, noise2, 1e-6f),
                        "Same seed produces identical GPU noise");

            Scalar mean_i = 0.0;
            Scalar mean_q = 0.0;
            for (const auto &sample : noise1)
            {
                mean_i += sample.real();
                mean_q += sample.imag();
            }
            mean_i /= static_cast<Scalar>(N);
            mean_q /= static_cast<Scalar>(N);
            const Scalar expected_power = cfg.sigma_complex * cfg.sigma_complex;
            const Scalar avg_power = vector_power(noise1) / static_cast<Scalar>(N);

            assert_near(mean_i, 0.0f, 2.0e-5f, "GPU noise real mean near zero");
            assert_near(mean_q, 0.0f, 2.0e-5f, "GPU noise imag mean near zero");
            assert_near(avg_power, expected_power, expected_power * 0.05f,
                        "GPU noise power statistics");
        }
        else
        {
            assert_true(!gpu_engine_1.using_gpu() && !gpu_engine_2.using_gpu(),
                        "GPU request falls back to CPU when CUDA runtime unavailable");
        }
    }
#endif
}

// ============================================================================
// Antenna 测试（反射面天线）
// ============================================================================
void test_antenna()
{
    std::cout << "\n=== Antenna Tests (Reflector) ===" << std::endl;

    // --- 反射面天线基本增益测试 ---
    {
        antenna::AntennaConfig cfg;
        cfg.az_beamwidth_deg = 5.0;
        cfg.el_beamwidth_deg = 5.0;
        cfg.peak_gain_db = 35.0;

        antenna::AntennaModel model;
        model.set_config(cfg);
        model.initialize();

        // 主瓣中心增益
        Scalar gain_db = model.gain_db(0.0, 0.0, 0.0, 0.0);
        Scalar norm_pwr = model.normalized_power(0.0, 0.0, 0.0, 0.0);

        assert_near(gain_db, 35.0, 0.01, "Reflector peak gain at boresight");
        assert_near(norm_pwr, 1.0, 1e-6, "Reflector normalized power at boresight");

        // 离轴增益应该降低
        Scalar off_axis_gain = model.gain_db(5.0, 0.0, 0.0, 0.0);
        assert_true(off_axis_gain < gain_db, "Reflector off-axis gain lower",
                    "on_axis=" + std::to_string(gain_db) +
                        ", off_axis=" + std::to_string(off_axis_gain));
    }

    // --- 波束宽度参数直通测试 ---
    {
        antenna::AntennaConfig cfg;
        cfg.az_beamwidth_deg = 6.0;
        cfg.el_beamwidth_deg = 9.0;
        cfg.peak_gain_db = 35.0;

        antenna::AntennaModel model;
        model.set_config(cfg);
        model.initialize();

        assert_near(model.beamwidth_3db_az_deg(), 6.0, 1e-6, "Az beamwidth passthrough");
        assert_near(model.beamwidth_3db_el_deg(), 9.0, 1e-6, "El beamwidth passthrough");
    }

    // --- MechanicalScanner 机械扫描测试 ---
    {
        antenna::AntennaConfig ant_cfg;
        ant_cfg.az_beamwidth_deg = 5.0;
        ant_cfg.el_beamwidth_deg = 5.0;
        ant_cfg.peak_gain_db = 35.0;

        antenna::MechanicalScanConfig scan_cfg;
        scan_cfg.rotation_rate_dps = 60.0;
        scan_cfg.az_start_deg = 0.0;
        scan_cfg.az_end_deg = 360.0;
        scan_cfg.elevation_deg = 0.0;
        scan_cfg.compute_derived_params(1.0 / 1600.0);

        antenna::MechanicalScanner scanner;
        scanner.set_antenna_config(ant_cfg);
        scanner.set_scan_config(scan_cfg);
        bool init = scanner.initialize();

        assert_true(init, "MechanicalScanner initialize");

        int pps = scan_cfg.pulses_per_rotation;
        assert_true(pps > 0, "Pulses per rotation > 0",
                    "pps=" + std::to_string(pps));
        assert_near(scan_cfg.rotation_period_s, 6.0, 1e-6, "MechanicalScan rotation period");

        scanner.reset();
        BeamPoint first = scanner.get_beam_pointing();
        assert_near(first.azimuth_deg, 0.0, 0.01, "Initial azimuth at start");

        Scalar step_deg = scan_cfg.rotation_rate_dps / 1600.0;
        scanner.advance_one_prt();
        BeamPoint second = scanner.get_beam_pointing();
        assert_near(second.azimuth_deg, step_deg, 0.01, "Azimuth after one PRT");

        scanner.reset();
        for (int i = 0; i < pps; ++i)
        {
            scanner.advance_one_prt();
        }
        BeamPoint after_loop = scanner.get_beam_pointing();
        assert_near(after_loop.azimuth_deg, 0.0, 0.5, "MechanicalScanner wraps around 360°");

        antenna::MechanicalScanConfig sector_cfg;
        sector_cfg.rotation_rate_dps = 60.0;
        sector_cfg.az_start_deg = 30.0;
        sector_cfg.az_end_deg = 60.0;
        sector_cfg.elevation_deg = 0.0;
        sector_cfg.compute_derived_params(1.0 / 1600.0);
        scanner.set_scan_config(sector_cfg);
        scanner.initialize();
        scanner.reset();
        assert_true(scanner.is_echo_active(), "Echo active at sector start");
        for (int i = 0; i < 1000 && scanner.is_echo_active(); ++i)
        {
            scanner.advance_one_prt();
        }
        assert_true(!scanner.is_echo_active(), "Echo inactive outside active sector");
    }
}

// ============================================================================
// TargetEngine 测试
// ============================================================================
void test_target_engine()
{
    std::cout << "\n=== TargetEngine Tests ===" << std::endl;

    radar::RadarSystemParams sys;
    radar::target::TargetConfig cfg;
    cfg.enabled = true;
    cfg.enable_swerling = true;
    cfg.enable_beam_gain = true;
    cfg.skip_out_of_beam_targets = false;
    cfg.seed = 123456;
    cfg.beam_gate_threshold_db = -20.0;

    radar::antenna::AntennaConfig ant_cfg;
    ant_cfg.az_beamwidth_deg = 6.0;
    ant_cfg.el_beamwidth_deg = 6.0;
    ant_cfg.peak_gain_db = 35.0;

    radar::antenna::AntennaModel antenna;
    antenna.set_config(ant_cfg);
    antenna.initialize();

    radar::target::TargetEngine engine;
    engine.set_config(cfg);

    radar::TargetState target;
    target.id = 7;
    target.position_m = radar::Vec3(5000.0, 0.0, 0.0);
    target.velocity_mps = radar::Vec3::Zero();
    target.acceleration_mps2 = radar::Vec3::Zero();
    target.motion_model = radar::MotionModel::Stationary;
    target.rcs_mean_m2 = 10.0;
    target.swerling = radar::SwerlingType::Swerling1;
    target.enabled = true;

    engine.set_initial_targets({target});
    bool init = engine.initialize();
    assert_true(init, "TargetEngine initialize", engine.last_error());

    radar::ComplexVec tx_waveform(static_cast<std::size_t>(sys.samples_per_tx), radar::Complex(1.0, 0.0));
    radar::target::BeamView beam{{0.0, 0.0}, &antenna};

    engine.update_targets(0.0);
    radar::target::PulseContext pulse0{0, 0, 0, 0.0};
    radar::PulseEcho echo0;
    bool ok0 = engine.generate_pulse(beam, pulse0, sys, tx_waveform, echo0);
    assert_true(ok0, "TargetEngine generate first pulse", engine.last_error());
    assert_true(echo0.size() == static_cast<std::size_t>(sys.samples_per_pulse),
                "TargetEngine pulse output size");

    const auto &targets_after_pulse0 = engine.get_current_targets();
    assert_true(!targets_after_pulse0.empty(), "TargetEngine current targets available");

    engine.update_targets(sys.pri_s);
    radar::target::PulseContext pulse1{0, 1, 1, sys.pri_s};
    radar::PulseEcho echo1;
    bool ok1 = engine.generate_pulse(beam, pulse1, sys, tx_waveform, echo1);
    assert_true(ok1, "TargetEngine generate second pulse", engine.last_error());

    radar::TargetSnapshot snapshot0;
    radar::TargetSnapshot snapshot1;
    std::string err0, err1;
    bool snap_ok0 = radar::target::TargetKinematics::generate_snapshot(targets_after_pulse0.front(), beam, pulse0, cfg, snapshot0, err0);
    bool snap_ok1 = radar::target::TargetKinematics::generate_snapshot(targets_after_pulse0.front(), beam, pulse1, cfg, snapshot1, err1);
    assert_true(snap_ok0 && snap_ok1, "TargetKinematics generate snapshot for tests", err0 + err1);

    Scalar rcs0 = 0.0;
    Scalar rcs1 = 0.0;
    for (const auto &s : echo0)
    {
        rcs0 += std::norm(s);
    }
    for (const auto &s : echo1)
    {
        rcs1 += std::norm(s);
    }
    assert_true(rcs0 > 0.0f, "First pulse contains target energy");
    assert_true(rcs1 > 0.0f, "Second pulse contains target energy");
    assert_near(snapshot0.gain_linear, snapshot1.gain_linear, 1e-6, "Visit gain remains stable");
    assert_near(rcs0, rcs1, std::max(rcs0, rcs1) * 1e-4f + 1e-6f, "Visit fluctuation remains stable within illumination");
}

// ============================================================================
// EchoSink / Preview / Metrics 测试
// ============================================================================
void test_output_preview_metrics()
{
    std::cout << "\n=== Output Preview Metrics Tests ===" << std::endl;

    {
        radar::output::OutputConfig cfg;
        cfg.enabled = true;
        cfg.sink_type = radar::output::OutputSinkType::Null;
        auto sink = radar::output::create_echo_sink(cfg);
        assert_true(sink != nullptr, "EchoSink factory creates Null sink");
        assert_true(sink->open(cfg), "NullEchoSink open", sink->last_error());

        radar::core::EchoFrame frame;
        frame.pulse_index = 3;
        frame.scan_index = 1;
        frame.beam_az_deg = 12.5f;
        frame.iq.assign(8, radar::Complex(1.0f, 0.0f));
        assert_true(sink->write_frame(frame), "NullEchoSink write frame", sink->last_error());
        assert_true(sink->frames_written() == 1, "NullEchoSink frame count");
        sink->close();
    }

    {
        const std::string dir = "out/test_echo_sink";
        std::filesystem::remove_all(dir);
        radar::output::OutputConfig cfg;
        cfg.enabled = true;
        cfg.sink_type = radar::output::OutputSinkType::File;
        cfg.output_dir = dir;
        cfg.file_prefix = "unit_echo";
        auto sink = radar::output::create_echo_sink(cfg);
        assert_true(sink->open(cfg), "FileEchoSink open", sink->last_error());

        radar::core::EchoFrame frame;
        frame.pulse_index = 0;
        frame.scan_index = 0;
        frame.timestamp_s = 0.25f;
        frame.beam_az_deg = 45.0f;
        frame.range_bin_size_m = 7.5f;
        frame.iq = {radar::Complex(1.0f, 2.0f), radar::Complex(3.0f, 4.0f)};
        assert_true(sink->write_frame(frame), "FileEchoSink write frame", sink->last_error());
        sink->close();

        assert_true(std::filesystem::exists(dir + "/unit_echo.dat"), "FileEchoSink DAT created");
        assert_true(std::filesystem::exists(dir + "/unit_echo_metadata.json"), "FileEchoSink metadata created");
    }

    {
        radar::output::OutputConfig cfg;
        cfg.enabled = true;
        cfg.sink_type = radar::output::OutputSinkType::Udp;
        cfg.udp.target_ip = "127.0.0.1";
        cfg.udp.target_port = 19001;
        auto sink = radar::output::create_echo_sink(cfg);
        assert_true(sink != nullptr, "EchoSink factory creates UDP sink");
        const bool opened = sink->open(cfg);
        if (!opened && sink->last_error().find("Operation not permitted") != std::string::npos)
        {
            assert_true(true, "UdpEchoSink socket test skipped: socket not permitted", sink->last_error());
        }
        else
        {
            assert_true(opened, "UdpEchoSink open", sink->last_error());

            radar::core::EchoFrame frame;
            frame.pulse_index = 9;
            frame.scan_index = 2;
            frame.timestamp_s = 0.125f;
            frame.beam_az_deg = 30.0f;
            frame.range_bin_size_m = 7.5f;
            frame.iq = {radar::Complex(1.0f, 0.0f), radar::Complex(0.0f, -1.0f)};
            assert_true(sink->write_frame(frame), "UdpEchoSink write frame", sink->last_error());
            assert_true(sink->frames_written() == 1, "UdpEchoSink frame count");
            sink->close();
        }
    }

    {
        radar::preview::PreviewService preview;
        radar::preview::PreviewConfig cfg;
        cfg.range_profile_stride_pulses = 1;
        cfg.ppi_range_bins = 4;
        cfg.ppi_az_bins = 8;
        preview.set_config(cfg);
        assert_true(preview.initialize(4), "PreviewService initialize", preview.last_error());

        radar::core::EchoFrame frame;
        frame.pulse_index = 0;
        frame.scan_index = 0;
        frame.beam_az_deg = 90.0f;
        frame.iq = {radar::Complex(1.0f, 0.0f),
                    radar::Complex(2.0f, 0.0f),
                    radar::Complex(0.5f, 0.0f),
                    radar::Complex(0.25f, 0.0f)};
        preview.consume_frame(frame);
        assert_true(preview.has_range_profile(), "PreviewService range profile available");
        assert_true(preview.has_ppi_frame(), "PreviewService PPI frame available");
        const auto *profile = preview.latest_range_profile();
        assert_true(profile != nullptr && profile->power_db.size() == 4,
                    "PreviewService range profile size");
    }

    {
        radar::core::MetricsCollector metrics;
        metrics.reset(10, 1000.0f);
        metrics.record_pulse(0, 0, 10.0f, 20.0f, 5.0f, 2.0f, 3.0f, 1.0f, 1, 0);
        const auto snapshot = metrics.metrics();
        assert_true(snapshot.current_pulse == 0, "Metrics current pulse");
        assert_true(snapshot.frames_output == 1, "Metrics frames output");
        assert_true(snapshot.avg_us_per_prt > 0.0f, "Metrics avg_us_per_prt positive");
        assert_true(snapshot.realtime_ratio > 0.0f, "Metrics realtime ratio positive");
    }
}

// ============================================================================
// Counter-based RNG 测试
// ============================================================================
void test_counter_rng_layout()
{
    std::cout << "\n=== Counter RNG Layout Tests ===" << std::endl;

    constexpr std::uint64_t seed = 0x0123456789ABCDEFULL;
    constexpr std::uint32_t range_idx = 1234U;
    constexpr std::uint32_t az_idx = 567U;
    constexpr std::uint32_t channel_idx = 9U;
    constexpr std::uint32_t source_id = 3U;
    constexpr std::uint64_t sample_index = 0x0000000200000005ULL;

    const auto direct = radar::cuda::counter_random4(seed,
                                                     range_idx,
                                                     az_idx,
                                                     channel_idx,
                                                     sample_index,
                                                     source_id);
    const radar::cuda::Uint4 expected_ctr{
        static_cast<std::uint32_t>(((az_idx & 0xFFFFU) << 16U) | (range_idx & 0xFFFFU)),
        static_cast<std::uint32_t>(((source_id & 0xFFU) << 16U) | (channel_idx & 0xFFFFU)),
        static_cast<std::uint32_t>(sample_index),
        static_cast<std::uint32_t>(sample_index >> 32U),
    };
    const radar::cuda::Uint2 expected_key{
        static_cast<std::uint32_t>(seed),
        static_cast<std::uint32_t>(seed >> 32U),
    };
    const auto manual = radar::cuda::philox4x32_10(expected_ctr, expected_key);

    assert_true(direct.x == manual.x && direct.y == manual.y &&
                    direct.z == manual.z && direct.w == manual.w,
                "counter_random4 matches final field layout");

    const auto default_channel = radar::cuda::counter_random4(seed,
                                                             range_idx,
                                                             az_idx,
                                                             sample_index,
                                                             source_id);
    const auto explicit_zero_channel = radar::cuda::counter_random4(seed,
                                                                   range_idx,
                                                                   az_idx,
                                                                   0U,
                                                                   sample_index,
                                                                   source_id);
    assert_true(default_channel.x == explicit_zero_channel.x &&
                    default_channel.y == explicit_zero_channel.y &&
                    default_channel.z == explicit_zero_channel.z &&
                    default_channel.w == explicit_zero_channel.w,
                "counter_random4 default channel is zero");

    const auto different_channel = radar::cuda::counter_random4(seed,
                                                               range_idx,
                                                               az_idx,
                                                               channel_idx + 1U,
                                                               sample_index,
                                                               source_id);
    assert_true(direct.x != different_channel.x || direct.y != different_channel.y ||
                    direct.z != different_channel.z || direct.w != different_channel.w,
                "counter_random4 channel index changes stream");

    const auto different_source = radar::cuda::counter_random4(seed,
                                                              range_idx,
                                                              az_idx,
                                                              channel_idx,
                                                              sample_index,
                                                              source_id + 1U);
    assert_true(direct.x != different_source.x || direct.y != different_source.y ||
                    direct.z != different_source.z || direct.w != different_source.w,
                "counter_random4 source id changes stream");

    const std::uint64_t pair_index = 77ULL;
    const auto pair_rnd = radar::cuda::counter_random4(seed,
                                                       range_idx,
                                                       az_idx,
                                                       channel_idx,
                                                       pair_index,
                                                       source_id);
    const auto pair = radar::cuda::counter_complex_gaussian_sample_pair(seed,
                                                                        range_idx,
                                                                        az_idx,
                                                                        channel_idx,
                                                                        pair_index,
                                                                        source_id);
    const auto even = radar::cuda::complex_gaussian_from_uint2(pair_rnd.x, pair_rnd.y);
    const auto odd = radar::cuda::complex_gaussian_from_uint2(pair_rnd.z, pair_rnd.w);
    assert_near(pair.even.real, even.real, 0.0f, "counter pair even real lane");
    assert_near(pair.even.imag, even.imag, 0.0f, "counter pair even imag lane");
    assert_near(pair.odd.real, odd.real, 0.0f, "counter pair odd real lane");
    assert_near(pair.odd.imag, odd.imag, 0.0f, "counter pair odd imag lane");
}

// ============================================================================
// ClutterMap 测试
// ============================================================================
void test_clutter_map()
{
    std::cout << "\n=== ClutterMap Tests ===" << std::endl;

    // --- 基本初始化测试 ---
    {
        if (!require_clutter_gpu_runtime("SeaClutterEngine GPU initialization test"))
        {
            // Runtime-only block skipped.
        }
        else
        {
            clutter::SeaClutterConfig cfg;
            cfg.enabled = true;
            cfg.az_patch_step_deg = 0.25;
            cfg.active_gain_floor_db = -40.0;
            cfg.sea_state = 3.0;
            cfg.doppler_center_hz = 0.0;
            cfg.doppler_sigma_hz = 20.0;
            cfg.ground_range_min_m = 1000.0;
            cfg.ground_range_max_m = 10000.0;
            cfg.seed = 2026;

            RadarSystemParams sys;

            clutter::SeaClutterEngine engine;
            engine.set_config(cfg);
            bool init = engine.initialize(sys);
            assert_true(init, "SeaClutterEngine initialize");

            int num_cells = engine.num_az_cells();
            assert_true(num_cells == 1440, "SeaClutterEngine az cell count from fixed patch step",
                        "cells=" + std::to_string(num_cells));
        }
    }

    // --- 杂波配置验证 ---
    {
        clutter::SeaClutterConfig cfg;
        std::string error;

        cfg.enabled = true;
        cfg.az_patch_step_deg = 0.0;
        bool valid = cfg.validate(error);
        assert_true(!valid, "Reject zero az patch step");

        cfg.az_patch_step_deg = 0.25;
        cfg.active_gain_floor_db = 0.0;
        valid = cfg.validate(error);
        assert_true(!valid, "Reject non-negative active gain floor");

        cfg.active_gain_floor_db = -40.0;
        cfg.sea_state = -1.0;
        valid = cfg.validate(error);
        assert_true(!valid, "Reject negative sea state");

        cfg.sea_state = 3.0;
        cfg.weibull_shape = 0.0;
        valid = cfg.validate(error);
        assert_true(!valid, "Reject non-positive Weibull shape");

        cfg.weibull_shape = 2.0;
        cfg.weibull_scale = 0.0;
        valid = cfg.validate(error);
        assert_true(!valid, "Reject non-positive Weibull scale");

        cfg.weibull_scale = 1.41421356f;
        cfg.lognormal_sigma = -1.0;
        valid = cfg.validate(error);
        assert_true(!valid, "Reject negative lognormal sigma");

        cfg.lognormal_sigma = 1.0;
        cfg.k_shape_nu = 0.0;
        valid = cfg.validate(error);
        assert_true(!valid, "Reject non-positive K shape");

        cfg.k_shape_nu = 1.0;
        cfg.k_texture_bandwidth_hz = 0.0;
        valid = cfg.validate(error);
        assert_true(!valid, "Reject non-positive K texture bandwidth");

        cfg.k_texture_bandwidth_hz = 2.0;
        cfg.k_lut_size = 8;
        valid = cfg.validate(error);
        assert_true(!valid, "Reject too-small K LUT");

        cfg.k_lut_size = 4096;
        cfg.distribution = ClutterDistribution::LogNormal;
        error.clear();
        valid = cfg.validate(error);
        assert_true(valid, "Valid clutter config", error);

        cfg.distribution = ClutterDistribution::K;
        error.clear();
        valid = cfg.validate(error);
        assert_true(valid, "Valid K clutter config", error);
    }

    // --- 杂波引擎接口测试 ---
    {
        if (!require_clutter_gpu_runtime("SeaClutterEngine GPU echo test"))
        {
            return;
        }

        auto echo_power = [](const PulseEcho &echo)
        {
            Scalar power = 0.0f;
            for (const auto &sample : echo)
            {
                power += std::norm(sample);
            }
            return power;
        };

        auto echoes_identical = [](const PulseEcho &lhs, const PulseEcho &rhs)
        {
            if (lhs.size() != rhs.size())
            {
                return false;
            }
            for (std::size_t i = 0; i < lhs.size(); ++i)
            {
                if (lhs[i] != rhs[i])
                {
                    return false;
                }
            }
            return true;
        };

        clutter::SeaClutterConfig cfg;
        cfg.enabled = true;
        cfg.az_patch_step_deg = 1.0;
        cfg.active_gain_floor_db = -30.0;
        cfg.sea_state = 3.0;
        cfg.doppler_sigma_hz = 40.0;
        cfg.fir_length = 9;
        cfg.ground_range_min_m = 1000.0;
        cfg.ground_range_max_m = 2000.0;
        cfg.seed = 2026;

        RadarSystemParams sys;
        sys.max_range_m = 2000.0;
        sys.compute_derived_params();

        clutter::SeaClutterEngine engine1;
        engine1.set_config(cfg);
        bool init = engine1.initialize(sys);
        assert_true(init, "SeaClutterEngine initialize");

        PulseEcho echo0;
        bool got = engine1.get_clutter_for_pulse(BeamPoint{0.0, 0.0}, 0, echo0);
        assert_true(got, "SeaClutterEngine get_clutter_for_pulse",
                    "echo_size=" + std::to_string(echo0.size()));
        assert_true(echo0.size() == static_cast<std::size_t>(sys.samples_per_pulse),
                    "SeaClutterEngine echo size matches samples_per_pulse");
        assert_true(echo_power(echo0) > 0.0f, "SeaClutterEngine echo has nonzero clutter power");

        PulseEcho echo1;
        got = engine1.get_clutter_for_pulse(BeamPoint{0.0, 0.0}, 1, echo1);
        assert_true(got && !echoes_identical(echo0, echo1),
                    "SeaClutterEngine different PRT uses different counter samples");

        clutter::SeaClutterEngine engine2;
        engine2.set_config(cfg);
        init = engine2.initialize(sys);
        assert_true(init, "SeaClutterEngine second initialize");
        PulseEcho echo0_repeat;
        got = engine2.get_clutter_for_pulse(BeamPoint{0.0, 0.0}, 0, echo0_repeat);
        assert_true(got && echoes_identical(echo0, echo0_repeat),
                    "SeaClutterEngine same seed and PRT are deterministic");
    }
}

// ============================================================================
// DataExporter 测试
// ============================================================================
void test_data_exporter()
{
    std::cout << "\n=== DataExporter Tests ===" << std::endl;

    std::string test_output_dir = "out/test_data";
    std::filesystem::create_directories(test_output_dir);

    radar::core::ExportConfig cfg;
    cfg.enabled = true;
    cfg.output_dir = test_output_dir;
    cfg.export_raw_echo_iq = true;
    cfg.export_target_snapshots = true;
    cfg.export_config_params = true;

    radar::core::DataExporter exporter(cfg);
    bool init = exporter.initialize();
    assert_true(init, "DataExporter initialize");

    // --- 导出配置参数 ---
    {
        radar::RadarSystemParams sys;
        radar::target::TargetConfig target_cfg;
        radar::TargetList targets;

        radar::TargetState t;
        t.id = 1;
        t.position_m = radar::Vec3(50000, 0, 5000);
        t.rcs_mean_m2 = 10.0;
        t.enabled = true;
        targets.push_back(t);

        bool exported = exporter.export_config_params(sys, target_cfg, targets);
        assert_true(exported, "Export config params");

        std::string filepath = test_output_dir + "/config_params.json";
        bool file_exists = std::filesystem::exists(filepath);
        assert_true(file_exists, "Config JSON file created");
    }

    // --- 导出 pulse-native dat ---
    {
        radar::ScanEcho scan_echo;
        scan_echo.scan_index = 3;

        radar::PulseResult pulse0;
        pulse0.pulse_index = 0;
        pulse0.echo = radar::PulseEcho(4, radar::Complex(1.0f, 2.0f));
        scan_echo.pulse_results.push_back(pulse0);

        radar::PulseResult pulse1;
        pulse1.pulse_index = 1;
        pulse1.echo = radar::PulseEcho(4, radar::Complex(3.0f, 4.0f));
        scan_echo.pulse_results.push_back(pulse1);

        bool exported = exporter.export_echo_iq_dat(scan_echo);
        assert_true(exported, "Export pulse-native DAT", exporter.last_error());

        std::string filepath = test_output_dir + "/echo_iq_scan_3.dat";
        bool file_exists = std::filesystem::exists(filepath);
        assert_true(file_exists, "Pulse-native DAT file created");

        std::ifstream file(filepath, std::ios::binary);
        uint32_t magic = 0, version = 0, scan_index = 0, pulse_count = 0, samples_per_pulse = 0;
        uint64_t reserved = 0;
        file.read(reinterpret_cast<char *>(&magic), sizeof(magic));
        file.read(reinterpret_cast<char *>(&version), sizeof(version));
        file.read(reinterpret_cast<char *>(&scan_index), sizeof(scan_index));
        file.read(reinterpret_cast<char *>(&pulse_count), sizeof(pulse_count));
        file.read(reinterpret_cast<char *>(&samples_per_pulse), sizeof(samples_per_pulse));
        file.read(reinterpret_cast<char *>(&reserved), sizeof(reserved));

        assert_true(magic == 0x4543484F, "DAT magic matches");
        assert_true(version == 2, "DAT version matches pulse-native format");
        assert_true(scan_index == 3, "DAT scan index matches");
        assert_true(pulse_count == 2, "DAT pulse count matches");
        assert_true(samples_per_pulse == 4, "DAT sample count matches");
    }
}

// ============================================================================
// SimulationEngine 导出验证测试
// ============================================================================
void test_simulation_engine_export()
{
    std::cout << "\n=== SimulationEngine Export Tests ===" << std::endl;

    radar::RadarConfig cfg;
    cfg.simulation.scan_count = 1;
    cfg.system.min_range_m = 1000.0;
    cfg.system.max_range_m = 4000.0;
    cfg.system.prf_hz = 1600.0;
    cfg.system.fs_hz = 20e6;
    cfg.system.bw_hz = 10e6;
    cfg.system.pulse_width_s = 2e-6;
    cfg.system.peak_power_w = 1000.0;
    cfg.compute_derived_params();

    cfg.noise.enabled = false;
    cfg.clutter.enabled = false;
    cfg.target.enabled = true;
    cfg.target.enable_swerling = true;
    cfg.target.seed = 42;
    cfg.data_export.enabled = true;
    cfg.data_export.output_dir = "out/sim_export_test";
    cfg.data_export.export_raw_echo_iq = true;
    cfg.data_export.export_config_params = true;
    cfg.udp_output.enabled = false;

    radar::TargetState target;
    target.id = 1;
    target.position_m = radar::Vec3(2000.0, 0.0, 0.0);
    target.velocity_mps = radar::Vec3::Zero();
    target.acceleration_mps2 = radar::Vec3::Zero();
    target.motion_model = radar::MotionModel::Stationary;
    target.rcs_mean_m2 = 10.0;
    target.swerling = radar::SwerlingType::Swerling1;
    target.enabled = true;
    cfg.initial_targets = {target};

    std::filesystem::create_directories(cfg.data_export.output_dir);

    radar::SimulationEngine engine(cfg);
    bool init = engine.initialize();
    assert_true(init, "SimulationEngine initialize", engine.last_error());

    engine.run();

    std::string filepath = cfg.data_export.output_dir + "/echo_iq_scan_0.dat";
    assert_true(std::filesystem::exists(filepath), "SimulationEngine exported DAT file");

    std::ifstream file(filepath, std::ios::binary);
    uint32_t magic = 0, version = 0, scan_index = 0, pulse_count = 0, samples_per_pulse = 0;
    uint64_t reserved = 0;
    file.read(reinterpret_cast<char *>(&magic), sizeof(magic));
    file.read(reinterpret_cast<char *>(&version), sizeof(version));
    file.read(reinterpret_cast<char *>(&scan_index), sizeof(scan_index));
    file.read(reinterpret_cast<char *>(&pulse_count), sizeof(pulse_count));
    file.read(reinterpret_cast<char *>(&samples_per_pulse), sizeof(samples_per_pulse));
    file.read(reinterpret_cast<char *>(&reserved), sizeof(reserved));

    assert_true(magic == 0x4543484F, "SimulationEngine DAT magic matches");
    assert_true(version == 2, "SimulationEngine DAT version matches");
    assert_true(scan_index == 0, "SimulationEngine DAT scan index matches");
    assert_true(pulse_count > 0, "SimulationEngine DAT pulse count positive");
    assert_true(samples_per_pulse == static_cast<uint32_t>(cfg.system.samples_per_pulse),
                "SimulationEngine DAT samples_per_pulse matches");
}

// ============================================================================
// MATLAB 导出验证测试
// ============================================================================
void test_matlab_export()
{
    std::cout << "\n=== MATLAB Export Tests ===" << std::endl;

    std::string matlab_output_dir = "out/matlab_test";
    std::filesystem::create_directories(matlab_output_dir);

    radar::RadarSystemParams sys;
    radar::waveform::WaveformConfig cfg;

    // --- 导出 LFM 信号 ---
    {
        radar::waveform::WaveformGenerator gen;
        cfg.waveform_type = radar::WaveformType::LFM;
        gen.set_config(cfg);
        gen.set_system_params(sys);
        gen.initialize();

        const auto &waveform = gen.get_waveform();

        std::string filepath = matlab_output_dir + "/lfm_signal.csv";
        std::ofstream file(filepath);
        if (file.is_open())
        {
            file << "real,imag\n";
            for (const auto &s : waveform)
            {
                file << s.real() << "," << s.imag() << "\n";
            }
            file.close();
            assert_true(true, "LFM signal exported to CSV");
        }
        else
        {
            assert_true(false, "LFM signal export failed");
        }
    }

    // --- 导出 NLFM 信号 ---
    {
        radar::waveform::WaveformGenerator gen;
        cfg.waveform_type = radar::WaveformType::NLFM;
        cfg.nlfm_window_type = radar::WindowType::Hamming;
        gen.set_config(cfg);
        gen.set_system_params(sys);
        gen.initialize();

        const auto &waveform = gen.get_waveform();

        std::string filepath = matlab_output_dir + "/nlfm_signal.csv";
        std::ofstream file(filepath);
        if (file.is_open())
        {
            file << "real,imag\n";
            for (const auto &s : waveform)
            {
                file << s.real() << "," << s.imag() << "\n";
            }
            file.close();
            assert_true(true, "NLFM signal exported to CSV");
        }
        else
        {
            assert_true(false, "NLFM signal export failed");
        }
    }

    // --- 噪声样本导出 ---
    {
        radar::noise::NoiseEngine engine;
        radar::noise::NoiseConfig cfg;
        cfg.mode = radar::NoiseLevelMode::ComplexSigma;
        cfg.sigma_complex = 1e-3;
        cfg.seed = 12345;

        engine.set_config(cfg);
        engine.set_system_params(sys);
        engine.initialize();

        auto noise = engine.generate(10000);

        std::string filepath = matlab_output_dir + "/noise_samples.csv";
        std::ofstream file(filepath);
        if (file.is_open())
        {
            file << "real,imag\n";
            for (const auto &n : noise)
            {
                file << n.real() << "," << n.imag() << "\n";
            }
            file.close();
            assert_true(true, "Noise samples exported to CSV");
        }
        else
        {
            assert_true(false, "Noise export failed");
        }
    }
}

// ============================================================================
// 主测试函数
// ============================================================================
int main()
{
    std::cout << "╔══════════════════════════════════════════╗" << std::endl;
    std::cout << "║   Radar System Unit Tests (Mechanical)   ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════╝" << std::endl;
    std::cout << "Platform: " <<
#ifdef __linux__
        "Linux"
#elif defined(_WIN32)
        "Windows"
#elif defined(__APPLE__)
        "macOS"
#else
        "Unknown"
#endif
              << std::endl;

    test_math_utils();
    test_radar_system_params();
    test_radar_config();
    test_waveform_generator();
    test_noise_engine();
    test_antenna();
    test_target_engine();
    test_output_preview_metrics();
    test_counter_rng_layout();
    test_clutter_map();
    test_data_exporter();
    test_simulation_engine_export();
    test_matlab_export();

    std::cout << "\n╔══════════════════════════════════════════╗" << std::endl;
    std::cout << "║              Test Summary                ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════╝" << std::endl;
    std::cout << "  Passed: " << tests_passed << std::endl;
    std::cout << "  Failed: " << tests_failed << std::endl;
    std::cout << "  Total:  " << (tests_passed + tests_failed) << std::endl;

    if (tests_failed > 0)
    {
        std::cout << "\n[WARNING] Some tests failed. Check output above." << std::endl;
    }
    else
    {
        std::cout << "\n[SUCCESS] All tests passed!" << std::endl;
    }

    return tests_failed > 0 ? 1 : 0;
}
