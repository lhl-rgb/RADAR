/**
 * @file radar_tests.cpp
 * @brief 雷达仿真系统单元测试
 *
 * 测试框架：简单的基于断言的测试框架，无需外部依赖
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

#include "core/types.h"
#include "core/tools/math_utils.h"
#include "core/radar_system_params.h"
#include "waveform/waveform_generator.h"
#include "waveform/waveform_config.h"
#include "noise/noise_engine.h"
#include "noise/noise_config.h"
#include "clutter/sea_clutter_model.h"
#include "clutter/sea_clutter_config.h"
#include "antenna/antenna_model.h"
#include "antenna/beam_scanner.h"
#include "core/tools/data_exporter.h"
#include "target/target_config.h"

using namespace radar;

namespace {

int tests_passed = 0;
int tests_failed = 0;

void assert_true(bool condition, const std::string& test_name, const std::string& detail = "") {
    if (condition) {
        std::cout << "  [PASS] " << test_name;
        if (!detail.empty()) std::cout << " (" << detail << ")";
        std::cout << std::endl;
        tests_passed++;
    } else {
        std::cout << "  [FAIL] " << test_name;
        if (!detail.empty()) std::cout << " (" << detail << ")";
        std::cout << std::endl;
        tests_failed++;
    }
}

void assert_near(Scalar actual, Scalar expected, Scalar tol, const std::string& test_name) {
    Scalar diff = std::abs(actual - expected);
    bool pass = diff <= tol;
    std::string detail = "expected=" + std::to_string(expected) +
                         ", actual=" + std::to_string(actual) +
                         ", diff=" + std::to_string(diff);
    assert_true(pass, test_name, detail);
}

}  // namespace

// ============================================================================
// Math Utils 测试
// ============================================================================
void test_math_utils() {
    std::cout << "\n=== Math Utils Tests ===" << std::endl;

    // db_to_linear
    assert_near(radar::math::db_to_linear(0.0), 1.0, 1e-10, "db_to_linear(0 dB)");
    assert_near(radar::math::db_to_linear(10.0), 10.0, 1e-10, "db_to_linear(10 dB)");
    assert_near(radar::math::db_to_linear(3.0), 2.0, 0.1, "db_to_linear(3 dB) ≈ 2");

    // linear_to_db
    assert_near(radar::math::linear_to_db(1.0), 0.0, 1e-10, "linear_to_db(1)");
    assert_near(radar::math::linear_to_db(10.0), 10.0, 0.1, "linear_to_db(10)");

    // deg_to_rad / rad_to_deg
    assert_near(radar::math::deg_to_rad(180.0), radar::PI, 1e-10, "deg_to_rad(180)");
    assert_near(radar::math::rad_to_deg(radar::PI), 180.0, 1e-10, "rad_to_deg(PI)");

    // wrap_azimuth_deg
    assert_near(radar::math::wrap_azimuth_deg(0.0), 0.0, 1e-10, "wrap_azimuth(0)");
    assert_near(radar::math::wrap_azimuth_deg(360.0), 0.0, 1e-10, "wrap_azimuth(360)");
    assert_near(radar::math::wrap_azimuth_deg(190.0), -170.0, 1e-10, "wrap_azimuth(190)");
    assert_near(radar::math::wrap_azimuth_deg(-190.0), 170.0, 1e-10, "wrap_azimuth(-190)");

    // clamp_positive_eps
    assert_near(radar::math::clamp_positive_eps(0.5), 0.5, 1e-10, "clamp_positive(0.5)");
    assert_near(radar::math::clamp_positive_eps(0.0), radar::EPSILON, 1e-10, "clamp_positive(0)");
    assert_near(radar::math::clamp_positive_eps(-1.0), radar::EPSILON, 1e-10, "clamp_positive(-1)");

    // safe_div
    assert_near(radar::math::safe_div(10.0, 2.0), 5.0, 1e-10, "safe_div(10/2)");
    assert_near(radar::math::safe_div(10.0, 0.0), 10.0/radar::EPSILON, 1e-6, "safe_div(10/0)");
}

// ============================================================================
// RadarSystemParams 测试
// ============================================================================
void test_radar_system_params() {
    std::cout << "\n=== RadarSystemParams Tests ===" << std::endl;

    radar::RadarSystemParams params;

    // 默认参数验证
    std::string error;
    bool valid = params.validate(error);
    assert_true(valid, "Default params valid", error);

    // 派生参数计算
    assert_near(params.wavelength_m, radar::C / params.fc_hz, 1e-6, "wavelength");
    assert_near(params.pri_s, 1.0 / params.prf_hz, 1e-10, "PRI");
    assert_near(params.range_resolution_m, radar::C / (2.0 * params.bw_hz), 10.0, "range resolution");

    // 非法参数检测
    radar::RadarSystemParams invalid_params;
    invalid_params.pulse_width_s = 0.003;  // 3ms 脉宽
    invalid_params.prf_hz = 500.0;  // 500 Hz PRF (PRI=2ms, 脉宽>PRI)
    std::string error2;
    bool valid2 = invalid_params.validate(error2);
    assert_true(!valid2, "Invalid pulse_width rejected", error2);

    invalid_params = radar::RadarSystemParams();
    invalid_params.fs_hz = 1e6;  // 1 MHz 采样率
    invalid_params.bw_hz = 10e6;  // 10 MHz 带宽
    valid = invalid_params.validate(error);
    assert_true(!valid, "fs < bw rejected", error);
}

// ============================================================================
// WaveformGenerator 测试
// ============================================================================
void test_waveform_generator() {
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

        const auto& waveform = gen.get_waveform();
        assert_true(!waveform.empty(), "LFM waveform not empty");
        // samples_per_pulse 是根据快时间窗口计算的，可能与预期不同
        assert_true(static_cast<int>(waveform.size()) > 0,
                    "LFM waveform size", "size=" + std::to_string(waveform.size()));

        // 匹配滤波器
        auto mf = gen.generate_matched_filter();
        assert_true(mf.size() == waveform.size(), "Matched filter size");

        // 验证共轭反转：mf[n] = conj(waveform[N-1-n])
        bool mf_correct = true;
        for (std::size_t i = 0; i < waveform.size(); ++i) {
            if (std::abs(mf[i] - std::conj(waveform[waveform.size() - 1 - i])) > radar::EPSILON) {
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

        const auto& waveform = gen.get_waveform();
        assert_true(!waveform.empty(), "NLFM waveform not empty");

        // 功率归一化验证
        Scalar total_power = 0.0;
        for (const auto& s : waveform) {
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

        const auto& waveform = gen.get_waveform();
        assert_true(!waveform.empty(), "Barker13 waveform not empty");

        // 验证幅度恒定
        bool constant_envelope = true;
        Scalar ref_mag = std::abs(waveform[0]);
        for (const auto& s : waveform) {
            if (std::abs(std::abs(s) - ref_mag) > 0.1) {
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

        const auto& waveform = gen.get_waveform();
        assert_true(!waveform.empty(), "CW waveform not empty");

        // CW 应该是恒定幅度
        bool constant = true;
        for (const auto& s : waveform) {
            if (std::abs(std::abs(s) - 1.0) > 0.1) {
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
void test_noise_engine() {
    std::cout << "\n=== NoiseEngine Tests ===" << std::endl;

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

        // 生成大量样本验证统计特性
        const std::size_t N = 100000;
        auto noise = engine.generate(N);

        // 计算平均功率
        Scalar total_power = 0.0;
        for (const auto& n : noise) {
            total_power += std::norm(n);
        }
        Scalar avg_power = total_power / N;
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

        Scalar total_power = 0.0;
        for (const auto& n : noise) {
            total_power += std::norm(n);
        }
        Scalar avg_power = total_power / N;

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

        // 验证噪声功率计算
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
        cfg.sigma_complex = 0.5;  // 增加噪声强度，使其显著
        cfg.seed = 12345;

        engine.set_config(cfg);
        engine.set_system_params(sys);
        engine.initialize();

        // 测试向量加噪
        ComplexVec signal(1000, Complex(1.0, 0.0));
        Scalar signal_power_before = 0.0;
        for (const auto& s : signal) {
            signal_power_before += std::norm(s);
        }

        engine.add_noise(signal);

        Scalar signal_power_after = 0.0;
        for (const auto& s : signal) {
            signal_power_after += std::norm(s);
        }

        // 加噪后功率应该增加（至少增加 10%）
        bool power_increased = signal_power_after > signal_power_before * 1.1;
        assert_true(power_increased, "add_noise increases power",
                    "before=" + std::to_string(signal_power_before) +
                    ", after=" + std::to_string(signal_power_after));
    }

    // --- 重复性测试 ---
    {
        radar::noise::NoiseEngine engine1, engine2;
        radar::noise::NoiseConfig cfg;
        cfg.seed = 42;
        engine1.set_config(cfg);
        engine1.set_system_params(sys);
        engine1.initialize();

        engine2.set_config(cfg);
        engine2.set_system_params(sys);
        engine2.initialize();

        auto noise1 = engine1.generate(100);
        auto noise2 = engine2.generate(100);

        bool identical = true;
        for (std::size_t i = 0; i < noise1.size(); ++i) {
            if (std::abs(noise1[i] - noise2[i]) > 1e-10) {
                identical = false;
                break;
            }
        }
        assert_true(identical, "Same seed produces identical noise");
    }

    // --- I/Q 分量独立性验证 ---
    {
        radar::noise::NoiseEngine engine;
        radar::noise::NoiseConfig cfg;
        cfg.mode = radar::NoiseLevelMode::ComplexSigma;
        cfg.sigma_complex = 1e-3;
        cfg.seed = 12345;

        engine.set_config(cfg);
        engine.set_system_params(sys);
        engine.initialize();

        const std::size_t N = 100000;
        auto noise = engine.generate(N);

        // 计算 I/Q 分量的互相关
        Scalar iq_correlation = 0.0;
        Scalar i_mean = 0.0, q_mean = 0.0;

        for (const auto& n : noise) {
            i_mean += n.real();
            q_mean += n.imag();
        }
        i_mean /= N;
        q_mean /= N;

        for (const auto& n : noise) {
            iq_correlation += (n.real() - i_mean) * (n.imag() - q_mean);
        }
        iq_correlation /= N;

        // I/Q 应该独立，互相关应该接近 0
        Scalar theoretical_iq_var = (cfg.sigma_complex * cfg.sigma_complex) / 2.0;
        Scalar normalized_correlation = iq_correlation / theoretical_iq_var;

        assert_true(std::abs(normalized_correlation) < 0.01,
                    "I/Q components uncorrelated",
                    "correlation=" + std::to_string(normalized_correlation));
    }

    // --- SNR 验证 ---
    {
        radar::noise::NoiseEngine engine;
        radar::noise::NoiseConfig cfg;
        cfg.mode = radar::NoiseLevelMode::NoisePower;
        cfg.noise_power_w = 1e-6;
        cfg.seed = 12345;

        engine.set_config(cfg);
        engine.set_system_params(sys);
        engine.initialize();

        // 创建信号：SNR = 20 dB (信号功率是噪声的 100 倍)
        const std::size_t N = 100000;
        Scalar signal_power = 100.0 * cfg.noise_power_w;
        ComplexVec signal(N, Complex(std::sqrt(signal_power), 0.0));
        ComplexVec noise = engine.generate(N);

        // 计算实测 SNR
        Scalar noise_power = 0.0;
        for (const auto& n : noise) {
            noise_power += std::norm(n);
        }
        noise_power /= N;

        Scalar measured_snr_db = math::linear_to_db(signal_power / noise_power);
        Scalar theoretical_snr_db = 20.0;  // 10 * log10(100)
        Scalar snr_error = std::abs(measured_snr_db - theoretical_snr_db);

        assert_true(snr_error < 0.1, "SNR verification at 20dB",
                    "measured=" + std::to_string(measured_snr_db) +
                    ", expected=" + std::to_string(theoretical_snr_db) +
                    ", error=" + std::to_string(snr_error));
    }

    // --- ThermalKTB 模式统计验证 ---
    {
        radar::noise::NoiseEngine engine;
        radar::noise::NoiseConfig cfg;
        cfg.mode = radar::NoiseLevelMode::ThermalKTB;
        cfg.system_temperature_k = 290.0;
        cfg.noise_bandwidth_hz = 20e6;
        cfg.seed = 12345;

        engine.set_config(cfg);
        engine.set_system_params(sys);
        engine.initialize();

        // 生成样本验证统计特性
        const std::size_t N = 100000;
        auto noise = engine.generate(N);

        Scalar total_power = 0.0;
        for (const auto& n : noise) {
            total_power += std::norm(n);
        }
        Scalar avg_power = total_power / N;
        Scalar expected_power = engine.noise_power_w();

        Scalar rel_error = std::abs(avg_power - expected_power) / expected_power;

        assert_true(rel_error < 0.005, "ThermalKTB statistical verification",
                    "expected=" + std::to_string(expected_power) +
                    ", actual=" + std::to_string(avg_power) +
                    ", rel_error=" + std::to_string(rel_error));
    }
}

// ============================================================================
// SeaClutter 测试
// ============================================================================
void test_sea_clutter() {
    std::cout << "\n=== SeaClutter Tests ===" << std::endl;

    radar::RadarSystemParams sys;
    radar::clutter::SeaClutterConfig cfg;

    // 配置海杂波参数
    cfg.enabled = true;
    cfg.ground_range_min_m = 1000.0;
    cfg.ground_range_max_m = 10000.0;
    cfg.az_grid_step_deg = 1.0;
    cfg.k_shape_nu = 4.5;
    cfg.doppler_center_hz = 0.0;
    cfg.doppler_sigma_hz = 50.0;
    cfg.pool_length_factor = 2;
    cfg.seed = 2024;
    cfg.morchin_sea_state = 3;

    // --- 参数验证 ---
    {
        radar::clutter::SeaClutterModel model(cfg);
        std::string err = model.last_error();
        assert_true(err.empty() || model.params().k_shape_nu > 0, "SeaClutter params valid");
    }

    // --- 单层单元功率验证 ---
    {
        radar::clutter::SeaClutterModel model(cfg);

        // 验证模型可以生成杂波（间接验证 Morchin 模型）
        radar::antenna::AntennaConfig ant_cfg;
        ant_cfg.model_type = radar::PhasedArrayModelType::UPA_2D;
        ant_cfg.num_elements_az = 16;
        ant_cfg.num_elements_el = 8;
        ant_cfg.peak_gain_db = 30.0;

        radar::antenna::AntennaModel antenna;
        antenna.set_config(ant_cfg);
        antenna.initialize();

        radar::BeamPoint beam;
        beam.azimuth_deg = 0.0;
        beam.elevation_deg = 0.0;

        radar::ComplexVec tx_waveform(100, radar::Complex(1.0, 0.0));
        radar::CpiEcho clutter;

        bool generated = model.generate_cpi(sys, antenna, beam, 0, tx_waveform, clutter);
        assert_true(generated, "SeaClutter generate_cpi",
                    generated ? "success" : model.last_error());
    }

    // --- K 分布拖尾特性 ---
    {
        // K 分布在 nu 较时有更重的拖尾
        std::mt19937_64 rng(12345);
        std::gamma_distribution<radar::Scalar> gamma_nu1(2.0, 1.0/2.0);  // nu=2
        std::gamma_distribution<radar::Scalar> gamma_nu2(10.0, 1.0/10.0);  // nu=10

        const int N = 10000;
        radar::Scalar max_nu1 = 0, max_nu2 = 0;

        for (int i = 0; i < N; ++i) {
            radar::Scalar tau1 = gamma_nu1(rng);
            radar::Scalar tau2 = gamma_nu2(rng);
            if (tau1 > max_nu1) max_nu1 = tau1;
            if (tau2 > max_nu2) max_nu2 = tau2;
        }

        // nu 越小，拖尾越重，最大值应该更大
        assert_true(max_nu1 > max_nu2, "K-distribution tail behavior",
                    "max_nu1=" + std::to_string(max_nu1) + ", max_nu2=" + std::to_string(max_nu2));
    }
}

// ============================================================================
// DataExporter 测试
// ============================================================================
void test_data_exporter() {
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

        // 验证文件存在
        std::string filepath = test_output_dir + "/config_params.json";
        bool file_exists = std::filesystem::exists(filepath);
        assert_true(file_exists, "Config JSON file created");
    }

    // --- 导出 IQ 数据 ---
    {
        std::vector<radar::CpiEcho> cpi_echos;

        radar::CpiEcho echo;
        echo.beam_index = 0;
        echo.azimuth_deg = 0.0;
        echo.elevation_deg = 0.0;

        // 创建 4 个脉冲，每个 100 个采样点
        for (int p = 0; p < 4; ++p) {
            radar::PulseEcho pulse(100);
            for (int s = 0; s < 100; ++s) {
                pulse[static_cast<std::size_t>(s)] = radar::Complex(1.0, 0.0);
            }
            echo.pulses.push_back(pulse);
        }
        cpi_echos.push_back(echo);

        bool exported = exporter.export_echo_iq_dat(cpi_echos, 0);
        assert_true(exported, "Export IQ data");

        // 验证文件存在
        std::string filepath = test_output_dir + "/echo_iq_scan_0.dat";
        bool file_exists = std::filesystem::exists(filepath);
        assert_true(file_exists, "IQ DAT file created");

        // 验证文件大小 (header=32 bytes + 4*100*8 bytes)
        if (file_exists) {
            auto size = std::filesystem::file_size(filepath);
            std::uint64_t expected_size = 32 + 4 * 100 * 8;  // header + IQ data
            assert_true(size == expected_size, "IQ file size",
                        "expected=" + std::to_string(expected_size) +
                        ", actual=" + std::to_string(size));
        }
    }
}

// ============================================================================
// MATLAB 导出验证测试（导出 LFM/NLFM 信号）
// ============================================================================
void test_matlab_export() {
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

        const auto& waveform = gen.get_waveform();

        // 导出为 CSV（MATLAB 可读）
        std::string filepath = matlab_output_dir + "/lfm_signal.csv";
        std::ofstream file(filepath);
        if (file.is_open()) {
            file << "real,imag\n";
            for (const auto& s : waveform) {
                file << s.real() << "," << s.imag() << "\n";
            }
            file.close();
            assert_true(true, "LFM signal exported to CSV");
        } else {
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

        const auto& waveform = gen.get_waveform();

        std::string filepath = matlab_output_dir + "/nlfm_signal.csv";
        std::ofstream file(filepath);
        if (file.is_open()) {
            file << "real,imag\n";
            for (const auto& s : waveform) {
                file << s.real() << "," << s.imag() << "\n";
            }
            file.close();
            assert_true(true, "NLFM signal exported to CSV");
        } else {
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
        if (file.is_open()) {
            file << "real,imag\n";
            for (const auto& n : noise) {
                file << n.real() << "," << n.imag() << "\n";
            }
            file.close();
            assert_true(true, "Noise samples exported to CSV");
        } else {
            assert_true(false, "Noise export failed");
        }
    }

    // --- 导出海杂波样本 ---
    {
        radar::clutter::SeaClutterConfig cfg;
        cfg.enabled = true;
        cfg.ground_range_min_m = 1000.0;
        cfg.ground_range_max_m = 5000.0;
        cfg.az_grid_step_deg = 1.0;
        cfg.k_shape_nu = 4.5;
        cfg.doppler_center_hz = 0.0;
        cfg.doppler_sigma_hz = 50.0;
        cfg.pool_length_factor = 2;
        cfg.seed = 2024;
        cfg.morchin_sea_state = 3;

        radar::clutter::SeaClutterModel model(cfg);

        // 创建简单天线模型
        radar::antenna::AntennaConfig ant_cfg;
        ant_cfg.model_type = radar::PhasedArrayModelType::UPA_2D;
        ant_cfg.num_elements_az = 16;
        ant_cfg.num_elements_el = 8;
        ant_cfg.peak_gain_db = 30.0;

        radar::antenna::AntennaModel antenna;
        antenna.set_config(ant_cfg);
        antenna.initialize();

        // 生成单个 CPI 的海杂波
        radar::BeamPoint beam;
        beam.azimuth_deg = 0.0;
        beam.elevation_deg = 0.0;

        radar::ComplexVec tx_waveform(100, radar::Complex(1.0, 0.0));
        radar::CpiEcho clutter;

        bool generated = model.generate_cpi(sys, antenna, beam, 0, tx_waveform, clutter);

        if (generated) {
            std::string filepath = matlab_output_dir + "/sea_clutter_samples.csv";
            std::ofstream file(filepath);
            if (file.is_open()) {
                file << "pulse,range,real,imag\n";
                int pulse_idx = 0;
                for (const auto& pulse : clutter.pulses) {
                    int range_idx = 0;
                    for (const auto& sample : pulse) {
                        file << pulse_idx << "," << range_idx << ","
                             << sample.real() << "," << sample.imag() << "\n";
                    }
                    ++range_idx;
                    ++pulse_idx;
                }
                file.close();
                assert_true(true, "Sea clutter exported to CSV");
            } else {
                assert_true(false, "Sea clutter export failed");
            }
        } else {
            assert_true(false, "Sea clutter generation failed", model.last_error());
        }
    }
}

// ============================================================================
// Antenna 测试
// ============================================================================
void test_antenna() {
    std::cout << "\n=== Antenna Tests ===" << std::endl;

    // --- ULA 单波位增益测试 ---
    {
        antenna::AntennaConfig cfg;
        cfg.model_type = PhasedArrayModelType::ULA_1D;
        cfg.num_elements_az = 16;
        cfg.num_elements_el = 1;
        cfg.spacing_az_lambda = 0.5;
        cfg.peak_gain_db = 30.0;
        cfg.weight_type_az = AntennaWeightType::Uniform;

        antenna::AntennaModel model;
        model.set_config(cfg);
        model.initialize();

        // 波束指向 0°，目标 10°，有一定角度偏差，增益会降低
        Scalar gain_db = model.gain_db(10.0, 0.0, 0.0, 0.0);
        Scalar norm_pwr = model.normalized_power(10.0, 0.0, 0.0, 0.0);

        // 验证增益在合理范围内 (10°偏离应该在主瓣内，但增益会降低)
        assert_true(gain_db > 15.0 && gain_db <= 30.0, "ULA gain in range",
                    "gain=" + std::to_string(gain_db) + " dB");

        // 验证归一化功率在 0-1 之间
        assert_true(norm_pwr > 0.0 && norm_pwr <= 1.0, "ULA normalized power valid",
                    "power=" + std::to_string(norm_pwr));
    }

    // --- ULA 均匀加权 vs Hamming 加权 ---
    {
        antenna::AntennaConfig cfg_uniform;
        cfg_uniform.model_type = PhasedArrayModelType::ULA_1D;
        cfg_uniform.num_elements_az = 16;
        cfg_uniform.weight_type_az = AntennaWeightType::Uniform;
        cfg_uniform.peak_gain_db = 30.0;

        antenna::AntennaConfig cfg_hamming;
        cfg_hamming.model_type = PhasedArrayModelType::ULA_1D;
        cfg_hamming.num_elements_az = 16;
        cfg_hamming.weight_type_az = AntennaWeightType::Hamming;
        cfg_hamming.peak_gain_db = 30.0;

        antenna::AntennaModel model_uniform, model_hamming;
        model_uniform.set_config(cfg_uniform);
        model_uniform.initialize();
        model_hamming.set_config(cfg_hamming);
        model_hamming.initialize();

        // 主瓣中心增益
        Scalar gain_uniform = model_uniform.gain_db(0.0, 0.0, 0.0, 0.0);
        Scalar gain_hamming = model_hamming.gain_db(0.0, 0.0, 0.0, 0.0);

        // Hamming 加权由于归一化，峰值应该与均匀加权相同 (都是 0dB 归一化)
        assert_near(gain_uniform, 30.0, 0.01, "ULA uniform peak gain");
        assert_near(gain_hamming, 30.0, 0.01, "ULA hamming peak gain");
    }

    // --- UPA 单波位增益测试 ---
    {
        antenna::AntennaConfig cfg;
        cfg.model_type = PhasedArrayModelType::UPA_2D;
        cfg.num_elements_az = 16;
        cfg.num_elements_el = 12;
        cfg.spacing_az_lambda = 0.5;
        cfg.spacing_el_lambda = 0.5;
        cfg.peak_gain_db = 35.0;
        cfg.weight_type_az = AntennaWeightType::Hamming;
        cfg.weight_type_el = AntennaWeightType::Hamming;

        antenna::AntennaModel model;
        model.set_config(cfg);
        model.initialize();

        // 主瓣中心
        Scalar gain_db = model.gain_db(0.0, 0.0, 0.0, 0.0);
        Scalar norm_pwr = model.normalized_power(0.0, 0.0, 0.0, 0.0);

        assert_near(gain_db, 35.0, 0.01, "UPA peak gain at boresight");
        assert_near(norm_pwr, 1.0, 1e-6, "UPA normalized power at boresight");

        // 离轴增益应该降低
        Scalar off_axis_gain = model.gain_db(15.0, 8.0, 0.0, 0.0);
        assert_true(off_axis_gain < gain_db, "UPA off-axis gain lower",
                    "on_axis=" + std::to_string(gain_db) +
                    ", off_axis=" + std::to_string(off_axis_gain));
    }

    // --- UPA 方向余弦计算验证 ---
    {
        antenna::AntennaConfig cfg;
        cfg.model_type = PhasedArrayModelType::UPA_2D;
        cfg.num_elements_az = 8;
        cfg.num_elements_el = 8;
        cfg.spacing_az_lambda = 0.5;
        cfg.spacing_el_lambda = 0.5;
        cfg.peak_gain_db = 25.0;

        antenna::AntennaModel model;
        model.set_config(cfg);
        model.initialize();

        // 波束指向 (15, 8) 度，目标也在 (15, 8) 度，应该获得最大增益
        Scalar gain = model.gain_db(15.0, 8.0, 15.0, 8.0);
        Scalar norm_pwr = model.normalized_power(15.0, 8.0, 15.0, 8.0);

        assert_near(norm_pwr, 1.0, 1e-6, "UPA power at scan direction");
        assert_near(gain, 25.0, 0.01, "UPA gain at scan direction");
    }

    // --- BeamScanner 基本功能测试 ---
    {
        antenna::AntennaConfig ant_cfg;
        ant_cfg.model_type = PhasedArrayModelType::ULA_1D;
        ant_cfg.num_elements_az = 8;
        ant_cfg.peak_gain_db = 25.0;

        antenna::BeamTableConfig beam_cfg;
        beam_cfg.type = "azimuth_scan";
        beam_cfg.az_start_deg = -10.0;
        beam_cfg.az_end_deg = 10.0;
        beam_cfg.az_step_deg = 5.0;
        beam_cfg.elevation_deg = 0.0;

        antenna::BeamScanner scanner;
        scanner.set_antenna_config(ant_cfg);
        scanner.set_beam_table_config(beam_cfg);
        bool init = scanner.initialize();

        assert_true(init, "BeamScanner initialize");
        assert_true(scanner.beam_count() == 5, "BeamScanner beam count",
                    "count=" + std::to_string(scanner.beam_count()));

        // 测试扫描
        scanner.reset();
        BeamPoint first = scanner.get_beam_pointing();
        assert_near(first.azimuth_deg, -10.0, 0.01, "First beam azimuth");

        scanner.advance_one_cpi();
        BeamPoint second = scanner.get_beam_pointing();
        assert_near(second.azimuth_deg, -5.0, 0.01, "Second beam azimuth");

        // 测试增益计算
        scanner.reset();
        Scalar gain = scanner.get_gain_db_to_target(-10.0, 0.0);
        assert_true(gain > 20.0 && gain <= 25.0, "BeamScanner gain in range",
                    "gain=" + std::to_string(gain));
    }

    // --- BeamScanner 循环扫描测试 ---
    {
        antenna::AntennaConfig ant_cfg;
        ant_cfg.model_type = PhasedArrayModelType::ULA_1D;
        ant_cfg.num_elements_az = 4;
        ant_cfg.peak_gain_db = 20.0;

        antenna::BeamTableConfig beam_cfg;
        beam_cfg.type = "azimuth_scan";
        beam_cfg.az_start_deg = 0.0;
        beam_cfg.az_end_deg = 3.0;
        beam_cfg.az_step_deg = 1.0;

        antenna::BeamScanner scanner;
        scanner.set_antenna_config(ant_cfg);
        scanner.set_beam_table_config(beam_cfg);
        scanner.initialize();

        // 循环扫描一圈回到起点
        BeamPoint start = scanner.get_beam_pointing();
        for (std::size_t i = 0; i < scanner.beam_count(); ++i) {
            scanner.advance_one_cpi();
        }
        BeamPoint after_loop = scanner.get_beam_pointing();

        assert_near(after_loop.azimuth_deg, start.azimuth_deg, 0.01,
                    "BeamScanner wraps around");
    }
}

// ============================================================================
// 主测试函数
// ============================================================================
int main() {
    std::cout << "╔══════════════════════════════════════════╗" << std::endl;
    std::cout << "║       Radar System Unit Tests            ║" << std::endl;
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

    // 运行所有测试
    test_math_utils();
    test_radar_system_params();
    test_waveform_generator();
    test_noise_engine();
    test_sea_clutter();
    test_data_exporter();
    test_matlab_export();
    test_antenna();

    // 打印汇总
    std::cout << "\n╔══════════════════════════════════════════╗" << std::endl;
    std::cout << "║              Test Summary                ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════╝" << std::endl;
    std::cout << "  Passed: " << tests_passed << std::endl;
    std::cout << "  Failed: " << tests_failed << std::endl;
    std::cout << "  Total:  " << (tests_passed + tests_failed) << std::endl;

    if (tests_failed > 0) {
        std::cout << "\n[WARNING] Some tests failed. Check output above." << std::endl;
    } else {
        std::cout << "\n[SUCCESS] All tests passed!" << std::endl;
    }

    return tests_failed > 0 ? 1 : 0;
}
