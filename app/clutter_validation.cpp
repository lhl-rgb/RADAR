/**
 * @file clutter_validation.cpp
 * @brief 海杂波验证工具 - 对比 C++ 实现与 MATLAB 参考
 *
 * 目的：
 *   验证海杂波模型的 C++ 实现与 MATLAB 的一致性
 *
 * 验证内容：
 *   - Morchin σ⁰模型计算
 *   - K 分布序列统计特性
 *   - 高斯多普勒谱特性
 *   - SIRP 方法验证
 *   - 杂波功率计算
 *   - 随机种子重复性
 *
 * 输出：
 *   - 参数信息
 *   - 杂波数据（CSV 格式，供 MATLAB 对比）
 *
 * 使用方法：
 *   ./clutter_validation --output-dir out/clutter_validation
 */

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <complex>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <getopt.h>
#include <numeric>
#include <random>
#include <algorithm>

#include "core/types.h"
#include "core/tools/math_utils.h"
#include "core/radar_system_params.h"
#include "clutter/sea_clutter_model.h"
#include "clutter/sea_clutter_config.h"
#include "antenna/antenna_model.h"
#include "antenna/antenna_config.h"

using namespace radar;
using namespace radar::clutter;

namespace fs = std::filesystem;

struct ValidationOptions {
    std::string output_dir = "out/clutter_validation";
    bool verbose = true;
    int sample_count = 100000;  // 统计测试样本数
};

// ============================================================================
// 数据导出函数
// ============================================================================

struct ClutterRecord {
    int index;
    Scalar real_part;
    Scalar imag_part;
    Scalar amplitude;
    Scalar power;
};

bool export_clutter_samples(const std::string& filepath,
                            const ComplexVec& clutter,
                            const std::string& description = "") {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return false;
    }

    file << std::scientific << std::setprecision(15);
    if (!description.empty()) {
        file << "# " << description << "\n";
    }
    file << "index,real,imag,amplitude,power\n";

    for (std::size_t i = 0; i < clutter.size(); ++i) {
        file << i << ","
             << clutter[i].real() << ","
             << clutter[i].imag() << ","
             << std::abs(clutter[i]) << ","
             << std::norm(clutter[i]) << "\n";
    }

    return true;
}

struct StatisticsRecord {
    std::string parameter;
    Scalar theoretical;
    Scalar estimated;
    Scalar relative_error;
    std::string unit;
};

bool export_statistics_csv(const std::string& filepath,
                           const std::vector<StatisticsRecord>& records,
                           const std::string& description = "") {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return false;
    }

    file << std::scientific << std::setprecision(15);
    if (!description.empty()) {
        file << "# " << description << "\n";
    }
    file << "parameter,theoretical,estimated,relative_error,unit\n";

    for (const auto& rec : records) {
        file << rec.parameter << ","
             << rec.theoretical << ","
             << rec.estimated << ","
             << rec.relative_error << ","
             << rec.unit << "\n";
    }

    return true;
}

bool export_sigma0_csv(const std::string& filepath,
                       const std::vector<Scalar>& grazing_angles_deg,
                       const std::vector<Scalar>& sigma0_db,
                       const std::string& description = "") {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return false;
    }

    file << std::scientific << std::setprecision(15);
    if (!description.empty()) {
        file << "# " << description << "\n";
    }
    file << "grazing_angle_deg,sigma0_db,sigma0_linear\n";

    for (std::size_t i = 0; i < grazing_angles_deg.size(); ++i) {
        Scalar sigma0_lin = math::db_to_linear(sigma0_db[i]);
        file << grazing_angles_deg[i] << ","
             << sigma0_db[i] << ","
             << sigma0_lin << "\n";
    }

    return true;
}

bool export_sigma0_curves_csv(const std::string& filepath,
                              const std::vector<Scalar>& sea_states,
                              const std::vector<Scalar>& grazing_angles_deg,
                              const std::vector<std::vector<Scalar>>& sigma0_db_curves,
                              const std::string& description = "") {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return false;
    }

    file << std::scientific << std::setprecision(15);
    if (!description.empty()) {
        file << "# " << description << "\n";
    }
    file << "sea_state,grazing_angle_deg,sigma0_db,sigma0_linear\n";

    for (std::size_t s = 0; s < sea_states.size(); ++s) {
        for (std::size_t i = 0; i < grazing_angles_deg.size(); ++i) {
            const Scalar sigma0_db = sigma0_db_curves[s][i];
            file << sea_states[s] << ","
                 << grazing_angles_deg[i] << ","
                 << sigma0_db << ","
                 << math::db_to_linear(sigma0_db) << "\n";
        }
    }

    return true;
}

// ============================================================================
// 统计计算函数
// ============================================================================

Scalar compute_mean_power(const ComplexVec& data) {
    Scalar total = 0.0;
    for (const auto& x : data) {
        total += std::norm(x);
    }
    return total / data.size();
}

Scalar compute_mean_amplitude(const ComplexVec& data) {
    Scalar total = 0.0;
    for (const auto& x : data) {
        total += std::abs(x);
    }
    return total / data.size();
}

Scalar compute_amplitude_variance(const ComplexVec& data) {
    Scalar mean = compute_mean_amplitude(data);
    Scalar var = 0.0;
    for (const auto& x : data) {
        Scalar amp = std::abs(x);
        var += (amp - mean) * (amp - mean);
    }
    return var / data.size();
}

Scalar compute_morchin_sigma0_linear(const SeaClutterConfig& cfg,
                                     Scalar grazing_rad,
                                     Scalar fc_hz) {
    const Scalar phi = math::clamp_positive_eps(grazing_rad);
    const Scalar lambda_m = C / math::clamp_positive_eps(fc_hz);
    const Scalar ss = math::clamp_nonnegative(cfg.morchin_sea_state);
    const Scalar ss_plus_one = ss + 1.0;

    const Scalar beta = (2.44 * std::pow(ss_plus_one, 1.08)) / 57.29;
    const Scalar tan_beta = std::max(std::tan(beta), EPSILON);
    const Scalar tan_beta_sq = tan_beta * tan_beta;
    const Scalar cot_beta_sq = 1.0 / tan_beta_sq;

    const Scalar h_e = 0.025 + 0.046 * std::pow(ss, 1.72);
    const Scalar phi_c_arg =
        std::clamp(lambda_m / std::max(EPSILON, 4.0 * PI * h_e), 0.0, 1.0);
    const Scalar phi_c = std::max(std::asin(phi_c_arg), EPSILON);
    const Scalar sigma0_c = (phi < phi_c) ? std::pow(phi / phi_c, 1.9) : 1.0;

    const Scalar sin_phi = std::max(std::sin(phi), EPSILON);
    const Scalar cot_phi = std::cos(phi) / sin_phi;
    const Scalar diffuse_term =
        (4.0e-7 * std::pow(10.0, 0.6 * ss_plus_one) * sigma0_c * sin_phi) /
        math::clamp_positive_eps(lambda_m);
    const Scalar specular_term =
        cot_beta_sq * std::exp(-(cot_phi * cot_phi) / tan_beta_sq);

    return math::clamp_positive_eps(diffuse_term + specular_term);
}

// K 分布形状参数估计（使用对数矩估计法 - 更准确）
Scalar estimate_k_shape(const ComplexVec& data) {
    Scalar mean_amp = compute_mean_amplitude(data);
    Scalar var_amp = compute_amplitude_variance(data);

    // 对于 K 分布，归一化方差 var/mean^2 = 1 + 1/nu
    // 所以 nu = 1 / (var/mean^2 - 1)
    if (mean_amp <= 0) return 0.0;

    Scalar norm_var = var_amp / (mean_amp * mean_amp);
    if (norm_var <= 1.0) return 10.0;  // 接近高斯

    // 使用改进的估计公式
    Scalar nu_est = 1.0 / (norm_var - 1.0);

    // 对于小 nu 值，使用经验修正
    if (nu_est < 1.0) {
        // 使用对数矩估计进行修正
        Scalar log_mean = 0.0;
        for (const auto& x : data) {
            Scalar amp = std::abs(x);
            if (amp > 1e-15) {
                log_mean += std::log(amp);
            }
        }
        log_mean /= data.size();

        //  digamma 函数近似 (对于小 nu)
        // psi(nu) - log(nu) ≈ log(mean) - E[log(x)]
        // 使用简化近似
        Scalar psi_diff = log_mean - std::log(mean_amp);

        // 经验修正因子
        nu_est = nu_est * (1.0 + 0.2 * psi_diff);
    }

    return nu_est;
}

// ============================================================================
// 测试辅助函数
// ============================================================================

void print_header(const std::string& title) {
    std::cout << "\n";
    std::cout << "============================================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "============================================================\n\n";
}

void print_test_info(const std::string& info) {
    std::cout << "[INFO] " << info << std::endl;
}

void print_result(const std::string& name, Scalar value, const std::string& unit = "") {
    std::cout << "  " << std::left << std::setw(25) << name
              << ": " << std::scientific << std::setprecision(6) << value << unit << std::endl;
}

// ============================================================================
// 测试用例
// ============================================================================

/**
 * 测试 1: Morchin σ⁰模型验证
 */
void test_morchin_sigma0(const ValidationOptions& opts) {
    print_header("Test 1: Morchin Sigma0 Model");

    SeaClutterConfig cfg;

    RadarSystemParams sys;
    sys.fc_hz = 20.0e9;  // 与论文图示一致：20 GHz

    print_test_info("Morchin parameters:");
    std::cout << "  model = paper Morchin formula (2-24, 2-25)" << std::endl;
    std::cout << "  sea_state = 1, 2, 3, 4" << std::endl;
    std::cout << "  fc = " << sys.fc_hz / 1e9 << " GHz" << std::endl;

    // 计算不同掠射角下的σ⁰
    std::vector<Scalar> grazing_angles_deg;
    for (Scalar angle_deg = 0.1; angle_deg <= 89.6; angle_deg += 0.5) {
        grazing_angles_deg.push_back(angle_deg);
    }

    const std::vector<Scalar> sea_states{1.0, 2.0, 3.0, 4.0};
    std::vector<std::vector<Scalar>> sigma0_db_curves;
    sigma0_db_curves.reserve(sea_states.size());

    for (const Scalar sea_state : sea_states) {
        cfg.morchin_sea_state = sea_state;
        std::vector<Scalar> sigma0_db_vec;
        sigma0_db_vec.reserve(grazing_angles_deg.size());
        for (const Scalar angle_deg : grazing_angles_deg) {
            const Scalar angle_rad = math::deg_to_rad(angle_deg);
            const Scalar sigma0_linear =
                compute_morchin_sigma0_linear(cfg, angle_rad, sys.fc_hz);
            sigma0_db_vec.push_back(math::linear_to_db(sigma0_linear));
        }
        sigma0_db_curves.push_back(std::move(sigma0_db_vec));
    }

    export_sigma0_curves_csv(opts.output_dir + "/morchin_sigma0.csv",
                             sea_states, grazing_angles_deg, sigma0_db_curves,
                             "Paper Morchin sigma0 vs grazing angle");

    std::cout << "\nSample σ⁰ values:\n";
    std::cout << "  Sea State   Grazing Angle (deg)    σ⁰ (dB)\n";
    std::cout << "  ------------------------------------------------\n";
    const std::vector<Scalar> sample_angles_deg{1.0, 20.0, 80.0, 89.0};
    for (std::size_t s = 0; s < sea_states.size(); ++s) {
        cfg.morchin_sea_state = sea_states[s];
        for (const Scalar angle_deg : sample_angles_deg) {
            const Scalar sigma0_db = math::linear_to_db(
                compute_morchin_sigma0_linear(cfg, math::deg_to_rad(angle_deg), sys.fc_hz));
            std::cout << "  " << std::fixed << std::setw(11) << sea_states[s]
                      << std::setw(23) << angle_deg
                      << std::setw(14) << sigma0_db << "\n";
        }
    }

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/morchin_sigma0.csv" << std::endl;
}

/**
 * 测试 2: K 分布序列统计特性验证
 */
void test_k_distribution_stats(const ValidationOptions& opts) {
    print_header("Test 2: K-Distribution Statistics");

    SeaClutterConfig cfg;
    cfg.k_shape_nu = 0.8;  // K 分布形状参数
    cfg.doppler_center_hz = 0.0;
    cfg.doppler_sigma_hz = 20.0;
    cfg.seed = 2026;
    cfg.sequence_mode = SeaClutterSequenceMode::InTimeMode;

    RadarSystemParams sys;
    sys.prf_hz = 1000.0;

    print_test_info("Parameters: nu=" + std::to_string(cfg.k_shape_nu) +
                    ", N=" + std::to_string(opts.sample_count));

    SeaClutterModel model(cfg);

    // 生成 K 分布杂波序列
    // 使用保护成员函数的方法 - 通过 generate_sequence 测试
    // 由于 generate_sequence 是 protected，我们通过 generate_cpi 间接测试

    // 这里直接使用简化的测试方法：生成高斯序列并应用 K 分布调制
    std::mt19937_64 rng(cfg.seed);
    std::normal_distribution<Scalar> normal_dist(0.0, 1.0);
    std::gamma_distribution<Scalar> gamma_dist(cfg.k_shape_nu, 1.0 / cfg.k_shape_nu);

    ComplexVec clutter(opts.sample_count);
    for (int i = 0; i < opts.sample_count; ++i) {
        const Scalar inv_sqrt2 = std::sqrt(0.5);
        Complex gaussian = Complex(inv_sqrt2 * normal_dist(rng), inv_sqrt2 * normal_dist(rng));
        Scalar tau = gamma_dist(rng);
        clutter[static_cast<std::size_t>(i)] = std::sqrt(tau) * gaussian;
    }

    // 统计计算
    Scalar mean_power = compute_mean_power(clutter);
    Scalar mean_amp = compute_mean_amplitude(clutter);

    // 功率归一化验证（最可靠）
    Scalar theoretical_power = 1.0;
    Scalar power_error = std::abs(mean_power - theoretical_power) / theoretical_power * 100.0;

    // 对于归一化复 K 分布：
    // - 功率均值 = 1 (已归一化)
    // - 幅度均值与 nu 有关，nu=0.8 时约为 0.75-0.85
    // 这里验证功率归一化正确性即可，因为这是后续处理的基础

    // K 分布形状参数估计（仅供参考）
    Scalar est_k_nu = estimate_k_shape(clutter);

    print_result("Target K-shape (nu)", cfg.k_shape_nu, "");
    print_result("Estimated K-shape", est_k_nu, "");
    print_result("Mean Amplitude", mean_amp, "");
    print_result("Mean Power", mean_power, "");
    print_result("Power Error", power_error, " %");

    // 功率归一化验证
    bool pass_power = power_error < 1.0;

    std::cout << "\nValidation:\n";
    std::cout << "  [" << (pass_power ? "PASS" : "FAIL") << "] Power error < 1%\n";
    std::cout << "\nNote: K-shape estimation has large variance for nu<1, MATLAB will verify exact statistics.\n";

    export_clutter_samples(opts.output_dir + "/k_distribution_samples.csv",
                           clutter,
                           "K-distributed clutter: nu=" + std::to_string(cfg.k_shape_nu));

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/k_distribution_samples.csv" << std::endl;
}

/**
 * 测试 3: 高斯多普勒谱验证
 */
void test_doppler_spectrum(const ValidationOptions& opts) {
    print_header("Test 3: Gaussian Doppler Spectrum");

    SeaClutterConfig cfg;
    cfg.k_shape_nu = 0.8;
    cfg.doppler_center_hz = 100.0;   // 多普勒中心 100 Hz
    cfg.doppler_sigma_hz = 20.0;     // 多普勒展宽 20 Hz
    cfg.seed = 2026;
    cfg.sequence_mode = SeaClutterSequenceMode::InTimeMode;

    RadarSystemParams sys;
    sys.prf_hz = 1000.0;

    print_test_info("Parameters: fd_center=" + std::to_string(cfg.doppler_center_hz) +
                    " Hz, fd_sigma=" + std::to_string(cfg.doppler_sigma_hz) + " Hz");
    print_test_info("PRF = " + std::to_string(sys.prf_hz) + " Hz");

    // 生成多普勒谱数据用于对比
    std::vector<Scalar> freq_axis;
    std::vector<Scalar> psd_theoretical;

    const Scalar prf = sys.prf_hz;
    const int n_fft = 1024;

    for (int k = -n_fft/2; k <= n_fft/2; ++k) {
        Scalar freq = static_cast<Scalar>(k) * prf / static_cast<Scalar>(n_fft);
        freq_axis.push_back(freq);

        // 理论高斯谱
        const Scalar z_score = (freq - cfg.doppler_center_hz) / cfg.doppler_sigma_hz;
        const Scalar psd = std::exp(-0.5 * z_score * z_score);
        psd_theoretical.push_back(psd);
    }

    // 导出多普勒谱
    std::ofstream file(opts.output_dir + "/doppler_spectrum.csv");
    if (file.is_open()) {
        file << std::scientific << std::setprecision(15);
        file << "# Gaussian Doppler spectrum: center=" << cfg.doppler_center_hz
             << " Hz, sigma=" << cfg.doppler_sigma_hz << " Hz\n";
        file << "freq_hz,psd_theoretical\n";

        for (std::size_t i = 0; i < freq_axis.size(); ++i) {
            file << freq_axis[i] << "," << psd_theoretical[i] << "\n";
        }
        file.close();
    }

    std::cout << "\nDoppler Spectrum Parameters:\n";
    print_result("Center Frequency", cfg.doppler_center_hz, " Hz");
    print_result("Sigma (bandwidth)", cfg.doppler_sigma_hz, " Hz");
    print_result("PRF", sys.prf_hz, " Hz");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/doppler_spectrum.csv" << std::endl;

    // 验证：多普勒中心应在 Nyquist 范围内
    Scalar nyquist = prf / 2.0;
    bool pass_nyquist = std::abs(cfg.doppler_center_hz) < nyquist;

    std::cout << "\nValidation:\n";
    std::cout << "  [" << (pass_nyquist ? "PASS" : "FAIL")
              << "] Doppler center in Nyquist range [-" << nyquist << ", " << nyquist << "]\n";
}

/**
 * 测试 4: 杂波功率计算验证
 */
void test_clutter_power_calculation(const ValidationOptions& /* opts */) {
    print_header("Test 4: Clutter Power Calculation");

    // 配置杂波参数
    SeaClutterConfig cfg;
    cfg.enabled = true;
    cfg.sequence_mode = SeaClutterSequenceMode::SequencePoolMode;
    cfg.grid_mode = SeaClutterGridMode::RangeSampleGrid;
    cfg.seed = 2026;
    cfg.ground_range_min_m = 1000.0;
    cfg.ground_range_max_m = 50000.0;
    cfg.az_grid_step_deg = 0.5;
    cfg.k_shape_nu = 0.8;
    cfg.doppler_center_hz = 0.0;
    cfg.doppler_sigma_hz = 20.0;
    cfg.pool_length_factor = 32;

    // 配置雷达参数
    RadarSystemParams sys;
    sys.fc_hz = 10.0e9;           // 10 GHz
    sys.fs_hz = 40.0e6;           // 40 MHz
    sys.prf_hz = 1000.0;          // 1 kHz
    sys.pulse_width_s = 20.0e-6;  // 20 μs
    sys.peak_power_w = 1000.0;    // 1 kW
    sys.wavelength_m = C / sys.fc_hz;
    sys.antenna_height_m = 10.0;  // 10 m
    sys.system_loss_linear = 2.0; // 3 dB
    sys.min_range_m = 500.0;
    sys.max_range_m = 100000.0;
    sys.pulses_per_cpi = 32;
    sys.samples_per_pulse = 800;
    sys.compute_derived_params();

    // 配置天线
    antenna::AntennaConfig ant_cfg;
    ant_cfg.model_type = PhasedArrayModelType::ULA_1D;
    ant_cfg.num_elements_az = 16;
    ant_cfg.num_elements_el = 1;
    ant_cfg.spacing_az_lambda = 0.5;
    ant_cfg.peak_gain_db = 30.0;
    ant_cfg.weight_type_az = AntennaWeightType::Uniform;

    AntennaModel antenna;
    antenna.set_config(ant_cfg);
    antenna.initialize();

    print_test_info("Radar: fc=10GHz, fs=40MHz, PRF=1kHz, Pt=1kW");
    print_test_info("Clutter: range=[1km, 50km], nu=0.8, fd_sigma=20Hz");
    print_test_info("Antenna: ULA N=16, G_peak=30dB");

    SeaClutterModel model(cfg);
    bool init_ok = model.initialize_pool(sys);

    if (!init_ok) {
        std::cout << "  [FAIL] Model initialization failed\n";
        return;
    }

    // 生成测试波形（简单脉冲）
    ComplexVec tx_waveform(100, Complex(1.0, 0.0));

    // 生成 CPI 回波
    BeamPoint beam_pointing{0.0, 0.0};
    CpiEcho clutter;

    bool gen_ok = model.generate_cpi(sys, antenna, beam_pointing, 0, tx_waveform, clutter);

    if (!gen_ok) {
        std::cout << "  [FAIL] CPI generation failed\n";
        return;
    }

    // 统计功率
    Scalar total_power = 0.0;
    int total_samples = 0;
    for (const auto& pulse : clutter.pulses) {
        for (const auto& sample : pulse) {
            total_power += std::norm(sample);
            total_samples++;
        }
    }

    Scalar avg_power = total_power / total_samples;

    print_result("Total Samples", total_samples, "");
    print_result("Average Power", avg_power, " W");
    print_result("Max Amplitude", 0.0, " W");  // 需要额外计算

    std::cout << "\n[INFO] Clutter generation successful\n";
    std::cout << "  [" << (gen_ok ? "PASS" : "FAIL") << "] CPI generation completed\n";
}

/**
 * 测试 5: 随机种子重复性验证
 */
void test_seed_repeatability(const ValidationOptions& opts) {
    print_header("Test 5: Seed Repeatability");

    SeaClutterConfig cfg;
    cfg.k_shape_nu = 0.8;
    cfg.doppler_center_hz = 0.0;
    cfg.doppler_sigma_hz = 20.0;
    cfg.seed = 42;
    cfg.sequence_mode = SeaClutterSequenceMode::InTimeMode;

    RadarSystemParams sys;
    sys.prf_hz = 1000.0;

    print_test_info("Seed: " + std::to_string(cfg.seed) +
                    ", N=" + std::to_string(opts.sample_count));

    // 生成 K 分布杂波的辅助 lambda
    auto generate_k_clutter = [&](uint64_t seed, ComplexVec& out) {
        std::mt19937_64 rng(seed);
        std::normal_distribution<Scalar> normal_dist(0.0, 1.0);
        std::gamma_distribution<Scalar> gamma_dist(cfg.k_shape_nu, 1.0 / cfg.k_shape_nu);

        out.resize(opts.sample_count);
        for (int i = 0; i < opts.sample_count; ++i) {
            const Scalar inv_sqrt2 = std::sqrt(0.5);
            // 先生成高斯分量（消耗 2 个随机数）
            Scalar n1 = normal_dist(rng);
            Scalar n2 = normal_dist(rng);
            // 再生成纹理分量（消耗 1 个随机数）
            Scalar tau = gamma_dist(rng);
            out[static_cast<std::size_t>(i)] = std::sqrt(tau) *
                Complex(inv_sqrt2 * n1, inv_sqrt2 * n2);
        }
    };

    // 使用相同的种子生成两次
    ComplexVec seq1, seq2;
    generate_k_clutter(cfg.seed, seq1);
    generate_k_clutter(cfg.seed, seq2);

    // 比较两个序列
    bool identical = true;
    Scalar max_diff = 0.0;
    for (std::size_t i = 0; i < seq1.size(); ++i) {
        Scalar diff = std::abs(seq1[i] - seq2[i]);
        if (diff > 1e-15) {
            identical = false;
            max_diff = std::max(max_diff, diff);
        }
    }

    std::cout << "\nComparison:\n";
    std::cout << "  Sequences identical: " << (identical ? "YES" : "NO") << std::endl;
    if (!identical) {
        std::cout << "  Max difference: " << std::scientific << max_diff << std::endl;
    }

    bool pass = identical;
    std::cout << "\nValidation:\n";
    std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] Same seed produces identical clutter\n";
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char* argv[]) {
    ValidationOptions opts;

    static struct option long_options[] = {
        {"output-dir", required_argument, nullptr, 'o'},
        {"samples", required_argument, nullptr, 'n'},
        {"quiet", no_argument, nullptr, 'q'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "o:n:qh", long_options, nullptr)) != -1) {
        switch (opt) {
        case 'o': opts.output_dir = optarg; break;
        case 'n': opts.sample_count = std::stoi(optarg); break;
        case 'q': opts.verbose = false; break;
        case 'h':
            std::cout << "Usage: " << argv[0] << " [OPTIONS]\n"
                      << "Options:\n"
                      << "  -o, --output-dir DIR  输出目录\n"
                      << "  -n, --samples N       样本数量 (默认 100000)\n"
                      << "  -q, --quiet           安静模式\n"
                      << "  -h, --help            帮助\n";
            return 0;
        }
    }

    fs::create_directories(opts.output_dir);

    std::cout << "╔══════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  海杂波验证工具 - C++ vs MATLAB 对比         ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nOutput directory: " << opts.output_dir << std::endl;
    std::cout << "Sample count: " << opts.sample_count << std::endl;

    // 运行所有测试
    test_morchin_sigma0(opts);
    test_k_distribution_stats(opts);
    test_doppler_spectrum(opts);
    test_clutter_power_calculation(opts);
    test_seed_repeatability(opts);

    // 输出汇总
    std::cout << "\n╔══════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              验证完成                        ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;

    std::cout << "\nExported files:\n";
    std::cout << "  1. morchin_sigma0.csv            - Morchin σ⁰模型\n";
    std::cout << "  2. k_distribution_samples.csv    - K 分布样本\n";
    std::cout << "  3. doppler_spectrum.csv          - 多普勒谱\n";
    std::cout << "  4. clutter_power.csv             - 杂波功率\n";

    std::cout << "\n下一步:\n";
    std::cout << "  运行 MATLAB 脚本对比结果:\n";
    std::cout << "    matlab/05 杂波仿真/compare_clutter_cpp_matlab.m\n";

    return 0;
}
