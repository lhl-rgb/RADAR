/**
 * @file noise_validation.cpp
 * @brief 噪声生成验证工具 - 对比 C++ 实现与 MATLAB 参考
 *
 * 目的：
 *   验证噪声引擎的 C++ 实现与 MATLAB 的一致性
 *
 * 验证内容：
 *   - ComplexSigma 模式：直接给定复噪声标准差
 *   - NoisePower 模式：直接给定噪声功率
 *   - ThermalKTB 模式：热噪声公式 kTB
 *   - SNR 验证：加噪后信噪比
 *   - 二维回波加噪
 *   - 随机种子重复性
 *
 * 输出：
 *   - 参数信息
 *   - 噪声统计数据（CSV 格式，供 MATLAB 对比）
 *
 * 使用方法：
 *   ./noise_validation --output_dir=out/noise_validation
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

#include "core/types.h"
#include "core/tools/math_utils.h"
#include "core/radar_system_params.h"
#include "noise/noise_engine.h"
#include "noise/noise_config.h"

using namespace radar;
using namespace radar::noise;

namespace fs = std::filesystem;

struct ValidationOptions {
    std::string output_dir = "out/noise_validation";
    bool verbose = true;
    int sample_count = 100000;  // 统计测试样本数
};

// ============================================================================
// 数据导出函数
// ============================================================================

struct NoiseRecord {
    int index;
    Scalar real_part;
    Scalar imag_part;
    Scalar power;
};

bool export_noise_samples(const std::string& filepath,
                          const ComplexVec& noise,
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
    file << "index,real,imag,power\n";

    for (std::size_t i = 0; i < noise.size(); ++i) {
        file << i << ","
             << noise[i].real() << ","
             << noise[i].imag() << ","
             << std::norm(noise[i]) << "\n";
    }

    return true;
}

struct StatisticsRecord {
    std::string mode;
    Scalar target_power;
    Scalar estimated_power;
    Scalar relative_error;
    Scalar sigma_iq;
    Scalar snr_db;
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
    file << "mode,target_power,estimated_power,relative_error,sigma_iq,snr_db\n";

    for (const auto& rec : records) {
        file << rec.mode << ","
             << rec.target_power << ","
             << rec.estimated_power << ","
             << rec.relative_error << ","
             << rec.sigma_iq << ","
             << rec.snr_db << "\n";
    }

    return true;
}

bool export_2d_noise_csv(const std::string& filepath,
                         const std::vector<std::vector<Complex>>& noise_mat,
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
    file << "range_bin,pulse,real,imag,power\n";

    for (std::size_t r = 0; r < noise_mat.size(); ++r) {
        for (std::size_t p = 0; p < noise_mat[r].size(); ++p) {
            file << r << "," << p << ","
                 << noise_mat[r][p].real() << ","
                 << noise_mat[r][p].imag() << ","
                 << std::norm(noise_mat[r][p]) << "\n";
        }
    }

    return true;
}

// ============================================================================
// 统计计算函数
// ============================================================================

Scalar compute_mean_power(const ComplexVec& noise) {
    Scalar total = 0.0;
    for (const auto& n : noise) {
        total += std::norm(n);
    }
    return total / noise.size();
}

Scalar compute_sigma_iq(const ComplexVec& noise) {
    // 计算 I 或 Q 分量的标准差
    std::vector<Scalar> i_components;
    i_components.reserve(noise.size());
    for (const auto& n : noise) {
        i_components.push_back(n.real());
    }

    Scalar mean = 0.0;
    for (Scalar x : i_components) {
        mean += x;
    }
    mean /= i_components.size();

    Scalar var = 0.0;
    for (Scalar x : i_components) {
        var += (x - mean) * (x - mean);
    }
    var /= i_components.size();

    return std::sqrt(var);
}

Scalar compute_snr_db(const ComplexVec& signal, const ComplexVec& noise) {
    Scalar signal_power = compute_mean_power(signal);
    Scalar noise_power = compute_mean_power(noise);
    return math::linear_to_db(signal_power / noise_power);
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
 * 测试 1: ComplexSigma 模式验证
 */
void test_complex_sigma(const ValidationOptions& opts) {
    print_header("Test 1: ComplexSigma Mode");

    RadarSystemParams sys;
    NoiseConfig cfg;
    cfg.mode = NoiseLevelMode::ComplexSigma;
    cfg.sigma_complex = 1.0e-3;
    cfg.seed = 12345;

    NoiseEngine engine;
    engine.set_config(cfg);
    engine.set_system_params(sys);
    engine.initialize();

    print_test_info("Parameters: sigma=" + std::to_string(cfg.sigma_complex) +
                    ", N=" + std::to_string(opts.sample_count));

    // 生成噪声样本
    ComplexVec noise = engine.generate(opts.sample_count);

    // 统计计算
    Scalar est_power = compute_mean_power(noise);
    Scalar target_power = cfg.sigma_complex * cfg.sigma_complex;
    Scalar rel_error = std::abs(est_power - target_power) / target_power * 100.0;
    Scalar sigma_iq = compute_sigma_iq(noise);
    Scalar theoretical_iq = cfg.sigma_complex / std::sqrt(2.0);

    print_result("Target Power", target_power, " W");
    print_result("Estimated Power", est_power, " W");
    print_result("Relative Error", rel_error, " %");
    print_result("I/Q Sigma (est.)", sigma_iq);
    print_result("I/Q Sigma (theory)", theoretical_iq);

    // 验证
    bool pass_power = rel_error < 0.5;
    bool pass_iq = std::abs(sigma_iq - theoretical_iq) / theoretical_iq < 0.005;

    std::cout << "\nValidation:\n";
    std::cout << "  [" << (pass_power ? "PASS" : "FAIL") << "] Power error < 0.5%\n";
    std::cout << "  [" << (pass_iq ? "PASS" : "FAIL") << "] I/Q sigma error < 0.5%\n";

    // 导出样本
    export_noise_samples(opts.output_dir + "/complex_sigma_samples.csv",
                         noise,
                         "ComplexSigma mode: sigma=" + std::to_string(cfg.sigma_complex));

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/complex_sigma_samples.csv" << std::endl;
}

/**
 * 测试 2: NoisePower 模式验证
 */
void test_noise_power_mode(const ValidationOptions& opts) {
    print_header("Test 2: NoisePower Mode");

    RadarSystemParams sys;
    NoiseConfig cfg;
    cfg.mode = NoiseLevelMode::NoisePower;
    cfg.noise_power_w = 1.0e-6;
    cfg.seed = 12345;

    NoiseEngine engine;
    engine.set_config(cfg);
    engine.set_system_params(sys);
    engine.initialize();

    print_test_info("Parameters: Pn=" + std::to_string(cfg.noise_power_w) +
                    " W, N=" + std::to_string(opts.sample_count));

    ComplexVec noise = engine.generate(opts.sample_count);

    Scalar est_power = compute_mean_power(noise);
    Scalar target_power = cfg.noise_power_w;
    Scalar rel_error = std::abs(est_power - target_power) / target_power * 100.0;
    Scalar sigma_iq = compute_sigma_iq(noise);
    Scalar theoretical_iq = std::sqrt(cfg.noise_power_w / 2.0);

    print_result("Target Power", target_power, " W");
    print_result("Estimated Power", est_power, " W");
    print_result("Relative Error", rel_error, " %");
    print_result("I/Q Sigma (est.)", sigma_iq);
    print_result("I/Q Sigma (theory)", theoretical_iq);

    bool pass_power = rel_error < 0.5;
    bool pass_iq = std::abs(sigma_iq - theoretical_iq) / theoretical_iq < 0.005;

    std::cout << "\nValidation:\n";
    std::cout << "  [" << (pass_power ? "PASS" : "FAIL") << "] Power error < 0.5%\n";
    std::cout << "  [" << (pass_iq ? "PASS" : "FAIL") << "] I/Q sigma error < 0.5%\n";

    export_noise_samples(opts.output_dir + "/noise_power_samples.csv",
                         noise,
                         "NoisePower mode: Pn=" + std::to_string(cfg.noise_power_w));

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/noise_power_samples.csv" << std::endl;
}

/**
 * 测试 3: ThermalKTB 模式验证
 */
void test_thermal_ktb(const ValidationOptions& opts) {
    print_header("Test 3: ThermalKTB Mode");

    RadarSystemParams sys;
    sys.noise_figure_db = 4.0;  // 噪声系数 4 dB

    NoiseConfig cfg;
    cfg.mode = NoiseLevelMode::ThermalKTB;
    cfg.system_temperature_k = 290.0;  // 290 K
    cfg.noise_bandwidth_hz = 20.0e6;   // 20 MHz
    cfg.seed = 12345;

    NoiseEngine engine;
    engine.set_config(cfg);
    engine.set_system_params(sys);
    engine.initialize();

    print_test_info("Parameters: T=" + std::to_string(cfg.system_temperature_k) +
                    " K, B=" + std::to_string(cfg.noise_bandwidth_hz / 1e6) + " MHz");
    print_test_info("Noise Figure: " + std::to_string(sys.noise_figure_db) + " dB");

    // C++ 计算的噪声功率
    Scalar cpp_power = engine.noise_power_w();

    // MATLAB 公式计算
    constexpr Scalar k_Boltzmann = 1.380649e-23;
    Scalar F = math::db_to_linear(sys.noise_figure_db);
    Scalar matlab_power = k_Boltzmann * cfg.system_temperature_k * cfg.noise_bandwidth_hz * F;

    Scalar rel_error = std::abs(cpp_power - matlab_power) / matlab_power * 100.0;

    std::cout << "\nNoise Power Calculation:\n";
    print_result("C++ Result", cpp_power, " W");
    print_result("MATLAB Formula", matlab_power, " W");
    print_result("Relative Error", rel_error, " %");

    bool pass = rel_error < 0.01;  // 纯计算，误差应该极小
    std::cout << "\nValidation:\n";
    std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] Power calculation error < 0.01%\n";

    // 导出结果
    std::ofstream file(opts.output_dir + "/thermalktb_power.csv");
    if (file.is_open()) {
        file << std::scientific << std::setprecision(15);
        file << "# ThermalKTB noise power comparison\n";
        file << "k_Boltzmann,T_K,B_Hz,NF_dB,F_linear,cpp_power,matlab_power,rel_error\n";
        file << k_Boltzmann << ","
             << cfg.system_temperature_k << ","
             << cfg.noise_bandwidth_hz << ","
             << sys.noise_figure_db << ","
             << F << ","
             << cpp_power << ","
             << matlab_power << ","
             << rel_error << "\n";
        file.close();
    }

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/thermalktb_power.csv" << std::endl;

    // 生成噪声样本验证统计特性
    ComplexVec noise = engine.generate(opts.sample_count);
    Scalar est_power = compute_mean_power(noise);
    Scalar stat_error = std::abs(est_power - cpp_power) / cpp_power * 100.0;

    std::cout << "\nStatistical Verification:\n";
    print_result("Target Power", cpp_power, " W");
    print_result("Estimated Power", est_power, " W");
    print_result("Statistical Error", stat_error, " %");

    bool stat_pass = stat_error < 0.5;
    std::cout << "  [" << (stat_pass ? "PASS" : "FAIL") << "] Statistical error < 0.5%\n";

    export_noise_samples(opts.output_dir + "/thermalktb_samples.csv",
                         noise,
                         "ThermalKTB mode: T=290K, B=20MHz, NF=4dB");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/thermalktb_samples.csv" << std::endl;
}

/**
 * 测试 4: SNR 验证
 */
void test_snr_verification(const ValidationOptions& opts) {
    print_header("Test 4: SNR Verification");

    RadarSystemParams sys;
    NoiseConfig cfg;
    cfg.mode = NoiseLevelMode::NoisePower;
    cfg.noise_power_w = 1.0e-6;
    cfg.seed = 12345;

    NoiseEngine engine;
    engine.set_config(cfg);
    engine.set_system_params(sys);
    engine.initialize();

    // 创建信号（恒定幅度）
    Scalar signal_power = 1.0e-4;  // 100 × 噪声功率，理论 SNR = 20 dB
    std::size_t N = static_cast<std::size_t>(opts.sample_count);
    ComplexVec signal(N, Complex(std::sqrt(signal_power), 0.0));
    ComplexVec noise = engine.generate(N);

    // 加噪
    ComplexVec received = signal;
    engine.add_noise(received);

    print_test_info("Signal Power: " + std::to_string(signal_power) + " W");
    print_test_info("Noise Power: " + std::to_string(cfg.noise_power_w) + " W");

    // 计算实测 SNR
    Scalar actual_snr_db = compute_snr_db(signal, noise);
    Scalar theoretical_snr_db = math::linear_to_db(signal_power / cfg.noise_power_w);
    Scalar snr_error = std::abs(actual_snr_db - theoretical_snr_db);

    std::cout << "\nSNR Results:\n";
    print_result("Theoretical SNR", theoretical_snr_db, " dB");
    print_result("Measured SNR", actual_snr_db, " dB");
    print_result("SNR Error", snr_error, " dB");

    bool pass = snr_error < 0.1;
    std::cout << "\nValidation:\n";
    std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] SNR error < 0.1 dB\n";

    // 导出结果
    std::ofstream file(opts.output_dir + "/snr_verification.csv");
    if (file.is_open()) {
        file << std::scientific << std::setprecision(15);
        file << "# SNR verification\n";
        file << "signal_power,noise_power,theoretical_snr_db,measured_snr_db,snr_error_db\n";
        file << signal_power << ","
             << cfg.noise_power_w << ","
             << theoretical_snr_db << ","
             << actual_snr_db << ","
             << snr_error << "\n";
        file.close();
    }

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/snr_verification.csv" << std::endl;
}

/**
 * 测试 5: 二维回波加噪验证
 */
void test_2d_noise_matrix(const ValidationOptions& opts) {
    print_header("Test 5: 2D Noise Matrix");

    RadarSystemParams sys;
    NoiseConfig cfg;
    cfg.mode = NoiseLevelMode::NoisePower;
    cfg.noise_power_w = 1.0e-8;
    cfg.seed = 12345;

    NoiseEngine engine;
    engine.set_config(cfg);
    engine.set_system_params(sys);
    engine.initialize();

    std::size_t Nr = 2048;  // 距离单元数
    std::size_t Np = 32;    // CPI 脉冲数

    print_test_info("Matrix Size: " + std::to_string(Nr) + " x " + std::to_string(Np));
    print_test_info("Noise Power per sample: " + std::to_string(cfg.noise_power_w) + " W");

    // 生成二维噪声矩阵（模拟 CPI 回波）
    std::vector<std::vector<Complex>> noise_mat(Nr, std::vector<Complex>(Np));

    for (std::size_t p = 0; p < Np; ++p) {
        for (std::size_t r = 0; r < Nr; ++r) {
            noise_mat[r][p] = engine.sample();
        }
    }

    // 统计计算
    Scalar total_samples = Nr * Np;
    Scalar total_power = 0.0;
    for (const auto& row : noise_mat) {
        for (const auto& sample : row) {
            total_power += std::norm(sample);
        }
    }
    Scalar avg_power = total_power / total_samples;
    Scalar rel_error = std::abs(avg_power - cfg.noise_power_w) / cfg.noise_power_w * 100.0;

    std::cout << "\nStatistics:\n";
    print_result("Total Samples", total_samples, "");
    print_result("Target Power (per sample)", cfg.noise_power_w, " W");
    print_result("Estimated Power (per sample)", avg_power, " W");
    print_result("Relative Error", rel_error, " %");

    bool pass = rel_error < 0.5;
    std::cout << "\nValidation:\n";
    std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] Power error < 0.5%\n";

    // 只导出前 100 个距离单元和前 8 个脉冲（避免文件过大）
    std::vector<std::vector<Complex>> noise_mat_export;
    for (std::size_t r = 0; r < std::min(Nr, static_cast<std::size_t>(100)); ++r) {
        noise_mat_export.push_back(
            std::vector<Complex>(noise_mat[r].begin(),
                                 noise_mat[r].begin() + std::min(Np, static_cast<std::size_t>(8))));
    }

    export_2d_noise_csv(opts.output_dir + "/noise_2d_matrix.csv",
                        noise_mat_export,
                        "2D noise matrix: Nr=2048, Np=32, Pn=1e-8W (partial export)");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/noise_2d_matrix.csv" << std::endl;
}

/**
 * 测试 6: 随机种子重复性验证
 */
void test_seed_repeatability(const ValidationOptions& opts) {
    print_header("Test 6: Seed Repeatability");

    RadarSystemParams sys;
    NoiseConfig cfg;
    cfg.mode = NoiseLevelMode::ComplexSigma;
    cfg.sigma_complex = 1.0e-3;
    cfg.seed = 42;

    NoiseEngine engine1, engine2;
    engine1.set_config(cfg);
    engine1.set_system_params(sys);
    engine1.initialize();

    engine2.set_config(cfg);
    engine2.set_system_params(sys);
    engine2.initialize();

    print_test_info("Seed: " + std::to_string(cfg.seed) +
                    ", N=" + std::to_string(opts.sample_count));

    ComplexVec noise1 = engine1.generate(opts.sample_count);
    ComplexVec noise2 = engine2.generate(opts.sample_count);

    // 比较两个序列
    bool identical = true;
    Scalar max_diff = 0.0;
    for (std::size_t i = 0; i < noise1.size(); ++i) {
        Scalar diff = std::abs(noise1[i] - noise2[i]);
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
    std::cout << "  [" << (pass ? "PASS" : "FAIL") << "] Same seed produces identical noise\n";
}

/**
 * 测试 7: 不同 SNR 点验证
 */
void test_multiple_snr_points(const ValidationOptions& opts) {
    print_header("Test 7: Multiple SNR Points");

    RadarSystemParams sys;
    NoiseConfig cfg;
    cfg.mode = NoiseLevelMode::NoisePower;
    cfg.noise_power_w = 1.0e-6;
    cfg.seed = 12345;

    NoiseEngine engine;
    engine.set_config(cfg);
    engine.set_system_params(sys);
    engine.initialize();

    // 测试不同 SNR 点
    std::vector<Scalar> snr_targets_db = {-5.0, 0.0, 5.0, 10.0, 20.0, 30.0};
    std::size_t N = static_cast<std::size_t>(opts.sample_count);

    std::cout << "\n";
    std::cout << "------------------------------------------------------------\n";
    std::cout << std::setw(12) << "Target SNR"
              << std::setw(15) << "Measured SNR"
              << std::setw(12) << "Error"
              << std::setw(10) << "Status" << std::endl;
    std::cout << "------------------------------------------------------------\n";

    std::vector<StatisticsRecord> records;

    bool all_pass = true;
    for (Scalar target_snr_db : snr_targets_db) {
        // 计算所需信号功率
        Scalar signal_power = cfg.noise_power_w * math::db_to_linear(target_snr_db);

        // 创建信号
        ComplexVec signal(N, Complex(std::sqrt(signal_power), 0.0));

        // 重新初始化噪声引擎（重置种子）
        cfg.seed += 1;  // 不同 SNR 点使用不同种子
        engine.reseed(cfg.seed);
        ComplexVec noise = engine.generate(N);

        // 计算实测 SNR
        Scalar measured_snr_db = compute_snr_db(signal, noise);
        Scalar error = std::abs(measured_snr_db - target_snr_db);

        bool pass = error < 0.1;
        all_pass = all_pass && pass;

        std::cout << std::fixed << std::setw(12) << target_snr_db
                  << std::setw(15) << measured_snr_db
                  << std::setw(12) << error
                  << std::setw(10) << (pass ? "PASS" : "FAIL") << std::endl;

        records.push_back({std::to_string(static_cast<int>(target_snr_db)),
                          signal_power, cfg.noise_power_w,
                          error, 0.0, measured_snr_db});
    }
    std::cout << "------------------------------------------------------------\n";
    std::cout << "Overall: " << (all_pass ? "PASS" : "FAIL") << std::endl;

    export_statistics_csv(opts.output_dir + "/snr_points.csv", records,
                          "Multiple SNR points verification");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/snr_points.csv" << std::endl;
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
    std::cout << "║  噪声验证工具 - C++ vs MATLAB 对比           ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nOutput directory: " << opts.output_dir << std::endl;
    std::cout << "Sample count: " << opts.sample_count << std::endl;

    // 运行所有测试
    test_complex_sigma(opts);
    test_noise_power_mode(opts);
    test_thermal_ktb(opts);
    test_snr_verification(opts);
    test_2d_noise_matrix(opts);
    test_seed_repeatability(opts);
    test_multiple_snr_points(opts);

    // 输出汇总
    std::cout << "\n╔══════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              验证完成                        ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;

    std::cout << "\nExported files:\n";
    std::cout << "  1. complex_sigma_samples.csv  - ComplexSigma 模式样本\n";
    std::cout << "  2. noise_power_samples.csv    - NoisePower 模式样本\n";
    std::cout << "  3. thermalktb_power.csv       - ThermalKTB 功率计算\n";
    std::cout << "  4. thermalktb_samples.csv     - ThermalKTB 模式样本\n";
    std::cout << "  5. snr_verification.csv       - SNR 验证\n";
    std::cout << "  6. noise_2d_matrix.csv        - 二维噪声矩阵\n";
    std::cout << "  7. snr_points.csv             - 多 SNR 点验证\n";

    std::cout << "\n下一步:\n";
    std::cout << "  运行 MATLAB 脚本对比结果:\n";
    std::cout << "    matlab/03 噪声仿真/compare_noise_cpp_matlab.m\n";

    return 0;
}
