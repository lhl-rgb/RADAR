/**
 * @file antenna_validation.cpp
 * @brief 天线部分验证工具 - 对比 C++ 实现与 MATLAB 参考
 *
 * 目的：
 *   验证相控阵天线 (ULA/UPA) 的 C++ 实现与 MATLAB 的一致性
 *
 * 验证内容：
 *   - ULA 单波位增益
 *   - ULA 多波位扫描
 *   - UPA 单波位增益
 *   - UPA 二维扫描
 *   - 归一化功率方向图
 *
 * 输出：
 *   - 参数信息
 *   - 增益数据（CSV 格式，供 MATLAB 对比）
 *   - 方向图数据
 *
 * 使用方法：
 *   ./antenna_validation --output_dir=out/antenna_validation
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

#include "core/types.h"
#include "core/tools/math_utils.h"
#include "antenna/antenna_model.h"
#include "antenna/beam_scanner.h"
#include "antenna/antenna_config.h"

using namespace radar;
using namespace radar::antenna;

namespace fs = std::filesystem;

struct ValidationOptions {
    std::string output_dir = "out/antenna_validation";
    bool verbose = true;
    bool export_pattern = true;
};

// ============================================================================
// 数据导出函数
// ============================================================================

/**
 * @brief 导出增益对比数据为 CSV
 */
bool export_gain_csv(const std::string& filepath,
                     const std::vector<std::tuple<Scalar, Scalar, Scalar, Scalar>>& data,
                     const std::string& description = "") {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return false;
    }

    file << std::scientific << std::setprecision(15);
    file << "# " << description << "\n";
    file << "target_az_deg,target_el_deg,beam_az_deg,beam_el_deg,"
         << "gain_linear,gain_db,normalized_power\n";

    for (const auto& row : data) {
        file << std::get<0>(row) << ","    // target_az
             << std::get<1>(row) << ","    // target_el
             << std::get<2>(row) << ","    // beam_az
             << std::get<3>(row) << ","    // beam_el
             << "0,0,0\n";                 // 占位，实际值由后续填充
    }

    return true;
}

/**
 * @brief 导出增益数据（简化版）
 */
struct GainRecord {
    Scalar target_az;
    Scalar target_el;
    Scalar beam_az;
    Scalar beam_el;
    Scalar gain_linear;
    Scalar gain_db;
    Scalar norm_power;
};

bool export_gain_records(const std::string& filepath,
                         const std::vector<GainRecord>& records,
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
    file << "target_az_deg,target_el_deg,beam_az_deg,beam_el_deg,"
         << "gain_linear,gain_db,normalized_power\n";

    for (const auto& rec : records) {
        file << rec.target_az << ","
             << rec.target_el << ","
             << rec.beam_az << ","
             << rec.beam_el << ","
             << rec.gain_linear << ","
             << rec.gain_db << ","
             << rec.norm_power << "\n";
    }

    return true;
}

/**
 * @brief 导出方向图数据为 CSV
 */
bool export_pattern_csv(const std::string& filepath,
                        const std::vector<Scalar>& angles,
                        const std::vector<Scalar>& gain_db,
                        const std::vector<Scalar>& norm_power,
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
    file << "angle_deg,gain_db,normalized_power,norm_power_db\n";

    for (std::size_t i = 0; i < angles.size(); ++i) {
        Scalar norm_power_db = (norm_power[i] > 0) ?
            10.0 * std::log10(norm_power[i]) : -100.0;
        file << angles[i] << ","
             << gain_db[i] << ","
             << norm_power[i] << ","
             << norm_power_db << "\n";
    }

    return true;
}

/**
 * @brief 导出二维方向图数据为 CSV
 */
bool export_pattern_2d_csv(const std::string& filepath,
                           const std::vector<Scalar>& az_angles,
                           const std::vector<Scalar>& el_angles,
                           const std::vector<std::vector<Scalar>>& gain_db,
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

    // 第一行：方位角
    file << "el_deg\\az_deg,";
    for (std::size_t i = 0; i < az_angles.size(); ++i) {
        file << az_angles[i];
        if (i < az_angles.size() - 1) file << ",";
    }
    file << "\n";

    // 后续行：俯仰角 + 数据
    for (std::size_t j = 0; j < el_angles.size(); ++j) {
        file << el_angles[j] << ",";
        for (std::size_t i = 0; i < az_angles.size(); ++i) {
            file << gain_db[j][i];
            if (i < az_angles.size() - 1) file << ",";
        }
        file << "\n";
    }

    return true;
}

// ============================================================================
// 测试辅助函数
// ============================================================================

void print_header(const std::string& title) {
    std::cout << "\n";
    std::cout << "============================================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "============================================================\n";
}

void print_test_info(const std::string& info) {
    std::cout << "\n[INFO] " << info << std::endl;
}

void print_result(const std::string& name, Scalar value, const std::string& unit = "") {
    std::cout << "  " << std::left << std::setw(35) << name
              << ": " << std::fixed << std::setprecision(6) << value << unit << std::endl;
}

// ============================================================================
// 测试用例
// ============================================================================

/**
 * 测试 1: ULA 单波位增益验证
 */
void test_ula_single_beam(const ValidationOptions& opts) {
    print_header("Test 1: ULA Single Beam Gain");

    // 配置与 MATLAB demo_ula_scan.m 一致
    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::ULA_1D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 1;
    cfg.spacing_az_lambda = 0.5;
    cfg.spacing_el_lambda = 0.5;
    cfg.peak_gain_db = 30.0;
    cfg.weight_type_az = AntennaWeightType::Uniform;
    cfg.weight_type_el = AntennaWeightType::Uniform;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    // 测试参数
    Scalar beam_az = 0.0;
    Scalar target_az = 10.0;
    Scalar target_el = 0.0;
    Scalar beam_el = 0.0;

    print_test_info("Parameters: N=16, d/lambda=0.5, Uniform, G_peak=30dB");
    print_test_info("Beam pointing: " + std::to_string(beam_az) + " deg");
    print_test_info("Target azimuth: " + std::to_string(target_az) + " deg");

    Scalar gain_lin = model.gain(target_az, target_el, beam_az, beam_el);
    Scalar gain_db = model.gain_db(target_az, target_el, beam_az, beam_el);
    Scalar norm_pwr = model.normalized_power(target_az, target_el, beam_az, beam_el);

    print_result("Gain (linear)", gain_lin, "");
    print_result("Gain (dB)", gain_db, " dB");
    print_result("Normalized Power", norm_pwr, "");

    // 导出数据
    std::vector<GainRecord> records;
    records.push_back({target_az, target_el, beam_az, beam_el, gain_lin, gain_db, norm_pwr});
    export_gain_records(opts.output_dir + "/ula_single_beam.csv", records,
                        "ULA single beam test: N=16, d/lambda=0.5, Uniform, G_peak=30dB");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/ula_single_beam.csv" << std::endl;
}

/**
 * 测试 2: ULA 多波位扫描验证
 */
void test_ula_scan(const ValidationOptions& opts) {
    print_header("Test 2: ULA Multi-Beam Scan");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::ULA_1D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 1;
    cfg.spacing_az_lambda = 0.5;
    cfg.peak_gain_db = 30.0;
    cfg.weight_type_az = AntennaWeightType::Uniform;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    // 波位表与 MATLAB demo_ula_scan_with_beam_table.m 一致
    std::vector<Scalar> beam_az_list = {-20.0, -10.0, 0.0, 10.0, 20.0};
    Scalar target_az = 10.0;
    Scalar target_el = 0.0;

    print_test_info("Beam table: [-20, -10, 0, 10, 20] deg");
    print_test_info("Target azimuth: " + std::to_string(target_az) + " deg");

    std::vector<GainRecord> records;

    std::cout << "\nScan Results:\n";
    std::cout << "------------------------------------------------------------\n";
    std::cout << std::setw(8) << "Beam"
              << std::setw(12) << "Beam Az"
              << std::setw(12) << "Target Az"
              << std::setw(14) << "P_norm (dB)"
              << std::setw(14) << "Gain (dBi)" << std::endl;
    std::cout << "------------------------------------------------------------\n";

    for (std::size_t i = 0; i < beam_az_list.size(); ++i) {
        Scalar beam_az = beam_az_list[i];
        Scalar gain_lin = model.gain(target_az, target_el, beam_az, 0.0);
        Scalar gain_db = model.gain_db(target_az, target_el, beam_az, 0.0);
        Scalar norm_pwr = model.normalized_power(target_az, target_el, beam_az, 0.0);
        Scalar norm_pwr_db = 10.0 * std::log10(std::max(norm_pwr, 1e-15));

        records.push_back({target_az, target_el, beam_az, 0.0, gain_lin, gain_db, norm_pwr});

        std::cout << std::setw(8) << i + 1
                  << std::setw(12) << beam_az
                  << std::setw(12) << target_az
                  << std::setw(14) << std::fixed << std::setprecision(4) << norm_pwr_db
                  << std::setw(14) << gain_db << std::endl;
    }
    std::cout << "------------------------------------------------------------\n";

    export_gain_records(opts.output_dir + "/ula_scan.csv", records,
                        "ULA scan test: N=16, d/lambda=0.5, Uniform, G_peak=30dB, Target=10deg");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/ula_scan.csv" << std::endl;
}

/**
 * 测试 3: UPA 单波位增益验证
 */
void test_upa_single_beam(const ValidationOptions& opts) {
    print_header("Test 3: UPA Single Beam Gain");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::UPA_2D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 12;
    cfg.spacing_az_lambda = 0.5;
    cfg.spacing_el_lambda = 0.5;
    cfg.peak_gain_db = 35.0;
    cfg.weight_type_az = AntennaWeightType::Hamming;
    cfg.weight_type_el = AntennaWeightType::Hamming;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    Scalar beam_az = 0.0;
    Scalar beam_el = 0.0;
    Scalar target_az = 0.0;
    Scalar target_el = 0.0;

    print_test_info("Parameters: Nx=16, Ny=12, d/lambda=0.5, Hamming, G_peak=35dB");
    print_test_info("Beam pointing: (0, 0) deg");
    print_test_info("Target: (0, 0) deg");

    Scalar gain_lin = model.gain(target_az, target_el, beam_az, beam_el);
    Scalar gain_db = model.gain_db(target_az, target_el, beam_az, beam_el);
    Scalar norm_pwr = model.normalized_power(target_az, target_el, beam_az, beam_el);

    print_result("Gain (linear)", gain_lin, "");
    print_result("Gain (dB)", gain_db, " dB");
    print_result("Normalized Power", norm_pwr, "");

    std::vector<GainRecord> records;
    records.push_back({target_az, target_el, beam_az, beam_el, gain_lin, gain_db, norm_pwr});
    export_gain_records(opts.output_dir + "/upa_single_beam.csv", records,
                        "UPA single beam test: Nx=16, Ny=12, d/lambda=0.5, Hamming, G_peak=35dB");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/upa_single_beam.csv" << std::endl;
}

/**
 * 测试 4: UPA 二维扫描验证
 */
void test_upa_scan(const ValidationOptions& opts) {
    print_header("Test 4: UPA 2D Scan");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::UPA_2D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 12;
    cfg.spacing_az_lambda = 0.5;
    cfg.spacing_el_lambda = 0.5;
    cfg.peak_gain_db = 35.0;
    cfg.weight_type_az = AntennaWeightType::Hamming;
    cfg.weight_type_el = AntennaWeightType::Hamming;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    // 波位表与 MATLAB demo_upa_scan_with_beam_table.m 一致
    std::vector<Scalar> beam_az_list = {-10.0, -10.0, -10.0, 0.0, 0.0, 0.0, 10.0, 10.0, 10.0};
    std::vector<Scalar> beam_el_list = {-5.0, 0.0, 5.0, -5.0, 0.0, 5.0, -5.0, 0.0, 5.0};
    Scalar target_az = 0.0;
    Scalar target_el = 0.0;

    print_test_info("Beam table: 3x3 grid (Az: -10,0,10 x El: -5,0,5) deg");
    print_test_info("Target: (0, 0) deg");

    std::vector<GainRecord> records;

    std::cout << "\nScan Results:\n";
    std::cout << "-------------------------------------------------------------------\n";
    std::cout << std::setw(6) << "Beam"
              << std::setw(12) << "Beam Az"
              << std::setw(12) << "Beam El"
              << std::setw(14) << "P_norm"
              << std::setw(14) << "Gain (dBi)" << std::endl;
    std::cout << "-------------------------------------------------------------------\n";

    for (std::size_t i = 0; i < beam_az_list.size(); ++i) {
        Scalar beam_az = beam_az_list[i];
        Scalar beam_el = beam_el_list[i];
        Scalar gain_lin = model.gain(target_az, target_el, beam_az, beam_el);
        Scalar gain_db = model.gain_db(target_az, target_el, beam_az, beam_el);
        Scalar norm_pwr = model.normalized_power(target_az, target_el, beam_az, beam_el);

        records.push_back({target_az, target_el, beam_az, beam_el, gain_lin, gain_db, norm_pwr});

        std::cout << std::setw(6) << i + 1
                  << std::setw(12) << beam_az
                  << std::setw(12) << beam_el
                  << std::setw(14) << std::fixed << std::setprecision(4) << norm_pwr
                  << std::setw(14) << gain_db << std::endl;
    }
    std::cout << "-------------------------------------------------------------------\n";

    export_gain_records(opts.output_dir + "/upa_scan.csv", records,
                        "UPA 2D scan test: Nx=16, Ny=12, d/lambda=0.5, Hamming, G_peak=35dB, Target=(0,0)");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/upa_scan.csv" << std::endl;
}

/**
 * 测试 5: ULA 方向图验证
 */
void test_ula_pattern(const ValidationOptions& opts) {
    print_header("Test 5: ULA Pattern");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::ULA_1D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 1;
    cfg.spacing_az_lambda = 0.5;
    cfg.peak_gain_db = 30.0;
    cfg.weight_type_az = AntennaWeightType::Uniform;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    Scalar scan_angle = 10.0;  // 波束扫描中心

    print_test_info("Parameters: N=16, d/lambda=0.5, Uniform, G_peak=30dB");
    print_test_info("Scan angle: " + std::to_string(scan_angle) + " deg");
    print_test_info("Angle range: -60 to +60 deg, step=0.5 deg");

    std::vector<Scalar> angles;
    std::vector<Scalar> gain_db_vec;
    std::vector<Scalar> norm_power_vec;

    for (Scalar az = -60.0; az <= 60.0; az += 0.5) {
        angles.push_back(az);
        Scalar gain_db = model.gain_db(az, 0.0, scan_angle, 0.0);
        Scalar norm_pwr = model.normalized_power(az, 0.0, scan_angle, 0.0);
        gain_db_vec.push_back(gain_db);
        norm_power_vec.push_back(norm_pwr);
    }

    export_pattern_csv(opts.output_dir + "/ula_pattern.csv",
                       angles, gain_db_vec, norm_power_vec,
                       "ULA pattern: N=16, d/lambda=0.5, Uniform, scan=10deg");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/ula_pattern.csv" << std::endl;

    // 找到主瓣特性
    auto max_it = std::max_element(gain_db_vec.begin(), gain_db_vec.end());
    std::size_t max_idx = std::distance(gain_db_vec.begin(), max_it);
    std::cout << "\nPattern characteristics:\n";
    print_result("Peak gain", gain_db_vec[max_idx], " dB");
    print_result("Peak angle", angles[max_idx], " deg");
}

/**
 * 测试 6: UPA 二维方向图验证
 */
void test_upa_pattern(const ValidationOptions& opts) {
    print_header("Test 6: UPA 2D Pattern");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::UPA_2D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 12;
    cfg.spacing_az_lambda = 0.5;
    cfg.spacing_el_lambda = 0.5;
    cfg.peak_gain_db = 35.0;
    cfg.weight_type_az = AntennaWeightType::Hamming;
    cfg.weight_type_el = AntennaWeightType::Hamming;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    Scalar scan_az = 15.0;
    Scalar scan_el = 8.0;

    print_test_info("Parameters: Nx=16, Ny=12, d/lambda=0.5, Hamming, G_peak=35dB");
    print_test_info("Scan angle: (" + std::to_string(scan_az) + ", " + std::to_string(scan_el) + ") deg");

    std::vector<Scalar> az_angles;
    std::vector<Scalar> el_angles;
    std::vector<std::vector<Scalar>> gain_db_2d;

    // 方位角范围 -60 到 +60，步长 2 度
    for (Scalar az = -60.0; az <= 60.0; az += 2.0) {
        az_angles.push_back(az);
    }
    // 俯仰角范围 -30 到 +30，步长 2 度
    for (Scalar el = -30.0; el <= 30.0; el += 2.0) {
        el_angles.push_back(el);
    }

    // 计算二维方向图
    gain_db_2d.resize(el_angles.size(), std::vector<Scalar>(az_angles.size()));
    for (std::size_t j = 0; j < el_angles.size(); ++j) {
        for (std::size_t i = 0; i < az_angles.size(); ++i) {
            gain_db_2d[j][i] = model.gain_db(az_angles[i], el_angles[j], scan_az, scan_el);
        }
    }

    export_pattern_2d_csv(opts.output_dir + "/upa_pattern_2d.csv",
                          az_angles, el_angles, gain_db_2d,
                          "UPA 2D pattern: Nx=16, Ny=12, d/lambda=0.5, Hamming, scan=(15,8)deg");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/upa_pattern_2d.csv" << std::endl;

    // 找到峰值
    Scalar max_gain = -1000.0;
    std::size_t max_j = 0, max_i = 0;
    for (std::size_t j = 0; j < el_angles.size(); ++j) {
        for (std::size_t i = 0; i < az_angles.size(); ++i) {
            if (gain_db_2d[j][i] > max_gain) {
                max_gain = gain_db_2d[j][i];
                max_j = j;
                max_i = i;
            }
        }
    }
    std::cout << "\nPattern characteristics:\n";
    print_result("Peak gain", max_gain, " dB");
    print_result("Peak azimuth", az_angles[max_i], " deg");
    print_result("Peak elevation", el_angles[max_j], " deg");
}

/**
 * 测试 7: BeamScanner 功能验证
 */
void test_beam_scanner(const ValidationOptions& opts) {
    print_header("Test 7: BeamScanner Functionality");

    // 配置天线
    AntennaConfig ant_cfg;
    ant_cfg.model_type = PhasedArrayModelType::ULA_1D;
    ant_cfg.num_elements_az = 16;
    ant_cfg.num_elements_el = 1;
    ant_cfg.spacing_az_lambda = 0.5;
    ant_cfg.peak_gain_db = 30.0;
    ant_cfg.weight_type_az = AntennaWeightType::Uniform;

    // 配置波位表
    BeamTableConfig beam_cfg;
    beam_cfg.type = "azimuth_scan";
    beam_cfg.az_start_deg = -20.0;
    beam_cfg.az_end_deg = 20.0;
    beam_cfg.az_step_deg = 10.0;
    beam_cfg.elevation_deg = 0.0;

    BeamScanner scanner;
    scanner.set_antenna_config(ant_cfg);
    scanner.set_beam_table_config(beam_cfg);
    scanner.initialize();

    print_test_info("Beam table: [-20, -10, 0, 10, 20] deg");
    print_test_info("Total beams: " + std::to_string(scanner.beam_count()));

    Scalar target_az = 10.0;
    Scalar target_el = 0.0;

    std::cout << "\nScanning Results:\n";
    std::cout << "------------------------------------------------------------\n";
    std::cout << std::setw(8) << "Beam"
              << std::setw(12) << "Beam Az"
              << std::setw(14) << "Gain (dB)"
              << std::setw(14) << "P_norm" << std::endl;
    std::cout << "------------------------------------------------------------\n";

    std::vector<GainRecord> records;

    for (std::size_t i = 0; i < scanner.beam_count(); ++i) {
        BeamPoint bp = scanner.get_beam_pointing();
        Scalar gain_db = scanner.get_gain_db_to_target(target_az, target_el);
        Scalar norm_pwr = scanner.get_normalized_power_to_target(target_az, target_el);

        records.push_back({target_az, target_el, bp.azimuth_deg, bp.elevation_deg,
                          std::pow(10.0, gain_db/10.0), gain_db, norm_pwr});

        std::cout << std::setw(8) << i + 1
                  << std::setw(12) << bp.azimuth_deg
                  << std::setw(14) << std::fixed << std::setprecision(4) << gain_db
                  << std::setw(14) << norm_pwr << std::endl;

        scanner.advance_one_cpi();
    }
    std::cout << "------------------------------------------------------------\n";

    export_gain_records(opts.output_dir + "/beam_scanner.csv", records,
                        "BeamScanner test: ULA N=16, d/lambda=0.5, Uniform, Target=10deg");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/beam_scanner.csv" << std::endl;
}

void test_front_back_boundary(const ValidationOptions& opts) {
    print_header("Test 8: Front/Back + 360 Boundary Check");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::UPA_2D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 12;
    cfg.spacing_az_lambda = 0.5;
    cfg.spacing_el_lambda = 0.5;
    cfg.peak_gain_db = 35.0;
    cfg.weight_type_az = AntennaWeightType::Hamming;
    cfg.weight_type_el = AntennaWeightType::Hamming;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    const Scalar beam_az = 0.0;
    const Scalar beam_el = 0.0;
    const Scalar target_el = 0.0;
    const std::vector<Scalar> test_az = {0.0, 1.0, 359.0, 360.0, 180.0};

    print_test_info("Beam pointing: (0, 0) deg");
    print_test_info("Target elevation fixed at 0 deg");
    std::cout << "\nBoundary check results:\n";
    std::cout << "------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(12) << "TargetAz"
              << std::setw(16) << "P_norm"
              << std::setw(16) << "P_norm(dB)"
              << std::setw(16) << "Gain(dBi)"
              << "\n";
    std::cout << "------------------------------------------------------------\n";

    std::vector<GainRecord> records;
    records.reserve(test_az.size());
    for (const Scalar az : test_az) {
        const Scalar norm_pwr = model.normalized_power(az, target_el, beam_az, beam_el);
        const Scalar gain_db = model.gain_db(az, target_el, beam_az, beam_el);
        const Scalar norm_db = (norm_pwr > 0.0) ? (10.0 * std::log10(norm_pwr)) : -100.0;

        records.push_back({az, target_el, beam_az, beam_el,
                           std::pow(10.0, gain_db / 10.0), gain_db, norm_pwr});

        std::cout << std::setw(12) << std::fixed << std::setprecision(4) << az
                  << std::setw(16) << norm_pwr
                  << std::setw(16) << norm_db
                  << std::setw(16) << gain_db
                  << "\n";
    }
    std::cout << "------------------------------------------------------------\n";

    export_gain_records(opts.output_dir + "/front_back_boundary.csv", records,
                        "Front/back + 360 boundary check: UPA beam=(0,0), el=0, az={0,1,359,360,180}");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/front_back_boundary.csv" << std::endl;
}

void test_ula_front_back_boundary(const ValidationOptions& opts) {
    print_header("Test 9: ULA Front/Back Boundary Check");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::ULA_1D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 1;
    cfg.spacing_az_lambda = 0.5;
    cfg.spacing_el_lambda = 0.5;
    cfg.peak_gain_db = 30.0;
    cfg.weight_type_az = AntennaWeightType::Uniform;
    cfg.weight_type_el = AntennaWeightType::Uniform;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    const Scalar beam_az = 0.0;
    const Scalar beam_el = 0.0;
    const Scalar target_el = 0.0;
    const std::vector<Scalar> test_az = {0.0, 1.0, 359.0, 360.0, 180.0, -1.0};

    print_test_info("Parameters: N=16, d/lambda=0.5, Uniform, G_peak=30dB");
    print_test_info("Beam pointing: 0 deg (ULA 1D)");
    std::cout << "\nULA Boundary check results:\n";
    std::cout << "------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(12) << "TargetAz"
              << std::setw(16) << "P_norm"
              << std::setw(16) << "P_norm(dB)"
              << std::setw(16) << "Gain(dBi)"
              << "\n";
    std::cout << "------------------------------------------------------------\n";

    std::vector<GainRecord> records;
    records.reserve(test_az.size());
    for (const Scalar az : test_az) {
        const Scalar norm_pwr = model.normalized_power(az, target_el, beam_az, beam_el);
        const Scalar gain_db = model.gain_db(az, target_el, beam_az, beam_el);
        const Scalar norm_db = (norm_pwr > 0.0) ? (10.0 * std::log10(norm_pwr)) : -100.0;

        records.push_back({az, target_el, beam_az, beam_el,
                           std::pow(10.0, gain_db / 10.0), gain_db, norm_pwr});

        std::cout << std::setw(12) << std::fixed << std::setprecision(4) << az
                  << std::setw(16) << norm_pwr
                  << std::setw(16) << norm_db
                  << std::setw(16) << gain_db
                  << "\n";
    }
    std::cout << "------------------------------------------------------------\n";

    export_gain_records(opts.output_dir + "/ula_front_back_boundary.csv", records,
                        "ULA front/back boundary check: ULA beam=0, az={0,1,359,360,180,-1}");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/ula_front_back_boundary.csv" << std::endl;
}

void test_ula_full_scan_debug(const ValidationOptions& opts) {
    print_header("Test 11: ULA Full Scan Debug (Per-Beam Target Angle Log)");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::ULA_1D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 1;
    cfg.spacing_az_lambda = 0.5;
    cfg.spacing_el_lambda = 0.5;
    cfg.peak_gain_db = 30.0;
    cfg.weight_type_az = AntennaWeightType::Uniform;
    cfg.weight_type_el = AntennaWeightType::Uniform;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    // 模拟你的配置：波束从 -178 到 180，步长 2 度
    const std::vector<Scalar> beam_azs = []() {
        std::vector<Scalar> beams;
        for (Scalar az = -178.0; az <= 180.0; az += 2.0) {
            beams.push_back(az);
        }
        return beams;
    }();

    // 你的两个目标
    struct Target { int id; Scalar x, y, z; };
    const std::vector<Target> targets = {
        {1, 3000.0, 3000.0, 0.0},  // 应该在 45°
        {2, 5000.0, 0.0, 0.0},     // 应该在 0°
    };

    print_test_info("Beam scan: -178 to 180 deg, step=2 deg");
    print_test_info("Targets: T1(3000,3000,0), T2(5000,0,0)");

    std::ofstream file(opts.output_dir + "/ula_full_scan_debug.csv");
    file << std::scientific << std::setprecision(15);
    file << "beam_az_deg,target_id,target_x,target_y,target_z,"
         << "target_az_deg,target_el_deg,rel_angle_deg,cos_rel_angle,"
         << "front_factor_pass,gain_linear,gain_db\n";

    for (const Scalar beam_az : beam_azs) {
        for (const auto& tgt : targets) {
            // 计算目标角度（和 to_az_el_deg 一致）
            const Scalar target_az = std::atan2(tgt.y, tgt.x) * 180.0 / PI;
            const Scalar target_el = std::atan2(tgt.z, std::sqrt(tgt.x*tgt.x + tgt.y*tgt.y)) * 180.0 / PI;

            // ULA 归一化功率
            const Scalar norm_pwr = model.normalized_power(target_az, 0.0, beam_az, 0.0);
            const Scalar gain_lin = model.peak_gain_linear() * norm_pwr;
            const Scalar gain_db = (gain_lin > 0.0) ? (10.0 * std::log10(gain_lin)) : -100.0;

            // 相对角度和门控
            const Scalar rel_angle = target_az - beam_az;
            const Scalar cos_rel = std::cos(rel_angle * PI / 180.0);
            const bool front_pass = (cos_rel >= 1e-10);

            file << beam_az << "," << tgt.id << ","
                 << tgt.x << "," << tgt.y << "," << tgt.z << ","
                 << target_az << "," << target_el << ","
                 << rel_angle << "," << cos_rel << ","
                 << (front_pass ? "YES" : "NO") << ","
                 << gain_lin << "," << gain_db << "\n";
        }
    }
    file.close();

    // 打印关键波位的摘要
    std::cout << "\nKey beam positions summary:\n";
    std::cout << "--------------------------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(10) << "BeamAz"
              << std::setw(10) << "Target"
              << std::setw(12) << "TargetAz"
              << std::setw(12) << "RelAngle"
              << std::setw(12) << "cos(Rel)"
              << std::setw(12) << "Gate"
              << std::setw(12) << "Gain(dB)"
              << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    const std::vector<Scalar> key_beams = {43.0, 45.0, 47.0, 89.0, 90.0, 91.0, 133.0, 135.0, 137.0};
    for (const Scalar ba : key_beams) {
        for (const auto& tgt : targets) {
            const Scalar t_az = std::atan2(tgt.y, tgt.x) * 180.0 / PI;
            const Scalar rel = t_az - ba;
            const Scalar cos_rel = std::cos(rel * PI / 180.0);
            const bool gate = (cos_rel >= 1e-10);
            const Scalar np = model.normalized_power(t_az, 0.0, ba, 0.0);
            const Scalar g_db = (np > 0.0) ? (10.0 * std::log10(np) + cfg.peak_gain_db) : -100.0;

            std::cout << std::setw(10) << std::fixed << std::setprecision(2) << ba
                      << std::setw(10) << ("T" + std::to_string(tgt.id))
                      << std::setw(12) << std::setprecision(4) << t_az
                      << std::setw(12) << rel
                      << std::setw(12) << cos_rel
                      << std::setw(12) << (gate ? "YES" : "NO")
                      << std::setw(12) << g_db
                      << "\n";
        }
    }
    std::cout << "--------------------------------------------------------------------------------\n";

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/ula_full_scan_debug.csv" << std::endl;
    std::cout << "\nUse this file to check each beam's target angle computation and gating.\n";
}

void test_upa_full_scan_debug(const ValidationOptions& opts) {
    print_header("Test 12: UPA Full Scan Debug (360 Ring Scan)");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::UPA_2D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 8;
    cfg.spacing_az_lambda = 0.5;
    cfg.spacing_el_lambda = 0.5;
    cfg.peak_gain_db = 30.0;
    cfg.weight_type_az = AntennaWeightType::Hamming;
    cfg.weight_type_el = AntennaWeightType::Hamming;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    struct Target { int id; Scalar x, y, z; };
    const std::vector<Target> targets = {
        {1, 3000.0, 3000.0, 0.0},  // 45 度方向
        {2, 5000.0, 0.0, 0.0},     // 0 度方向
    };

    print_test_info("Beam scan: -178 to 180 deg, step=2 deg, el=0");
    print_test_info("Targets: T1(3000,3000,0)@45°, T2(5000,0,0)@0°");

    std::ofstream file(opts.output_dir + "/upa_full_scan_debug.csv");
    file << std::scientific << std::setprecision(15);
    file << "beam_az_deg,beam_el_deg,target_id,target_x,target_y,target_z,"
         << "target_az_deg,target_el_deg,front_factor,gate_pass,gain_db\n";

    const Scalar beam_el = 0.0;
    const Scalar target_el = 0.0;

    for (Scalar beam_az = -178.0; beam_az <= 180.0; beam_az += 2.0) {
        for (const auto& tgt : targets) {
            const Scalar t_az = std::atan2(tgt.y, tgt.x) * 180.0 / PI;
            const Scalar t_el = std::atan2(tgt.z, std::sqrt(tgt.x*tgt.x + tgt.y*tgt.y)) * 180.0 / PI;

            const Scalar front = std::cos(t_el * PI/180.0) * std::cos(t_az * PI/180.0);
            const bool gate = (front >= 0.0);
            const Scalar norm_pwr = model.normalized_power(t_az, t_el, beam_az, beam_el);
            const Scalar gain_db = (norm_pwr > 0.0) ? (10.0 * std::log10(norm_pwr) + cfg.peak_gain_db) : -100.0;

            file << beam_az << "," << beam_el << "," << tgt.id << ","
                 << tgt.x << "," << tgt.y << "," << tgt.z << ","
                 << t_az << "," << t_el << ","
                 << front << "," << (gate ? "YES" : "NO") << "," << gain_db << "\n";
        }
    }
    file.close();

    std::cout << "\nKey beam positions (UPA):\n";
    std::cout << "--------------------------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(10) << "BeamAz"
              << std::setw(10) << "Target"
              << std::setw(12) << "TargetAz"
              << std::setw(14) << "FrontFactor"
              << std::setw(10) << "Gate"
              << std::setw(12) << "Gain(dB)"
              << "\n";
    std::cout << "--------------------------------------------------------------------------------\n";

    const std::vector<Scalar> key_beams = {43.0, 45.0, 47.0, 89.0, 90.0, 91.0, 133.0, 135.0, 137.0, 178.0, 180.0};
    for (const Scalar ba : key_beams) {
        for (const auto& tgt : targets) {
            const Scalar t_az = std::atan2(tgt.y, tgt.x) * 180.0 / PI;
            const Scalar front = std::cos(0.0) * std::cos(t_az * PI/180.0);
            const bool gate = (front >= 0.0);
            const Scalar norm_pwr = model.normalized_power(t_az, 0.0, ba, 0.0);
            const Scalar g_db = (norm_pwr > 0.0) ? (10.0 * std::log10(norm_pwr) + cfg.peak_gain_db) : -100.0;

            std::cout << std::setw(10) << std::fixed << std::setprecision(1) << ba
                      << std::setw(10) << ("T" + std::to_string(tgt.id))
                      << std::setw(12) << std::setprecision(4) << t_az
                      << std::setw(14) << front
                      << std::setw(10) << (gate ? "YES" : "NO")
                      << std::setw(12) << g_db
                      << "\n";
        }
    }
    std::cout << "--------------------------------------------------------------------------------\n";

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/upa_full_scan_debug.csv" << std::endl;
}

void test_ula_relative_angle_gating(const ValidationOptions& opts) {
    print_header("Test 10: ULA Relative Angle Gating Check");

    AntennaConfig cfg;
    cfg.model_type = PhasedArrayModelType::ULA_1D;
    cfg.num_elements_az = 16;
    cfg.num_elements_el = 1;
    cfg.spacing_az_lambda = 0.5;
    cfg.spacing_el_lambda = 0.5;
    cfg.peak_gain_db = 30.0;
    cfg.weight_type_az = AntennaWeightType::Uniform;
    cfg.weight_type_el = AntennaWeightType::Uniform;

    AntennaModel model;
    model.set_config(cfg);
    model.initialize();

    // 测试场景：目标固定在 45°，波束分别指向 0°, 45°, 90°, 135°, 180°
    const Scalar target_az = 45.0;
    const std::vector<Scalar> beam_azs = {0.0, 45.0, 90.0, 135.0, 180.0};

    print_test_info("Target azimuth: 45 deg (fixed)");
    print_test_info("Beam azimuths: 0, 45, 90, 135, 180 deg");
    std::cout << "\nRelative angle gating results:\n";
    std::cout << "------------------------------------------------------------\n";
    std::cout << std::left
              << std::setw(12) << "BeamAz"
              << std::setw(12) << "RelAngle"
              << std::setw(16) << "P_norm"
              << std::setw(16) << "P_norm(dB)"
              << std::setw(16) << "Gain(dBi)"
              << "\n";
    std::cout << "------------------------------------------------------------\n";

    std::vector<GainRecord> records;
    records.reserve(beam_azs.size());
    for (const Scalar beam_az : beam_azs) {
        const Scalar rel_angle_deg = target_az - beam_az;
        const Scalar norm_pwr = model.normalized_power(target_az, 0.0, beam_az, 0.0);
        const Scalar gain_db = model.gain_db(target_az, 0.0, beam_az, 0.0);
        const Scalar norm_db = (norm_pwr > 0.0) ? (10.0 * std::log10(norm_pwr)) : -100.0;

        records.push_back({target_az, 0.0, beam_az, 0.0,
                           std::pow(10.0, gain_db / 10.0), gain_db, norm_pwr});

        std::cout << std::setw(12) << std::fixed << std::setprecision(4) << beam_az
                  << std::setw(12) << rel_angle_deg
                  << std::setw(16) << norm_pwr
                  << std::setw(16) << norm_db
                  << std::setw(16) << gain_db
                  << "\n";
    }
    std::cout << "------------------------------------------------------------\n";
    std::cout << "\nExpected behavior:\n";
    std::cout << "  - Beam 0°: rel=45°, visible (|45°| < 90°)\n";
    std::cout << "  - Beam 45°: rel=0°, peak gain\n";
    std::cout << "  - Beam 90°: rel=-45°, visible but reduced\n";
    std::cout << "  - Beam 135°: rel=-90°, at boundary (should be ~0)\n";
    std::cout << "  - Beam 180°: rel=-135°, blocked (|-135°| > 90°)\n";

    export_gain_records(opts.output_dir + "/ula_relative_angle_gating.csv", records,
                        "ULA relative angle gating: target=45deg, beam={0,45,90,135,180}deg");

    std::cout << "\n[OUTPUT] Exported: " << opts.output_dir << "/ula_relative_angle_gating.csv" << std::endl;
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char* argv[]) {
    ValidationOptions opts;

    static struct option long_options[] = {
        {"output-dir", required_argument, nullptr, 'o'},
        {"no-pattern", no_argument, nullptr, 'n'},
        {"quiet", no_argument, nullptr, 'q'},
        {"help", no_argument, nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt;
    while ((opt = getopt_long(argc, argv, "o:nqh", long_options, nullptr)) != -1) {
        switch (opt) {
        case 'o': opts.output_dir = optarg; break;
        case 'n': opts.export_pattern = false; break;
        case 'q': opts.verbose = false; break;
        case 'h':
            std::cout << "Usage: " << argv[0] << " [OPTIONS]\n"
                      << "Options:\n"
                      << "  -o, --output-dir DIR  输出目录\n"
                      << "  -n, --no-pattern      不导出方向图数据\n"
                      << "  -q, --quiet           安静模式\n"
                      << "  -h, --help            帮助\n";
            return 0;
        }
    }

    fs::create_directories(opts.output_dir);

    std::cout << "╔══════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  天线验证工具 - C++ vs MATLAB 对比           ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nOutput directory: " << opts.output_dir << std::endl;

    // 运行所有测试
    test_ula_single_beam(opts);
    test_ula_scan(opts);
    test_upa_single_beam(opts);
    test_upa_scan(opts);
    test_ula_pattern(opts);

    if (opts.export_pattern) {
        test_upa_pattern(opts);
    }

    test_beam_scanner(opts);
    test_front_back_boundary(opts);
    test_ula_front_back_boundary(opts);
    test_ula_full_scan_debug(opts);
    test_upa_full_scan_debug(opts);
    test_ula_relative_angle_gating(opts);

    // 输出汇总
    std::cout << "\n╔══════════════════════════════════════════════╗" << std::endl;
    std::cout << "║              验证完成                        ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nExported files:\n";
    std::cout << "  1. ula_single_beam.csv    - ULA 单波位增益\n";
    std::cout << "  2. ula_scan.csv           - ULA 多波位扫描\n";
    std::cout << "  3. upa_single_beam.csv    - UPA 单波位增益\n";
    std::cout << "  4. upa_scan.csv           - UPA 二维扫描\n";
    std::cout << "  5. ula_pattern.csv        - ULA 方向图\n";
    if (opts.export_pattern) {
        std::cout << "  6. upa_pattern_2d.csv     - UPA 二维方向图\n";
    }
    std::cout << "  7. beam_scanner.csv       - BeamScanner 功能\n";

    std::cout << "\n下一步:\n";
    std::cout << "  运行 MATLAB 脚本对比结果:\n";
    std::cout << "    matlab/02 天线仿真/compare_antenna_cpp_matlab.m\n";

    return 0;
}
