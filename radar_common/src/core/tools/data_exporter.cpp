/**
 * @file data_exporter.cpp
 * @brief 仿真数据导出器实现
 */

#include "core/tools/data_exporter.h"
#include <filesystem>
#include <sstream>
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>

namespace radar::core {

namespace fs = std::filesystem;

DataExporter::DataExporter(const ExportConfig& config) : config_(config) {}

void DataExporter::set_config(const ExportConfig& config) {
    config_ = config;
}

bool DataExporter::initialize() {
    if (!ensure_directory_exists(config_.output_dir)) {
        return false;
    }
    return true;
}

bool DataExporter::ensure_directory_exists(const std::string& path) {
    try {
        if (!fs::exists(path)) {
            fs::create_directories(path);
        }
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("Failed to create directory: ") + e.what();
        return false;
    }
}

bool DataExporter::export_scan(const ScanData& scan_data) {
    if (!config_.enabled) {
        return true;
    }

    bool success = true;

    // 导出配置参数（每圈都覆盖写，保证参数文件与本次运行一致）
    if (config_.export_config_params) {
        success &= export_config_params(scan_data.system_params,
                                        scan_data.target_config,
                                        scan_data.initial_targets);
    }

    // 导出 IQ 数据（二进制 dat 格式）
    if (config_.export_raw_echo_iq) {
        success &= export_echo_iq_dat(scan_data.cpi_echos, scan_data.scan_index);
    }

    return success;
}

bool DataExporter::export_config_params(const RadarSystemParams& system,
                                         const target::TargetConfig& target_config,
                                         const TargetList& initial_targets) {
    std::string filepath = config_.output_dir + "/config_params.json";

    nlohmann::json j;

    // System parameters
    j["system_params"] = {
        {"fc_hz", system.fc_hz},
        {"prf_hz", system.prf_hz},
        {"pri_s", system.pri_s},
        {"bw_hz", system.bw_hz},
        {"pulse_width_s", system.pulse_width_s},
        {"fs_hz", system.fs_hz},
        {"pulses_per_cpi", system.pulses_per_cpi},
        {"samples_per_pulse", system.samples_per_pulse},
        {"wavelength_m", system.wavelength_m},
        {"max_unambiguous_range_m", system.max_unambiguous_range_m},
        {"range_resolution_m", system.range_resolution_m}
    };

    // Target config
    j["target_config"] = {
        {"enabled", target_config.enabled},
        {"enable_beam_gain", target_config.enable_beam_gain},
        {"enable_two_way_propagation_loss", target_config.enable_two_way_propagation_loss},
        {"enable_phase", target_config.enable_phase},
        {"enable_swerling", target_config.enable_swerling},
        {"seed", target_config.seed},
        {"beam_gate_threshold_db", target_config.beam_gate_threshold_db}
    };

    // Initial targets
    j["initial_targets"] = nlohmann::json::array();
    for (const auto& t : initial_targets) {
        nlohmann::json target_json;
        target_json["id"] = t.id;
        target_json["position_m"] = {t.position_m.x(), t.position_m.y(), t.position_m.z()};
        target_json["velocity_mps"] = {t.velocity_mps.x(), t.velocity_mps.y(), t.velocity_mps.z()};
        target_json["acceleration_mps2"] = {t.acceleration_mps2.x(), t.acceleration_mps2.y(), t.acceleration_mps2.z()};
        target_json["rcs_mean_m2"] = t.rcs_mean_m2;
        target_json["swerling_type"] = static_cast<int>(t.swerling);
        target_json["enabled"] = t.enabled;
        j["initial_targets"].push_back(target_json);
    }

    try {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            last_error_ = "Failed to open file: " + filepath;
            return false;
        }
        file << j.dump(2);
        file.close();
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("Failed to write JSON: ") + e.what();
        return false;
    }
}

bool DataExporter::export_echo_iq_dat(const std::vector<CpiEcho>& cpi_echos,
                                       int scan_index) {
    std::string filepath = config_.output_dir + "/echo_iq_scan_" + std::to_string(scan_index) + ".dat";

    try {
        std::ofstream file(filepath, std::ios::binary);
        if (!file.is_open()) {
            last_error_ = "Failed to open file: " + filepath;
            return false;
        }

        // 文件头：[magic(4)] [version(4)] [scan_index(4)] [cpi_count(4)]
        //        [pulses_per_cpi(4)] [samples_per_pulse(4)] [reserved(8)]
        const uint32_t magic = 0x4543484F;  // "ECHO"
        const uint32_t version = 1;
        const uint32_t cpi_count = static_cast<uint32_t>(cpi_echos.size());
        const uint32_t pulses_per_cpi = cpi_echos.empty() ? 0 : static_cast<uint32_t>(cpi_echos[0].pulses.size());
        const uint32_t samples_per_pulse = cpi_echos.empty() || cpi_echos[0].pulses.empty() ? 0 : static_cast<uint32_t>(cpi_echos[0].pulses[0].size());
        const uint64_t reserved = 0;

        file.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
        file.write(reinterpret_cast<const char*>(&version), sizeof(version));
        file.write(reinterpret_cast<const char*>(&scan_index), sizeof(scan_index));
        file.write(reinterpret_cast<const char*>(&cpi_count), sizeof(cpi_count));
        file.write(reinterpret_cast<const char*>(&pulses_per_cpi), sizeof(pulses_per_cpi));
        file.write(reinterpret_cast<const char*>(&samples_per_pulse), sizeof(samples_per_pulse));
        file.write(reinterpret_cast<const char*>(&reserved), sizeof(reserved));

        // 写入每个 CPI 的 IQ 数据 (float32 复数：real, imag 交替)
        for (const auto& cpi : cpi_echos) {
            for (const auto& pulse : cpi.pulses) {
                for (const auto& sample : pulse) {
                    float real = static_cast<float>(sample.real());
                    float imag = static_cast<float>(sample.imag());
                    file.write(reinterpret_cast<const char*>(&real), sizeof(real));
                    file.write(reinterpret_cast<const char*>(&imag), sizeof(imag));
                }
            }
        }

        file.close();
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("Failed to write DAT file: ") + e.what();
        return false;
    }
}

bool DataExporter::write_csv_file(const std::string& filepath,
                                   const std::vector<std::string>& headers,
                                   const std::vector<std::vector<std::string>>& rows) {
    try {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            last_error_ = "Failed to open file: " + filepath;
            return false;
        }

        // 写入表头
        for (std::size_t i = 0; i < headers.size(); ++i) {
            file << headers[i];
            if (i < headers.size() - 1) file << ",";
        }
        file << "\n";

        // 写入数据行
        for (const auto& row : rows) {
            for (std::size_t i = 0; i < row.size(); ++i) {
                file << row[i];
                if (i < row.size() - 1) file << ",";
            }
            file << "\n";
        }

        file.close();
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("Failed to write CSV: ") + e.what();
        return false;
    }
}



// 导出复数数据为CSV
bool DataExporter::export_complex_csv(const std::string& filepath,
                        const ComplexVec& data,
                        const std::string& var_name) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        last_error_ = "Failed to open file: " + filepath;
        return false;
    }

    file << std::scientific << std::setprecision(15);
    file << "index,real,imag\n";

    for (std::size_t i = 0; i < data.size(); ++i) {
        file << i << "," << data[i].real() << "," << data[i].imag() << "\n";
    }

    return true;
}

// 导出实数数据为CSV
bool DataExporter::export_scalar_csv(const std::string& filepath,
                       const std::vector<Scalar>& data,
                       const std::string& var_name) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        last_error_ = "Failed to open file: " + filepath;
        return false;
    }

    file << std::scientific << std::setprecision(15);
    file << "index,value\n";

    for (std::size_t i = 0; i < data.size(); ++i) {
        file << i << "," << data[i] << "\n";
    }

    return true;
}
}  // namespace radar::core
