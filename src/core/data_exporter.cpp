/**
 * @file data_exporter.cpp
 * @brief 仿真数据导出器实现
 */

#include "core/data_exporter.h"
#include <filesystem>
#include <sstream>
#include <cmath>
#include <cstdint>

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

    // 导出配置参数（只在第一圈导出）
    if (config_.export_config_params && scan_data.scan_index == 0) {
        success &= export_config_params(scan_data.system_params,
                                        scan_data.target_config,
                                        scan_data.initial_targets);
    }

    // 导出 IQ 数据（二进制 dat 格式）
    if (config_.export_raw_echo_iq) {
        success &= export_echo_iq_dat(scan_data.cpi_echos, scan_data.scan_index);
    }

    // 导出目标轨迹
    if (config_.export_target_snapshots && !scan_data.target_snapshots_per_cpi.empty()) {
        int pulses_per_cpi = scan_data.system_params.pulses_per_cpi;
        Scalar pri_s = scan_data.system_params.pri_s;
        success &= export_target_trajectory(scan_data.target_snapshots_per_cpi,
                                            pulses_per_cpi, pri_s);
    }

    return success;
}

bool DataExporter::export_config_params(const RadarSystemParams& system,
                                         const target::TargetConfig& target_config,
                                         const TargetList& initial_targets) {
    std::string filepath = config_.output_dir + "/config_params.json";

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "{\n";
    oss << "  \"system_params\": {\n";
    oss << "    \"fc_hz\": " << system.fc_hz << ",\n";
    oss << "    \"prf_hz\": " << system.prf_hz << ",\n";
    oss << "    \"pri_s\": " << system.pri_s << ",\n";
    oss << "    \"bw_hz\": " << system.bw_hz << ",\n";
    oss << "    \"pulse_width_s\": " << system.pulse_width_s << ",\n";
    oss << "    \"fs_hz\": " << system.fs_hz << ",\n";
    oss << "    \"pulses_per_cpi\": " << system.pulses_per_cpi << ",\n";
    oss << "    \"samples_per_pulse\": " << system.samples_per_pulse << ",\n";
    oss << "    \"wavelength_m\": " << system.wavelength_m << ",\n";
    oss << "    \"max_unambiguous_range_m\": " << system.max_unambiguous_range_m << ",\n";
    oss << "    \"range_resolution_m\": " << system.range_resolution_m << "\n";
    oss << "  },\n";

    oss << "  \"target_config\": {\n";
    oss << "    \"enabled\": " << (target_config.enabled ? "true" : "false") << ",\n";
    oss << "    \"enable_beam_gain\": " << (target_config.enable_beam_gain ? "true" : "false") << ",\n";
    oss << "    \"enable_two_way_propagation_loss\": " << (target_config.enable_two_way_propagation_loss ? "true" : "false") << ",\n";
    oss << "    \"enable_phase\": " << (target_config.enable_phase ? "true" : "false") << ",\n";
    oss << "    \"enable_swerling\": " << (target_config.enable_swerling ? "true" : "false") << ",\n";
    oss << "    \"seed\": " << target_config.seed << ",\n";
    oss << "    \"beam_gate_threshold_db\": " << target_config.beam_gate_threshold_db << "\n";
    oss << "  },\n";

    oss << "  \"initial_targets\": [\n";
    for (std::size_t i = 0; i < initial_targets.size(); ++i) {
        const auto& t = initial_targets[i];
        oss << "    {\n";
        oss << "      \"id\": " << t.id << ",\n";
        oss << "      \"position_m\": [" << t.position_m.x() << ", " << t.position_m.y() << ", " << t.position_m.z() << "],\n";
        oss << "      \"velocity_mps\": [" << t.velocity_mps.x() << ", " << t.velocity_mps.y() << ", " << t.velocity_mps.z() << "],\n";
        oss << "      \"acceleration_mps2\": [" << t.acceleration_mps2.x() << ", " << t.acceleration_mps2.y() << ", " << t.acceleration_mps2.z() << "],\n";
        oss << "      \"rcs_mean_m2\": " << t.rcs_mean_m2 << ",\n";
        oss << "      \"swerling_type\": " << static_cast<int>(t.swerling) << ",\n";
        oss << "      \"enabled\": " << (t.enabled ? "true" : "false") << "\n";
        oss << "    }";
        if (i < initial_targets.size() - 1) oss << ",";
        oss << "\n";
    }
    oss << "  ]\n";
    oss << "}\n";

    return write_json_file(filepath, oss.str());
}

bool DataExporter::export_target_trajectory(const std::vector<TargetList>& all_snapshots,
                                             int pulses_per_cpi,
                                             Scalar pri_s) {
    std::string filepath = config_.output_dir + "/target_trajectory.csv";

    std::vector<std::string> headers = {
        "cpi_index", "pulse_index", "slow_time_s",
        "target_id", "range_m", "radial_velocity_mps", "radial_acceleration_mps2",
        "azimuth_deg", "elevation_deg", "rcs_m2"
    };

    std::vector<std::vector<std::string>> rows;
    Scalar total_time = 0.0;

    for (std::size_t cpi_idx = 0; cpi_idx < all_snapshots.size(); ++cpi_idx) {
        const auto& targets = all_snapshots[cpi_idx];
        for (int pulse_idx = 0; pulse_idx < pulses_per_cpi; ++pulse_idx) {
            Scalar slow_time = total_time + pulse_idx * pri_s;

            for (const auto& target : targets) {
                if (!target.enabled) continue;

                // 计算径向速度和距离
                Vec3 pos = target.position_m;
                Vec3 vel = target.velocity_mps;

                Scalar range = pos.norm();
                Vec3 los = pos / range;  // 单位视线向量
                Scalar radial_vel = los.dot(vel);

                // 简化计算径向加速度
                Scalar radial_acc = 0.0;

                // 计算方位角和俯仰角
                Scalar az = std::atan2(pos.y(), pos.x()) * 180.0 / PI;
                Scalar el = std::asin(pos.z() / range) * 180.0 / PI;

                std::vector<std::string> row;
                row.push_back(std::to_string(cpi_idx));
                row.push_back(std::to_string(pulse_idx));
                row.push_back(std::to_string(slow_time));
                row.push_back(std::to_string(target.id));
                row.push_back(std::to_string(range));
                row.push_back(std::to_string(radial_vel));
                row.push_back(std::to_string(radial_acc));
                row.push_back(std::to_string(az));
                row.push_back(std::to_string(el));
                row.push_back(std::to_string(target.rcs_mean_m2));

                rows.push_back(row);
            }
        }
        total_time += pulses_per_cpi * pri_s;
    }

    return write_csv_file(filepath, headers, rows);
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

bool DataExporter::write_json_file(const std::string& filepath, const std::string& content) {
    try {
        std::ofstream file(filepath);
        if (!file.is_open()) {
            last_error_ = "Failed to open file: " + filepath;
            return false;
        }
        file << content;
        file.close();
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("Failed to write JSON: ") + e.what();
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

}  // namespace radar::core
