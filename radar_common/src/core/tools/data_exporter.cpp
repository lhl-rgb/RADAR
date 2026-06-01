/**
 * @file data_exporter.cpp
 * @brief 仿真数据导出器实现
 */

#include "core/tools/data_exporter.h"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <sstream>

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

    if (config_.export_config_params) {
        success &= export_config_params(scan_data.system_params,
                                        scan_data.target_config,
                                        scan_data.mech_scan_config,
                                        scan_data.initial_targets);
    }

    if (config_.export_raw_echo_iq) {
        success &= export_echo_iq_dat(scan_data.scan_echo);
        success &= export_pulse_metadata_csv(scan_data.scan_echo);
    }

    return success;
}

bool DataExporter::export_config_params(const RadarSystemParams& system,
                                        const target::TargetConfig& target_config,
                                        const TargetList& initial_targets) {
    antenna::MechanicalScanConfig mech_scan_config;
    mech_scan_config.compute_derived_params(system.pri_s);
    return export_config_params(system, target_config, mech_scan_config, initial_targets);
}

bool DataExporter::export_config_params(const RadarSystemParams& system,
                                        const target::TargetConfig& target_config,
                                        const antenna::MechanicalScanConfig& mech_scan_config,
                                        const TargetList& initial_targets) {
    std::string filepath = config_.output_dir + "/config_params.json";

    nlohmann::json j;
    j["system_params"] = {
        {"fc_hz", system.fc_hz},
        {"prf_hz", system.prf_hz},
        {"pri_s", system.pri_s},
        {"bw_hz", system.bw_hz},
        {"pulse_width_s", system.pulse_width_s},
        {"peak_power_w", system.peak_power_w},
        {"fs_hz", system.fs_hz},
        {"min_range_m", system.min_range_m},
        {"max_range_m", system.max_range_m},
        {"antenna_height_m", system.antenna_height_m},
        {"noise_figure_db", system.noise_figure_db},
        {"system_loss_db", system.system_loss_db},
        {"samples_per_pulse", system.samples_per_pulse},
        {"samples_per_tx", system.samples_per_tx},
        {"wavelength_m", system.wavelength_m},
        {"range_resolution_m", system.range_resolution_m},
        {"range_bin_size_m", system.range_bin_size_m},
        {"max_unambiguous_range_m", system.max_unambiguous_range_m},
        {"max_unambiguous_velocity_mps", system.max_unambiguous_velocity_mps},
        {"max_doppler_hz", system.max_doppler_hz}
    };

    j["radar_location"] = {
        {"latitude_deg", system.radar_location.latitude},
        {"longitude_deg", system.radar_location.longitude},
        {"altitude_m", system.radar_location.altitude}
    };

    j["mechanical_scan"] = {
        {"rotation_rate_dps", mech_scan_config.rotation_rate_dps},
        {"az_start_deg", mech_scan_config.az_start_deg},
        {"az_end_deg", mech_scan_config.az_end_deg},
        {"elevation_deg", mech_scan_config.elevation_deg},
        {"rotation_period_s", mech_scan_config.rotation_period_s},
        {"azimuth_step_per_prt_deg", mech_scan_config.azimuth_step_per_prt_deg},
        {"pulses_per_rotation", mech_scan_config.pulses_per_rotation}
    };

    j["target_config"] = {
        {"enabled", target_config.enabled},
        {"enable_beam_gain", target_config.enable_beam_gain},
        {"enable_two_way_propagation_loss", target_config.enable_two_way_propagation_loss},
        {"enable_phase", target_config.enable_phase},
        {"enable_swerling", target_config.enable_swerling},
        {"seed", target_config.seed},
        {"beam_gate_threshold_db", target_config.beam_gate_threshold_db}
    };

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

bool DataExporter::export_echo_iq_dat(const ScanEcho& scan_echo) {
    const std::string filepath = config_.output_dir + "/echo_iq_scan_" + std::to_string(scan_echo.scan_index) + ".dat";

    try {
        std::ofstream file(filepath, std::ios::binary);
        if (!file.is_open()) {
            last_error_ = "Failed to open file: " + filepath;
            return false;
        }

        const uint32_t magic = 0x4543484F;
        const uint32_t version = 2;
        const uint32_t scan_index = static_cast<uint32_t>(scan_echo.scan_index);
        const uint32_t pulse_count = static_cast<uint32_t>(scan_echo.pulse_results.size());
        const uint32_t samples_per_pulse =
            scan_echo.pulse_results.empty() ? 0U : static_cast<uint32_t>(scan_echo.pulse_results.front().echo.size());
        const uint64_t reserved = 0;

        file.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
        file.write(reinterpret_cast<const char*>(&version), sizeof(version));
        file.write(reinterpret_cast<const char*>(&scan_index), sizeof(scan_index));
        file.write(reinterpret_cast<const char*>(&pulse_count), sizeof(pulse_count));
        file.write(reinterpret_cast<const char*>(&samples_per_pulse), sizeof(samples_per_pulse));
        file.write(reinterpret_cast<const char*>(&reserved), sizeof(reserved));

        for (const auto& pulse_result : scan_echo.pulse_results) {
            for (const auto& sample : pulse_result.echo) {
                float real = static_cast<float>(sample.real());
                float imag = static_cast<float>(sample.imag());
                file.write(reinterpret_cast<const char*>(&real), sizeof(real));
                file.write(reinterpret_cast<const char*>(&imag), sizeof(imag));
            }
        }

        file.close();
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("Failed to write DAT file: ") + e.what();
        return false;
    }
}

bool DataExporter::export_pulse_metadata_csv(const ScanEcho& scan_echo) {
    const std::string filepath = config_.output_dir + "/echo_iq_scan_" + std::to_string(scan_echo.scan_index) + "_metadata.csv";
    std::vector<std::vector<std::string>> rows;
    rows.reserve(scan_echo.pulse_results.size());

    for (const auto& pulse_result : scan_echo.pulse_results) {
        rows.push_back({
            std::to_string(pulse_result.pulse_index),
            std::to_string(pulse_result.azimuth_deg),
            std::to_string(pulse_result.elevation_deg)
        });
    }

    return write_csv_file(filepath, {"pulse_index", "azimuth_deg", "elevation_deg"}, rows);
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

        for (std::size_t i = 0; i < headers.size(); ++i) {
            file << headers[i];
            if (i < headers.size() - 1) file << ",";
        }
        file << "\n";

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

bool DataExporter::export_complex_csv(const std::string& filepath,
                                      const ComplexVec& data,
                                      const std::string& var_name) {
    (void)var_name;
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

bool DataExporter::export_scalar_csv(const std::string& filepath,
                                     const std::vector<Scalar>& data,
                                     const std::string& var_name) {
    (void)var_name;
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
