/**
 * @file data_exporter.h
 * @brief 仿真数据导出器
 * @details
 * 负责将每圈（Scan）的仿真数据导出到文件，供 MATLAB 验证脚本使用。
 */

#pragma once

#include "core/radar_system_params.h"
#include "core/types.h"
#include "antenna/antenna_config.h"
#include "target/target_config.h"

#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace radar::core {

struct ExportConfig {
    bool enabled = true;
    std::string output_dir = "output";
    bool export_raw_echo_iq = true;
    bool export_target_snapshots = false;
    bool export_config_params = true;
};

struct ScanData {
    ScanEcho scan_echo;
    RadarSystemParams system_params;
    target::TargetConfig target_config;
    antenna::MechanicalScanConfig mech_scan_config;
    TargetList initial_targets;
};

class DataExporter {
public:
    DataExporter() = default;
    explicit DataExporter(const ExportConfig& config);

    void set_config(const ExportConfig& config);
    bool initialize();
    bool export_scan(const ScanData& scan_data);

    bool export_config_params(const RadarSystemParams& system,
                              const target::TargetConfig& target_config,
                              const TargetList& initial_targets);
    bool export_config_params(const RadarSystemParams& system,
                              const target::TargetConfig& target_config,
                              const antenna::MechanicalScanConfig& mech_scan_config,
                              const TargetList& initial_targets);

    bool export_echo_iq_dat(const ScanEcho& scan_echo);
    bool export_pulse_metadata_csv(const ScanEcho& scan_echo);

    bool export_complex_csv(const std::string& filepath,
                            const ComplexVec& data,
                            const std::string& var_name = "signal");

    bool export_scalar_csv(const std::string& filepath,
                           const std::vector<Scalar>& data,
                           const std::string& var_name = "data");

    const std::string& last_error() const { return last_error_; }

private:
    ExportConfig config_;
    std::string last_error_;

    bool ensure_directory_exists(const std::string& path);
    bool write_csv_file(const std::string& filepath,
                        const std::vector<std::string>& headers,
                        const std::vector<std::vector<std::string>>& rows);
};

}  // namespace radar::core
