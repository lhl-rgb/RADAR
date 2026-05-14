/**
 * @file radar_config.h
 * @brief 雷达仿真系统配置组合容器
 * @details 组合所有模块配置，作为配置入口（服务器端模式）
 */

#pragma once

#include <spdlog/spdlog.h>

#include "antenna/antenna_config.h"
#include "antenna/beam_scanner.h"
#include "clutter/sea_clutter_config.h"
#include "core/radar_system_params.h"
#include "noise/noise_config.h"
#include "target/target_config.h"
#include "waveform/waveform_config.h"
#include "core/tools/data_exporter.h"
#include "core/tools/udp_config.h"
#include "core/tools/udp_sender.h"

#include <string>
#include <vector>

namespace radar {

/**
 * @brief 仿真流程控制配置
 * @details 这些参数控制仿真运行流程，不属于雷达物理系统参数。
 */
struct SimulationConfig {
    int scan_count = 2; ///< 仿真扫描圈数。

    bool validate(std::string& error) const {
        if (scan_count <= 0) {
            error = "simulation.scan_count must be positive";
            return false;
        }
        return true;
    }

    void print() const {
        SPDLOG_INFO("--- SimulationConfig ---");
        SPDLOG_INFO("  scan_count: {}", scan_count);
    }
};

inline void from_json(const nlohmann::json& j, SimulationConfig& cfg) {
    if (j.contains("scan_count")) {
        j.at("scan_count").get_to(cfg.scan_count);
    }
}

inline void to_json(nlohmann::json& j, const SimulationConfig& cfg) {
    j = nlohmann::json{
        {"scan_count", cfg.scan_count}
    };
}



/**
 * @brief 雷达仿真系统配置组合容器
 * @details
 * 包含所有模块配置，用于一次性加载和管理配置。
 * 服务器端模式：所有配置通过 JSON 文件加载，不在此代码中硬编码。
 */
struct RadarConfig {
    SimulationConfig simulation;        ///< 仿真流程控制配置
    RadarSystemParams system;          ///< 全局共享参数
    waveform::WaveformConfig waveform; ///< 波形配置
    antenna::AntennaConfig antenna;    ///< 天线配置（只含物理配置）
    antenna::BeamTableConfig beam_table;        ///< 波位表配置（独立于天线配置）
    noise::NoiseConfig noise;          ///< 噪声配置
    clutter::SeaClutterConfig clutter; ///< 海杂波配置
    target::TargetConfig target;       ///< 目标配置
    core::ExportConfig data_export;    ///< 数据导出配置
    UdpConfig udp_output;              ///< UDP 输出配置
    TargetList initial_targets;        ///< 初始目标列表

    RadarConfig();

    /**
     * @brief 计算派生参数
     * @details 调用 system.compute_derived_params()
     */
    void compute_derived_params();

    /**
     * @brief 验证所有配置
     * @param error 错误信息输出
     * @return 全部有效返回 true
     */
    bool validate(std::string& error) const;

    /**
     * @brief 验证跨模块一致性
     * @param error 错误信息输出
     * @return 一致返回 true
     */
    bool validate_cross_module(std::string& error) const;

    /**
     * @brief 同步 UDP 配置到 system 参数
     * @deprecated UDP 配置已独立，不再需要同步
     */
    void sync_udp_config();

    /**
     * @brief 打印配置摘要
     */
    void print() const;
};

// JSON serialization declarations (implemented in configuration_manager.cpp)
void from_json(const nlohmann::json& j, RadarConfig& cfg);
void to_json(nlohmann::json& j, const RadarConfig& cfg);

inline RadarConfig::RadarConfig() {
    compute_derived_params();
}

inline void RadarConfig::compute_derived_params() {
    system.compute_derived_params();
}

inline bool RadarConfig::validate(std::string& error) const {
    // Validate each module config
    if (!simulation.validate(error)) {
        return false;
    }
    if (!system.validate(error)) {
        return false;
    }
    if (!waveform.validate(error)) {
        return false;
    }
    if (!antenna.validate(error)) {
        return false;
    }
    if (!noise.validate(error)) {
        return false;
    }
    if (!clutter.validate(error)) {
        return false;
    }
    if (!target.validate(error)) {
        return false;
    }

    // Validate data export config
    if (data_export.enabled && data_export.output_dir.empty()) {
        error = "data_export.output_dir cannot be empty when export is enabled";
        return false;
    }

    // Validate UDP output config
    if (udp_output.enabled && !udp_output.validate(error)) {
        error = "UDP output configuration invalid: " + error;
        return false;
    }

    // Cross-module validation
    return validate_cross_module(error);
}

inline bool RadarConfig::validate_cross_module(std::string& error) const {
    // Pulse width < PRI
    if (system.pulse_width_s > system.pri_s) {
        error = "pulse_width exceeds PRI";
        return false;
    }

    // Sampling rate >= bandwidth
    if (system.fs_hz < system.bw_hz) {
        error = "fs must be >= bw for complex baseband modeling";
        return false;
    }

    // Clutter doppler center within unambiguous range
    const Scalar clutter_nyquist_hz = 0.5 * system.prf_hz;
    if (clutter.doppler_center_hz < -clutter_nyquist_hz ||
        clutter.doppler_center_hz >= clutter_nyquist_hz) {
        error = "clutter doppler_center must be within [-prf/2, prf/2)";
        return false;
    }

    // Clutter range within radar range
    const Scalar clutter_range_min =
        (clutter.ground_range_min_m < 0.0) ? system.min_range_m : clutter.ground_range_min_m;
    const Scalar clutter_range_max =
        (clutter.ground_range_max_m < 0.0) ? system.max_range_m : clutter.ground_range_max_m;
    if (clutter_range_min < 0.0 || clutter_range_max <= clutter_range_min) {
        error = "clutter range invalid (min >= 0, max > min)";
        return false;
    }

    return true;
}

inline void RadarConfig::sync_udp_config() {
    // Deprecated: UDP 配置已独立，不再需要同步
}

inline void RadarConfig::print() const {
    SPDLOG_INFO("=== Radar Simulation Config ===");
    SPDLOG_INFO("");
    simulation.print();
    SPDLOG_INFO("");
    system.print();
    SPDLOG_INFO("");
    SPDLOG_INFO("--- WaveformConfig ---");
    SPDLOG_INFO("  waveform_type: {}", static_cast<int>(waveform.waveform_type));
    SPDLOG_INFO("  phase_code_type: {}", static_cast<int>(waveform.phase_code_type));
    SPDLOG_INFO("  nlfm_window_type: {}", static_cast<int>(waveform.nlfm_window_type));
    SPDLOG_INFO("  polarization: {}", static_cast<int>(waveform.polarization));
    SPDLOG_INFO("");
    SPDLOG_INFO("--- AntennaConfig ---");
    SPDLOG_INFO("  model_type: {}", static_cast<int>(antenna.model_type));
    SPDLOG_INFO("  num_elements_az: {}", antenna.num_elements_az);
    SPDLOG_INFO("  num_elements_el: {}", antenna.num_elements_el);
    SPDLOG_INFO("  peak_gain_db: {}", antenna.peak_gain_db);
    SPDLOG_INFO("");
    SPDLOG_INFO("--- BeamTableConfig ---");
    SPDLOG_INFO("  type: {}", this->beam_table.type);
    SPDLOG_INFO("  az_start_deg: {}", this->beam_table.az_start_deg);
    SPDLOG_INFO("  az_end_deg: {}", this->beam_table.az_end_deg);
    SPDLOG_INFO("  az_step_deg: {}", this->beam_table.az_step_deg);
    SPDLOG_INFO("  elevation_deg: {}", this->beam_table.elevation_deg);
    SPDLOG_INFO("  beam_count: {}", this->beam_table.beams.size());
    SPDLOG_INFO("");
    SPDLOG_INFO("--- NoiseConfig ---");
    SPDLOG_INFO("  mode: {}", static_cast<int>(noise.mode));
    SPDLOG_INFO("  sigma_complex: {}", noise.sigma_complex);
    SPDLOG_INFO("  seed: {}", noise.seed);
    SPDLOG_INFO("");
    SPDLOG_INFO("--- SeaClutterConfig ---");
    SPDLOG_INFO("  enabled: {}", clutter.enabled ? "true" : "false");
    SPDLOG_INFO("  k_shape_nu: {}", clutter.k_shape_nu);
    SPDLOG_INFO("  morchin_sea_state: {}", clutter.morchin_sea_state);
    SPDLOG_INFO("");
    SPDLOG_INFO("--- TargetConfig ---");
    SPDLOG_INFO("  enabled: {}", target.enabled ? "true" : "false");
    SPDLOG_INFO("  enable_swerling: {}", target.enable_swerling ? "true" : "false");
    SPDLOG_INFO("  seed: {}", target.seed);
    SPDLOG_INFO("  enable_beam_gain: {}", target.enable_beam_gain ? "true" : "false");
    SPDLOG_INFO("  beam_gate_threshold_db: {}", target.beam_gate_threshold_db);
    SPDLOG_INFO("");
    SPDLOG_INFO("--- Data Export Config ---");
    SPDLOG_INFO("  enabled: {}", data_export.enabled ? "true" : "false");
    SPDLOG_INFO("  output_dir: {}", data_export.output_dir);
    SPDLOG_INFO("  export_raw_echo_iq: {}", data_export.export_raw_echo_iq ? "true" : "false");
    SPDLOG_INFO("  export_target_snapshots: {}", data_export.export_target_snapshots ? "true" : "false");
    SPDLOG_INFO("  export_config_params: {}", data_export.export_config_params ? "true" : "false");
    SPDLOG_INFO("");
    SPDLOG_INFO("--- UDP Output Config ---");
    SPDLOG_INFO("  enabled: {}", udp_output.enabled ? "true" : "false");
    SPDLOG_INFO("  target_ip: {}", udp_output.target_ip);
    SPDLOG_INFO("  target_port: {}", udp_output.target_port);
}

}  // namespace radar
