/**
 * @file radar_config.h
 * @brief 雷达仿真系统配置组合容器
 * @details 组合所有模块配置，作为配置入口（服务器端模式）
 */

#pragma once

#include <spdlog/spdlog.h>

#include "antenna/antenna_config.h"
#include "clutter/sea_clutter_config.h"
#include "core/radar_system_params.h"
#include "noise/noise_config.h"
#include "target/target_config.h"
#include "waveform/waveform_config.h"
#include "core/tools/data_exporter.h"
#include "core/tools/udp_config.h"
#include "core/tools/udp_sender.h"
#include "output/output_config.h"
#include "preview/preview_config.h"

#include <string>
#include <vector>

namespace radar
{

    /**
     * @brief 仿真流程控制配置
     */
    struct SimulationConfig
    {
        int scan_count = 2;

        bool validate(std::string &error) const
        {
            if (scan_count <= 0)
            {
                error = "simulation.scan_count must be positive";
                return false;
            }
            return true;
        }

        void print() const
        {
            SPDLOG_INFO("--- SimulationConfig ---");
            SPDLOG_INFO("  scan_count: {}", scan_count);
        }
    };

    /**
     * @brief 雷达仿真系统配置组合容器
     */
    struct RadarConfig
    {
        SimulationConfig simulation;
        RadarSystemParams system;
        waveform::WaveformConfig waveform;
        antenna::AntennaConfig antenna;
        antenna::MechanicalScanConfig mech_scan;
        noise::NoiseConfig noise;
        clutter::SeaClutterConfig clutter;
        target::TargetConfig target;
        core::ExportConfig data_export;
        output::OutputConfig output;
        preview::PreviewConfig preview;
        UdpConfig udp_output;
        TargetList initial_targets;

        RadarConfig();

        void compute_derived_params();
        bool validate(std::string &error) const;
        bool validate_cross_module(std::string &error) const;
        void print() const;
    };

    inline RadarConfig::RadarConfig()
    {
        compute_derived_params();
    }

    inline void RadarConfig::compute_derived_params()
    {
        system.compute_derived_params();
        mech_scan.compute_derived_params(system.pri_s);
    }

    inline bool RadarConfig::validate(std::string &error) const
    {
        if (!simulation.validate(error))
            return false;
        if (!system.validate(error))
            return false;
        if (!waveform.validate(error))
            return false;
        if (!antenna.validate(error))
            return false;
        if (!mech_scan.validate(error))
            return false;
        if (!noise.validate(error))
            return false;
        if (!clutter.validate(error))
            return false;
        if (!target.validate(error))
            return false;

        if (data_export.enabled && data_export.output_dir.empty())
        {
            error = "data_export.output_dir cannot be empty when export is enabled";
            return false;
        }

        if (!output.validate(error))
        {
            error = "Output configuration invalid: " + error;
            return false;
        }

        if (!preview.validate(error))
        {
            error = "Preview configuration invalid: " + error;
            return false;
        }

        if (udp_output.enabled && !udp_output.validate(error))
        {
            error = "UDP output configuration invalid: " + error;
            return false;
        }

        return validate_cross_module(error);
    }

    inline bool RadarConfig::validate_cross_module(std::string &error) const
    {
        if (system.pulse_width_s > system.pri_s)
        {
            error = "pulse_width exceeds PRI";
            return false;
        }
        if (system.fs_hz < system.bw_hz)
        {
            error = "fs must be >= bw for complex baseband modeling";
            return false;
        }
        return true;
    }

    inline void RadarConfig::print() const
    {
        SPDLOG_INFO("=== Radar Simulation Config ===");
        SPDLOG_INFO("");
        simulation.print();
        SPDLOG_INFO("");
        system.print();
        SPDLOG_INFO("");
        SPDLOG_INFO("--- WaveformConfig ---");
        SPDLOG_INFO("  waveform_type: {}", static_cast<int>(waveform.waveform_type));
        SPDLOG_INFO("");
        SPDLOG_INFO("--- AntennaConfig ---");
        SPDLOG_INFO("  az_beamwidth_deg: {}", antenna.az_beamwidth_deg);
        SPDLOG_INFO("  el_beamwidth_deg: {}", antenna.el_beamwidth_deg);
        SPDLOG_INFO("  peak_gain_db: {}", antenna.peak_gain_db);
        SPDLOG_INFO("");
        SPDLOG_INFO("--- MechanicalScanConfig ---");
        SPDLOG_INFO("  rotation_rate_dps: {}", mech_scan.rotation_rate_dps);
        SPDLOG_INFO("  az_start_deg: {}", mech_scan.az_start_deg);
        SPDLOG_INFO("  az_end_deg: {}", mech_scan.az_end_deg);
        SPDLOG_INFO("  elevation_deg: {}", mech_scan.elevation_deg);
        SPDLOG_INFO("  rotation_period_s: {}", mech_scan.rotation_period_s);
        SPDLOG_INFO("  azimuth_step_per_prt_deg: {}", mech_scan.azimuth_step_per_prt_deg);
        SPDLOG_INFO("  pulses_per_rotation: {}", mech_scan.pulses_per_rotation);
        SPDLOG_INFO("");
        SPDLOG_INFO("--- NoiseConfig ---");
        SPDLOG_INFO("  mode: {}", static_cast<int>(noise.mode));
        SPDLOG_INFO("");
        SPDLOG_INFO("--- SeaClutterConfig ---");
        SPDLOG_INFO("  enabled: {}", clutter.enabled ? "true" : "false");
        SPDLOG_INFO("  backend: {}", static_cast<int>(clutter.backend));
        SPDLOG_INFO("  distribution: {}", static_cast<int>(clutter.distribution));
        SPDLOG_INFO("  az_patch_step_deg: {}", clutter.az_patch_step_deg);
        SPDLOG_INFO("  active_gain_floor_db: {}", clutter.active_gain_floor_db);
        SPDLOG_INFO("  sea_state: {}", clutter.sea_state);
        SPDLOG_INFO("  doppler_sigma_hz: {}", clutter.doppler_sigma_hz);
        SPDLOG_INFO("  weibull_shape: {}", clutter.weibull_shape);
        SPDLOG_INFO("  weibull_scale: {}", clutter.weibull_scale);
        SPDLOG_INFO("  lognormal_mu: {}", clutter.lognormal_mu);
        SPDLOG_INFO("  lognormal_sigma: {}", clutter.lognormal_sigma);
        SPDLOG_INFO("");
        SPDLOG_INFO("--- TargetConfig ---");
        SPDLOG_INFO("  enabled: {}", target.enabled ? "true" : "false");
        SPDLOG_INFO("");
    }

} // namespace radar
