/**
 * @file configuration_manager.cpp
 * @brief 配置管理器实现
 */

#include "core/configuration_manager.h"

#include <fstream>
#include <nlohmann/json.hpp>

namespace radar {

// Note: from_json/to_json for all config structs are now defined in their
// respective headers:
// - RadarSystemParams: radar_system_params.h
// - WaveformConfig: waveform_config.h
// - AntennaConfig/BeamTableConfig: antenna_config.h
// - NoiseConfig: noise_config.h
// - SeaClutterConfig: sea_clutter_config.h
// - TargetConfig: target_config.h
// - GeoCoord/TargetState: types.h
// - UdpConfig: udp_config.h
// - ExportConfig: data_exporter.h

void from_json(const nlohmann::json& j, RadarConfig& cfg) {
    if (j.contains("simulation")) {
        j.at("simulation").get_to(cfg.simulation);
    } else if (j.contains("scan_count")) {
        j.at("scan_count").get_to(cfg.simulation.scan_count);
    }

    // System parameters - use unified serializer
    if (j.contains("system")) {
        j.at("system").get_to(cfg.system);
    }

    // Module configs - use unified serializers
    if (j.contains("waveform")) j.at("waveform").get_to(cfg.waveform);
    if (j.contains("antenna")) j.at("antenna").get_to(cfg.antenna);

    // Beam table config (independent of antenna)
    if (j.contains("beam_table")) {
        j.at("beam_table").get_to(cfg.beam_table);
    }
    // Backward compatibility: beam_table_cfg inside antenna
    else if (j.contains("antenna") && j["antenna"].contains("beam_table_cfg")) {
        j.at("antenna").at("beam_table_cfg").get_to(cfg.beam_table);
    }

    if (j.contains("noise")) j.at("noise").get_to(cfg.noise);
    if (j.contains("clutter")) j.at("clutter").get_to(cfg.clutter);
    if (j.contains("target")) j.at("target").get_to(cfg.target);

    // Data export config
    if (j.contains("data_export")) {
        j.at("data_export").get_to(cfg.data_export);
    }

    // UDP output config
    if (j.contains("udp_output")) {
        j.at("udp_output").get_to(cfg.udp_output);
    }

    // Initial targets - use unified TargetState serializer
    if (j.contains("initial_targets")) {
        const auto& targets = j.at("initial_targets");
        if (targets.is_array()) {
            cfg.initial_targets.clear();
            for (const auto& t : targets) {
                cfg.initial_targets.push_back(t.get<TargetState>());
            }
        }
    }

    cfg.compute_derived_params();
}

void to_json(nlohmann::json& j, const RadarConfig& cfg) {
    j["simulation"] = cfg.simulation;
    j["system"] = cfg.system;
    j["waveform"] = cfg.waveform;
    j["antenna"] = cfg.antenna;
    j["beam_table"] = cfg.beam_table;
    j["noise"] = cfg.noise;
    j["clutter"] = cfg.clutter;
    j["target"] = cfg.target;
    j["data_export"] = cfg.data_export;
    j["udp_output"] = cfg.udp_output;

    // Initial targets
    j["initial_targets"] = nlohmann::json::array();
    for (const auto& target : cfg.initial_targets) {
        nlohmann::json t;
        to_json(t, target);
        j["initial_targets"].push_back(t);
    }
}

bool ConfigurationManager::load_from_json(const std::string& filepath) {
    try {
        std::ifstream ifs(filepath);
        if (!ifs.is_open()) {
            last_error_ = "Failed to open config file: " + filepath;
            return false;
        }

        nlohmann::json j;
        ifs >> j;
        ifs.close();

        config_ = j.get<RadarConfig>();
        config_.compute_derived_params();

        // Validate the loaded config
        std::string error;
        if (!config_.validate(error)) {
            last_error_ = "Config validation failed: " + error;
            return false;
        }

        notify_all();
        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("JSON parsing failed: ") + e.what();
        return false;
    }
}

bool ConfigurationManager::save_to_json(const std::string& filepath) const {
    try {
        nlohmann::json j;
        to_json(j, config_);

        std::ofstream ofs(filepath);
        if (!ofs.is_open()) {
            last_error_ = "Failed to open file for writing: " + filepath;
            return false;
        }

        ofs << j.dump(2);  // Pretty print with 2-space indent
        ofs.close();

        return true;
    } catch (const std::exception& e) {
        last_error_ = std::string("JSON serialization failed: ") + e.what();
        return false;
    }
}

void ConfigurationManager::update_system(RadarSystemParams new_system) {
    config_.system = new_system;
    config_.compute_derived_params();
    notify_type<RadarSystemParams>(config_.system);
    notify_all();
}

void ConfigurationManager::update_waveform(waveform::WaveformConfig new_waveform) {
    config_.waveform = new_waveform;
    notify_type<waveform::WaveformConfig>(config_.waveform);
    notify_all();
}

void ConfigurationManager::update_antenna(antenna::AntennaConfig new_antenna) {
    config_.antenna = new_antenna;
    notify_type<antenna::AntennaConfig>(config_.antenna);
    notify_all();
}

void ConfigurationManager::update_noise(noise::NoiseConfig new_noise) {
    config_.noise = new_noise;
    notify_type<noise::NoiseConfig>(config_.noise);
    notify_all();
}

void ConfigurationManager::update_clutter(clutter::SeaClutterConfig new_clutter) {
    config_.clutter = new_clutter;
    notify_type<clutter::SeaClutterConfig>(config_.clutter);
    notify_all();
}

void ConfigurationManager::update_target(target::TargetConfig new_target) {
    config_.target = new_target;
    notify_type<target::TargetConfig>(config_.target);
    notify_all();
}

void ConfigurationManager::update_config(RadarConfig new_config) {
    config_ = new_config;
    config_.compute_derived_params();
    notify_all();
}

void ConfigurationManager::notify_all() {
    for (const auto& [type_idx, observers] : observers_) {
        for (auto* observer : observers) {
            observer->on_config_changed(config_);
        }
    }
}

}  // namespace radar
