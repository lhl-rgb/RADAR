/**
 * @file configuration_manager.cpp
 * @brief 配置管理器实现
 */

#include "core/configuration_manager.h"

#include <fstream>
#include <nlohmann/json.hpp>

namespace radar {

// Helper: convert from json to UdpConfig
void from_json(const nlohmann::json& j, UdpConfig& cfg) {
    if (j.contains("enabled")) j.at("enabled").get_to(cfg.enabled);
    if (j.contains("target_ip")) j.at("target_ip").get_to(cfg.target_ip);
    if (j.contains("target_port")) j.at("target_port").get_to(cfg.target_port);
}

// Helper: convert from json to antenna::BeamTableConfig
void from_json(const nlohmann::json& j, antenna::BeamTableConfig& cfg) {
    if (j.contains("type")) j.at("type").get_to(cfg.type);
    if (j.contains("file_path")) j.at("file_path").get_to(cfg.file_path);

    // 方位扫描参数
    if (j.contains("az_start_deg")) j.at("az_start_deg").get_to(cfg.az_start_deg);
    if (j.contains("az_end_deg")) j.at("az_end_deg").get_to(cfg.az_end_deg);
    if (j.contains("az_step_deg")) j.at("az_step_deg").get_to(cfg.az_step_deg);
    if (j.contains("elevation_deg")) j.at("elevation_deg").get_to(cfg.elevation_deg);

    // 直接定义的波位表数组
    if (j.contains("beams")) {
        const auto& beams = j.at("beams");
        if (beams.is_array()) {
            cfg.beams.clear();
            for (const auto& beam : beams) {
                if (beam.is_array() && beam.size() >= 2) {
                    // 数组格式：[az, el]
                    cfg.beams.emplace_back(
                        beam[0].template get<double>(),
                        beam[1].template get<double>()
                    );
                } else if (beam.contains("azimuth") && beam.contains("elevation")) {
                    // 对象格式：{"azimuth": x, "elevation": y}
                    cfg.beams.emplace_back(
                        beam.at("azimuth").template get<double>(),
                        beam.at("elevation").template get<double>()
                    );
                }
            }
        }
    }

    // 兼容旧格式 custom_beams
    if (j.contains("custom_beams")) {
        const auto& beams = j.at("custom_beams");
        if (beams.is_array()) {
            cfg.beams.clear();
            for (const auto& beam : beams) {
                if (beam.is_array() && beam.size() >= 2) {
                    cfg.beams.emplace_back(
                        beam[0].template get<double>(),
                        beam[1].template get<double>()
                    );
                }
            }
        }
    }
}

// Helper: convert from json to TargetState
void from_json(const nlohmann::json& j, TargetState& target) {
    if (j.contains("id")) j.at("id").get_to(target.id);
    if (j.contains("position_m")) {
        const auto& pos = j.at("position_m");
        if (pos.size() >= 3) {
            target.position_m = Vec3(pos[0].get<Scalar>(), pos[1].get<Scalar>(), pos[2].get<Scalar>());
        }
    }
    if (j.contains("velocity_mps")) {
        const auto& vel = j.at("velocity_mps");
        if (vel.size() >= 3) {
            target.velocity_mps = Vec3(vel[0].get<Scalar>(), vel[1].get<Scalar>(), vel[2].get<Scalar>());
        }
    }
    if (j.contains("acceleration_mps2")) {
        const auto& acc = j.at("acceleration_mps2");
        if (acc.size() >= 3) {
            target.acceleration_mps2 = Vec3(acc[0].get<Scalar>(), acc[1].get<Scalar>(), acc[2].get<Scalar>());
        }
    }
    if (j.contains("motion_model")) j.at("motion_model").get_to(target.motion_model);
    if (j.contains("rcs_mean_m2")) j.at("rcs_mean_m2").get_to(target.rcs_mean_m2);
    if (j.contains("swerling")) j.at("swerling").get_to(target.swerling);
    if (j.contains("enabled")) j.at("enabled").get_to(target.enabled);
}

// Helper: convert from json to GeoCoord
void from_json(const nlohmann::json& j, GeoCoord& coord) {
    if (j.contains("latitude")) j.at("latitude").get_to(coord.latitude);
    if (j.contains("longitude")) j.at("longitude").get_to(coord.longitude);
    if (j.contains("altitude")) j.at("altitude").get_to(coord.altitude);
}

// Helper: convert from json to RadarSystemParams
void from_json(const nlohmann::json& j, RadarSystemParams& sys) {
    if (j.contains("fc_hz")) j.at("fc_hz").get_to(sys.fc_hz);
    if (j.contains("prf_hz")) j.at("prf_hz").get_to(sys.prf_hz);
    if (j.contains("fs_hz")) j.at("fs_hz").get_to(sys.fs_hz);
    if (j.contains("bw_hz")) j.at("bw_hz").get_to(sys.bw_hz);
    if (j.contains("pulse_width_s")) j.at("pulse_width_s").get_to(sys.pulse_width_s);
    if (j.contains("peak_power_w")) j.at("peak_power_w").get_to(sys.peak_power_w);
    if (j.contains("pulses_per_cpi")) j.at("pulses_per_cpi").get_to(sys.pulses_per_cpi);
    if (j.contains("min_range_m")) j.at("min_range_m").get_to(sys.min_range_m);
    if (j.contains("max_range_m")) j.at("max_range_m").get_to(sys.max_range_m);
    if (j.contains("antenna_height_m")) j.at("antenna_height_m").get_to(sys.antenna_height_m);
    if (j.contains("radar_location")) j.at("radar_location").get_to(sys.radar_location);
    if (j.contains("noise_figure_db")) j.at("noise_figure_db").get_to(sys.noise_figure_db);
    if (j.contains("system_loss_db")) j.at("system_loss_db").get_to(sys.system_loss_db);
    sys.compute_derived_params();
}

// Helper: convert from json to waveform::WaveformConfig
void from_json(const nlohmann::json& j, waveform::WaveformConfig& cfg) {
    // 支持扁平化格式（向后兼容）
    if (j.contains("waveform_type")) j.at("waveform_type").get_to(cfg.options.waveform_type);
    if (j.contains("phase_code_type")) j.at("phase_code_type").get_to(cfg.options.phase_code_type);
    if (j.contains("nlfm_window_type")) j.at("nlfm_window_type").get_to(cfg.options.nlfm_window_type);
    if (j.contains("polarization")) j.at("polarization").get_to(cfg.options.polarization);

    // 支持嵌套格式（新格式）
    if (j.contains("options")) {
        const auto& opts = j.at("options");
        if (opts.contains("waveform_type")) opts.at("waveform_type").get_to(cfg.options.waveform_type);
        if (opts.contains("phase_code_type")) opts.at("phase_code_type").get_to(cfg.options.phase_code_type);
        if (opts.contains("nlfm_window_type")) opts.at("nlfm_window_type").get_to(cfg.options.nlfm_window_type);
        if (opts.contains("polarization")) opts.at("polarization").get_to(cfg.options.polarization);
    }
}

// Helper: convert from json to antenna::AntennaConfig
void from_json(const nlohmann::json& j, antenna::AntennaConfig& cfg) {
    // 支持扁平化格式（向后兼容）
    if (j.contains("model_type")) j.at("model_type").get_to(cfg.model_type);
    if (j.contains("weight_type_az")) j.at("weight_type_az").get_to(cfg.weight_type_az);
    if (j.contains("weight_type_el")) j.at("weight_type_el").get_to(cfg.weight_type_el);
    if (j.contains("num_elements_az")) j.at("num_elements_az").get_to(cfg.num_elements_az);
    if (j.contains("num_elements_el")) j.at("num_elements_el").get_to(cfg.num_elements_el);
    if (j.contains("spacing_az_lambda")) j.at("spacing_az_lambda").get_to(cfg.spacing_az_lambda);
    if (j.contains("spacing_el_lambda")) j.at("spacing_el_lambda").get_to(cfg.spacing_el_lambda);
    if (j.contains("peak_gain_db")) j.at("peak_gain_db").get_to(cfg.peak_gain_db);

    // 支持嵌套格式（新格式）
    if (j.contains("options")) {
        const auto& opts = j.at("options");
        if (opts.contains("model_type")) opts.at("model_type").get_to(cfg.model_type);
        if (opts.contains("weight_type_az")) opts.at("weight_type_az").get_to(cfg.weight_type_az);
        if (opts.contains("weight_type_el")) opts.at("weight_type_el").get_to(cfg.weight_type_el);
    }

    if (j.contains("params")) {
        const auto& params = j.at("params");
        if (params.contains("num_elements_az")) params.at("num_elements_az").get_to(cfg.num_elements_az);
        if (params.contains("num_elements_el")) params.at("num_elements_el").get_to(cfg.num_elements_el);
        if (params.contains("spacing_az_lambda")) params.at("spacing_az_lambda").get_to(cfg.spacing_az_lambda);
        if (params.contains("spacing_el_lambda")) params.at("spacing_el_lambda").get_to(cfg.spacing_el_lambda);
        if (params.contains("peak_gain_db")) params.at("peak_gain_db").get_to(cfg.peak_gain_db);
    }
}

// Helper: convert from json to noise::NoiseConfig
void from_json(const nlohmann::json& j, noise::NoiseConfig& cfg) {
    // 支持扁平化格式（向后兼容）
    if (j.contains("enabled")) j.at("enabled").get_to(cfg.options.enabled);
    if (j.contains("mode")) j.at("mode").get_to(cfg.options.mode);
    if (j.contains("sigma_complex")) j.at("sigma_complex").get_to(cfg.params.sigma_complex);
    if (j.contains("noise_power_w")) j.at("noise_power_w").get_to(cfg.params.noise_power_w);
    if (j.contains("system_temperature_k")) j.at("system_temperature_k").get_to(cfg.params.system_temperature_k);
    if (j.contains("noise_bandwidth_hz")) j.at("noise_bandwidth_hz").get_to(cfg.params.noise_bandwidth_hz);
    if (j.contains("seed")) j.at("seed").get_to(cfg.options.seed);

    // 支持嵌套格式（新格式）
    if (j.contains("options")) {
        const auto& opts = j.at("options");
        if (opts.contains("enabled")) opts.at("enabled").get_to(cfg.options.enabled);
        if (opts.contains("mode")) opts.at("mode").get_to(cfg.options.mode);
        if (opts.contains("seed")) opts.at("seed").get_to(cfg.options.seed);
    }

    if (j.contains("params")) {
        const auto& params = j.at("params");
        if (params.contains("sigma_complex")) params.at("sigma_complex").get_to(cfg.params.sigma_complex);
        if (params.contains("noise_power_w")) params.at("noise_power_w").get_to(cfg.params.noise_power_w);
        if (params.contains("system_temperature_k")) params.at("system_temperature_k").get_to(cfg.params.system_temperature_k);
        if (params.contains("noise_bandwidth_hz")) params.at("noise_bandwidth_hz").get_to(cfg.params.noise_bandwidth_hz);
    }
}

// Helper: convert from json to clutter::SeaClutterConfig
void from_json(const nlohmann::json& j, clutter::SeaClutterConfig& cfg) {
    // 支持扁平化格式（向后兼容）
    if (j.contains("enabled")) j.at("enabled").get_to(cfg.options.enabled);
    if (j.contains("sequence_mode")) j.at("sequence_mode").get_to(cfg.options.sequence_mode);
    if (j.contains("seed")) j.at("seed").get_to(cfg.options.seed);

    if (j.contains("ground_range_min_m")) j.at("ground_range_min_m").get_to(cfg.params.ground_range_min_m);
    if (j.contains("ground_range_max_m")) j.at("ground_range_max_m").get_to(cfg.params.ground_range_max_m);
    if (j.contains("range_step_m")) j.at("range_step_m").get_to(cfg.params.range_step_m);
    if (j.contains("beam_az_width_deg")) j.at("beam_az_width_deg").get_to(cfg.params.beam_az_width_deg);
    if (j.contains("az_step_deg")) j.at("az_step_deg").get_to(cfg.params.az_step_deg);
    if (j.contains("k_shape_nu")) j.at("k_shape_nu").get_to(cfg.params.k_shape_nu);
    if (j.contains("doppler_center_hz")) j.at("doppler_center_hz").get_to(cfg.params.doppler_center_hz);
    if (j.contains("doppler_sigma_hz")) j.at("doppler_sigma_hz").get_to(cfg.params.doppler_sigma_hz);
    if (j.contains("pool_length_factor")) j.at("pool_length_factor").get_to(cfg.params.pool_length_factor);

    // 支持嵌套格式（新格式）
    if (j.contains("options")) {
        const auto& opts = j.at("options");
        if (opts.contains("enabled")) opts.at("enabled").get_to(cfg.options.enabled);
        if (opts.contains("sequence_mode")) opts.at("sequence_mode").get_to(cfg.options.sequence_mode);
        if (opts.contains("seed")) opts.at("seed").get_to(cfg.options.seed);
    }

    if (j.contains("params")) {
        const auto& params = j.at("params");
        if (params.contains("ground_range_min_m")) params.at("ground_range_min_m").get_to(cfg.params.ground_range_min_m);
        if (params.contains("ground_range_max_m")) params.at("ground_range_max_m").get_to(cfg.params.ground_range_max_m);
        if (params.contains("range_step_m")) params.at("range_step_m").get_to(cfg.params.range_step_m);
        if (params.contains("beam_az_width_deg")) params.at("beam_az_width_deg").get_to(cfg.params.beam_az_width_deg);
        if (params.contains("az_step_deg")) params.at("az_step_deg").get_to(cfg.params.az_step_deg);
        if (params.contains("k_shape_nu")) params.at("k_shape_nu").get_to(cfg.params.k_shape_nu);
        if (params.contains("doppler_center_hz")) params.at("doppler_center_hz").get_to(cfg.params.doppler_center_hz);
        if (params.contains("doppler_sigma_hz")) params.at("doppler_sigma_hz").get_to(cfg.params.doppler_sigma_hz);
        if (params.contains("pool_length_factor")) params.at("pool_length_factor").get_to(cfg.params.pool_length_factor);
        if (params.contains("morchin")) {
            const auto& mor = params.at("morchin");
            if (mor.contains("sea_state")) mor.at("sea_state").get_to(cfg.params.morchin.sea_state);
        }
    }

    // Morchin 嵌套解析（扁平化兼容）
    if (j.contains("morchin")) {
        const auto& morchin_json = j.at("morchin");
        if (morchin_json.contains("sea_state")) morchin_json.at("sea_state").get_to(cfg.params.morchin.sea_state);
    }
}

// Helper: convert from json to core::ExportConfig
void from_json(const nlohmann::json& j, core::ExportConfig& cfg) {
    if (j.contains("enabled")) j.at("enabled").get_to(cfg.enabled);
    if (j.contains("output_dir")) j.at("output_dir").get_to(cfg.output_dir);
    if (j.contains("export_raw_echo_iq")) j.at("export_raw_echo_iq").get_to(cfg.export_raw_echo_iq);
    if (j.contains("export_target_snapshots")) j.at("export_target_snapshots").get_to(cfg.export_target_snapshots);
    if (j.contains("export_config_params")) j.at("export_config_params").get_to(cfg.export_config_params);
}

// Helper: convert from json to RadarConfig
void from_json(const nlohmann::json& j, RadarConfig& cfg) {
    if (j.contains("system")) j.at("system").get_to(cfg.system);

    // Manually parse nested configs that don't have from_json overloads
    if (j.contains("waveform")) {
        const auto& wf = j.at("waveform");
        // 扁平化格式（向后兼容）
        if (wf.contains("waveform_type")) wf.at("waveform_type").get_to(cfg.waveform.options.waveform_type);
        if (wf.contains("phase_code_type")) wf.at("phase_code_type").get_to(cfg.waveform.options.phase_code_type);
        if (wf.contains("nlfm_window_type")) wf.at("nlfm_window_type").get_to(cfg.waveform.options.nlfm_window_type);
        if (wf.contains("polarization")) wf.at("polarization").get_to(cfg.waveform.options.polarization);

        // 嵌套格式（新格式）
        if (wf.contains("options")) {
            const auto& opts = wf.at("options");
            if (opts.contains("waveform_type")) opts.at("waveform_type").get_to(cfg.waveform.options.waveform_type);
            if (opts.contains("phase_code_type")) opts.at("phase_code_type").get_to(cfg.waveform.options.phase_code_type);
            if (opts.contains("nlfm_window_type")) opts.at("nlfm_window_type").get_to(cfg.waveform.options.nlfm_window_type);
            if (opts.contains("polarization")) opts.at("polarization").get_to(cfg.waveform.options.polarization);
        }
    }

    if (j.contains("antenna")) {
        const auto& ant = j.at("antenna");
        // 扁平化格式（向后兼容）
        if (ant.contains("model_type")) ant.at("model_type").get_to(cfg.antenna.model_type);
        if (ant.contains("num_elements_az")) ant.at("num_elements_az").get_to(cfg.antenna.num_elements_az);
        if (ant.contains("num_elements_el")) ant.at("num_elements_el").get_to(cfg.antenna.num_elements_el);
        if (ant.contains("peak_gain_db")) ant.at("peak_gain_db").get_to(cfg.antenna.peak_gain_db);

        // 嵌套格式（新格式）
        if (ant.contains("options")) {
            const auto& opts = ant.at("options");
            if (opts.contains("model_type")) opts.at("model_type").get_to(cfg.antenna.model_type);
            if (opts.contains("weight_type_az")) opts.at("weight_type_az").get_to(cfg.antenna.weight_type_az);
            if (opts.contains("weight_type_el")) opts.at("weight_type_el").get_to(cfg.antenna.weight_type_el);
        }
        if (ant.contains("params")) {
            const auto& params = ant.at("params");
            if (params.contains("num_elements_az")) params.at("num_elements_az").get_to(cfg.antenna.num_elements_az);
            if (params.contains("num_elements_el")) params.at("num_elements_el").get_to(cfg.antenna.num_elements_el);
            if (params.contains("spacing_az_lambda")) params.at("spacing_az_lambda").get_to(cfg.antenna.spacing_az_lambda);
            if (params.contains("spacing_el_lambda")) params.at("spacing_el_lambda").get_to(cfg.antenna.spacing_el_lambda);
            if (params.contains("peak_gain_db")) params.at("peak_gain_db").get_to(cfg.antenna.peak_gain_db);
        }
    }

    // 解析波位表配置（独立于 antenna）
    if (j.contains("beam_table")) {
        j.at("beam_table").get_to(cfg.beam_table);
    }
    // 兼容旧格式：beam_table_cfg 在 antenna 内部
    else if (j.contains("antenna") && j["antenna"].contains("beam_table_cfg")) {
        j.at("antenna").at("beam_table_cfg").get_to(cfg.beam_table);
    }

    if (j.contains("noise")) {
        const auto& nse = j.at("noise");
        // 扁平化格式（向后兼容）
        if (nse.contains("enabled")) nse.at("enabled").get_to(cfg.noise.options.enabled);
        if (nse.contains("mode")) nse.at("mode").get_to(cfg.noise.options.mode);
        if (nse.contains("sigma_complex")) nse.at("sigma_complex").get_to(cfg.noise.params.sigma_complex);
        if (nse.contains("seed")) nse.at("seed").get_to(cfg.noise.options.seed);

        // 嵌套格式（新格式）
        if (nse.contains("options")) {
            const auto& opts = nse.at("options");
            if (opts.contains("enabled")) opts.at("enabled").get_to(cfg.noise.options.enabled);
            if (opts.contains("mode")) opts.at("mode").get_to(cfg.noise.options.mode);
            if (opts.contains("seed")) opts.at("seed").get_to(cfg.noise.options.seed);
        }
        if (nse.contains("params")) {
            const auto& params = nse.at("params");
            if (params.contains("sigma_complex")) params.at("sigma_complex").get_to(cfg.noise.params.sigma_complex);
        }
    }

    // Clutter config - manual parsing
    if (j.contains("clutter")) {
        const auto& clt = j.at("clutter");
        // 扁平化格式（向后兼容）
        if (clt.contains("enabled")) clt.at("enabled").get_to(cfg.clutter.options.enabled);
        if (clt.contains("k_shape_nu")) clt.at("k_shape_nu").get_to(cfg.clutter.params.k_shape_nu);
        if (clt.contains("doppler_center_hz")) clt.at("doppler_center_hz").get_to(cfg.clutter.params.doppler_center_hz);
        if (clt.contains("doppler_sigma_hz")) clt.at("doppler_sigma_hz").get_to(cfg.clutter.params.doppler_sigma_hz);
        if (clt.contains("ground_range_min_m")) clt.at("ground_range_min_m").get_to(cfg.clutter.params.ground_range_min_m);
        if (clt.contains("ground_range_max_m")) clt.at("ground_range_max_m").get_to(cfg.clutter.params.ground_range_max_m);
        if (clt.contains("morchin")) {
            const auto& mor = clt.at("morchin");
            if (mor.contains("sea_state")) mor.at("sea_state").get_to(cfg.clutter.params.morchin.sea_state);
        }

        // 嵌套格式（新格式）
        if (clt.contains("options")) {
            const auto& opts = clt.at("options");
            if (opts.contains("enabled")) opts.at("enabled").get_to(cfg.clutter.options.enabled);
        }
        if (clt.contains("params")) {
            const auto& params = clt.at("params");
            if (params.contains("k_shape_nu")) params.at("k_shape_nu").get_to(cfg.clutter.params.k_shape_nu);
            if (params.contains("doppler_center_hz")) params.at("doppler_center_hz").get_to(cfg.clutter.params.doppler_center_hz);
            if (params.contains("doppler_sigma_hz")) params.at("doppler_sigma_hz").get_to(cfg.clutter.params.doppler_sigma_hz);
            if (params.contains("ground_range_min_m")) params.at("ground_range_min_m").get_to(cfg.clutter.params.ground_range_min_m);
            if (params.contains("ground_range_max_m")) params.at("ground_range_max_m").get_to(cfg.clutter.params.ground_range_max_m);
            if (params.contains("morchin")) {
                const auto& mor = params.at("morchin");
                if (mor.contains("sea_state")) mor.at("sea_state").get_to(cfg.clutter.params.morchin.sea_state);
            }
        }
    }

    if (j.contains("target")) {
        j.at("target").get_to(cfg.target);
    }

    // Data export config - manual parsing
    if (j.contains("data_export")) {
        const auto& exp = j.at("data_export");
        if (exp.contains("enabled")) exp.at("enabled").get_to(cfg.data_export.enabled);
        if (exp.contains("output_dir")) exp.at("output_dir").get_to(cfg.data_export.output_dir);
        if (exp.contains("export_raw_echo_iq")) exp.at("export_raw_echo_iq").get_to(cfg.data_export.export_raw_echo_iq);
        if (exp.contains("export_target_snapshots")) exp.at("export_target_snapshots").get_to(cfg.data_export.export_target_snapshots);
        if (exp.contains("export_config_params")) exp.at("export_config_params").get_to(cfg.data_export.export_config_params);
    }

    if (j.contains("udp_output")) j.at("udp_output").get_to(cfg.udp_output);

    // Parse initial targets
    if (j.contains("initial_targets")) {
        const auto& targets = j.at("initial_targets");
        if (targets.is_array()) {
            cfg.initial_targets.clear();
            for (const auto& t : targets) {
                TargetState target;
                from_json(t, target);
                cfg.initial_targets.push_back(target);
            }
        }
    }

    cfg.compute_derived_params();
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

        // System parameters
        j["system"]["fc_hz"] = config_.system.fc_hz;
        j["system"]["prf_hz"] = config_.system.prf_hz;
        j["system"]["fs_hz"] = config_.system.fs_hz;
        j["system"]["bw_hz"] = config_.system.bw_hz;
        j["system"]["pulse_width_s"] = config_.system.pulse_width_s;
        j["system"]["peak_power_w"] = config_.system.peak_power_w;
        j["system"]["pulses_per_cpi"] = config_.system.pulses_per_cpi;
        j["system"]["min_range_m"] = config_.system.min_range_m;
        j["system"]["max_range_m"] = config_.system.max_range_m;
        j["system"]["antenna_height_m"] = config_.system.antenna_height_m;
        j["system"]["radar_location"]["latitude"] = config_.system.radar_location.latitude;
        j["system"]["radar_location"]["longitude"] = config_.system.radar_location.longitude;
        j["system"]["radar_location"]["altitude"] = config_.system.radar_location.altitude;
        j["system"]["noise_figure_db"] = config_.system.noise_figure_db;
        j["system"]["system_loss_db"] = config_.system.system_loss_db;

        // Waveform config (支持嵌套格式)
        j["waveform"]["options"]["waveform_type"] = config_.waveform.options.waveform_type;
        j["waveform"]["options"]["phase_code_type"] = config_.waveform.options.phase_code_type;
        j["waveform"]["options"]["nlfm_window_type"] = config_.waveform.options.nlfm_window_type;
        j["waveform"]["options"]["polarization"] = config_.waveform.options.polarization;

        // Antenna config (扁平化格式)
        j["antenna"]["model_type"] = config_.antenna.model_type;
        j["antenna"]["weight_type_az"] = config_.antenna.weight_type_az;
        j["antenna"]["weight_type_el"] = config_.antenna.weight_type_el;
        j["antenna"]["num_elements_az"] = config_.antenna.num_elements_az;
        j["antenna"]["num_elements_el"] = config_.antenna.num_elements_el;
        j["antenna"]["spacing_az_lambda"] = config_.antenna.spacing_az_lambda;
        j["antenna"]["spacing_el_lambda"] = config_.antenna.spacing_el_lambda;
        j["antenna"]["peak_gain_db"] = config_.antenna.peak_gain_db;

        // Noise config (嵌套格式)
        j["noise"]["options"]["enabled"] = config_.noise.options.enabled;
        j["noise"]["options"]["mode"] = config_.noise.options.mode;
        j["noise"]["options"]["seed"] = config_.noise.options.seed;
        j["noise"]["params"]["sigma_complex"] = config_.noise.params.sigma_complex;
        j["noise"]["params"]["noise_power_w"] = config_.noise.params.noise_power_w;
        j["noise"]["params"]["system_temperature_k"] = config_.noise.params.system_temperature_k;
        j["noise"]["params"]["noise_bandwidth_hz"] = config_.noise.params.noise_bandwidth_hz;

        // Clutter config (嵌套格式)
        j["clutter"]["options"]["enabled"] = config_.clutter.options.enabled;
        j["clutter"]["options"]["sequence_mode"] = config_.clutter.options.sequence_mode;
        j["clutter"]["options"]["seed"] = config_.clutter.options.seed;
        j["clutter"]["params"]["ground_range_min_m"] = config_.clutter.params.ground_range_min_m;
        j["clutter"]["params"]["ground_range_max_m"] = config_.clutter.params.ground_range_max_m;
        j["clutter"]["params"]["range_step_m"] = config_.clutter.params.range_step_m;
        j["clutter"]["params"]["beam_az_width_deg"] = config_.clutter.params.beam_az_width_deg;
        j["clutter"]["params"]["az_step_deg"] = config_.clutter.params.az_step_deg;
        j["clutter"]["params"]["k_shape_nu"] = config_.clutter.params.k_shape_nu;
        j["clutter"]["params"]["doppler_center_hz"] = config_.clutter.params.doppler_center_hz;
        j["clutter"]["params"]["doppler_sigma_hz"] = config_.clutter.params.doppler_sigma_hz;
        j["clutter"]["params"]["pool_length_factor"] = config_.clutter.params.pool_length_factor;
        j["clutter"]["params"]["morchin"]["sea_state"] = config_.clutter.params.morchin.sea_state;

        // Target config (嵌套格式)
        // Target config (扁平化)
        j["target"]["enabled"] = config_.target.enabled;
        j["target"]["enable_swerling"] = config_.target.enable_swerling;
        j["target"]["skip_out_of_beam_targets"] = config_.target.skip_out_of_beam_targets;
        j["target"]["seed"] = config_.target.seed;
        j["target"]["enable_beam_gain"] = config_.target.enable_beam_gain;
        j["target"]["enable_two_way_propagation_loss"] = config_.target.enable_two_way_propagation_loss;
        j["target"]["enable_phase"] = config_.target.enable_phase;
        j["target"]["beam_gate_threshold_db"] = config_.target.beam_gate_threshold_db;

        // Data export config
        j["data_export"]["enabled"] = config_.data_export.enabled;
        j["data_export"]["output_dir"] = config_.data_export.output_dir;
        j["data_export"]["export_raw_echo_iq"] = config_.data_export.export_raw_echo_iq;
        j["data_export"]["export_target_snapshots"] = config_.data_export.export_target_snapshots;
        j["data_export"]["export_config_params"] = config_.data_export.export_config_params;

        // UDP output config
        j["udp_output"]["enabled"] = config_.udp_output.enabled;
        j["udp_output"]["target_ip"] = config_.udp_output.target_ip;
        j["udp_output"]["target_port"] = config_.udp_output.target_port;

        // Beam table config (saved as top-level beam_table)
        const auto& beam_cfg = config_.beam_table;
        j["beam_table"]["type"] = beam_cfg.type;
        j["beam_table"]["az_start_deg"] = beam_cfg.az_start_deg;
        j["beam_table"]["az_end_deg"] = beam_cfg.az_end_deg;
        j["beam_table"]["az_step_deg"] = beam_cfg.az_step_deg;
        j["beam_table"]["elevation_deg"] = beam_cfg.elevation_deg;
        j["beam_table"]["file_path"] = beam_cfg.file_path;
        if (!beam_cfg.beams.empty()) {
            j["beam_table"]["beams"] = nlohmann::json::array();
            for (const auto& beam : beam_cfg.beams) {
                nlohmann::json beam_json;
                beam_json["azimuth"] = beam.azimuth;
                beam_json["elevation"] = beam.elevation;
                j["beam_table"]["beams"].push_back(beam_json);
            }
        }

        // Initial targets
        j["initial_targets"] = nlohmann::json::array();
        for (const auto& target : config_.initial_targets) {
            nlohmann::json t;
            t["id"] = target.id;
            t["position_m"] = {target.position_m.x(), target.position_m.y(), target.position_m.z()};
            t["velocity_mps"] = {target.velocity_mps.x(), target.velocity_mps.y(), target.velocity_mps.z()};
            t["acceleration_mps2"] = {target.acceleration_mps2.x(), target.acceleration_mps2.y(), target.acceleration_mps2.z()};
            t["motion_model"] = static_cast<int>(target.motion_model);
            t["rcs_mean_m2"] = target.rcs_mean_m2;
            t["swerling"] = static_cast<int>(target.swerling);
            t["enabled"] = target.enabled;
            j["initial_targets"].push_back(t);
        }

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