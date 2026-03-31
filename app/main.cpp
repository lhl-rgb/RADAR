/**
 * @file main.cpp
 * @brief 雷达仿真主程序
 */

#include <iostream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "core/types.h"
#include "core/radar_config.h"
#include "core/radar_system_params.h"
#include "core/waveform_config.h"
#include "core/antenna_set.h"
#include "core/waveform_generator.h"
#include "target/target_engine.h"
#include "target/target_manager.h"
#include "noise/noise_engine.h"
#include "clutter/clutter_engine.h"

namespace {

using radar::AntennaScanModel;
using radar::AzEl;
using radar::BeamView;
using radar::ClutterEngine;
using radar::Complex;
using radar::ComplexVec;
using radar::CpiEcho;
using radar::MotionModel;
using radar::NoiseEngine;
using radar::PhasedArrayAntenna;
using radar::RadarConfig;
using radar::RadarSystemParams;
using radar::Scalar;
using radar::SwerlingType;
using radar::TargetEngine;
using radar::TargetList;
using radar::TargetManager;
using radar::TargetState;
using radar::Vec3;
using radar::WaveformGenerator;

}  // namespace

int main() {
    std::cout << "=== Radar Simulation ===\n\n";

    // ========== 1. 配置初始化 ==========
    RadarConfig config;
    config.compute_derived_params();

    // 配置验证
    std::string error;
    if (!config.validate(error)) {
        std::cerr << "Configuration validation failed: " << error << "\n";
        return 1;
    }

    std::cout << "[Config] Loaded successfully\n";
    std::cout << "  fc = " << config.system.fc_hz / 1e9 << " GHz\n";
    std::cout << "  prf = " << config.system.prf_hz << " Hz\n";
    std::cout << "  pulses_per_cpi = " << config.system.pulses_per_cpi << "\n";
    std::cout << "  wavelength = " << config.system.wavelength_m * 100 << " cm\n\n";

    // ========== 2. Engine 初始化 ==========
    WaveformGenerator waveform_gen(config.system, config.waveform);
    NoiseEngine noise_engine(config.system, config.noise);
    ClutterEngine clutter_engine;
    TargetEngine target_engine;
    TargetManager target_manager;

    // 初始化杂波引擎
    if (config.clutter.enabled) {
        // 转换 SeaClutterConfig 到 SeaClutterParams（向后兼容）
        radar::SeaClutterParams sea_params;
        sea_params.enabled = config.clutter.enabled;
        sea_params.ground_range_min_m = config.clutter.ground_range_min_m;
        sea_params.ground_range_max_m = config.clutter.ground_range_max_m;
        sea_params.range_step_m = config.clutter.range_step_m;
        sea_params.beam_az_width_deg = config.clutter.beam_az_width_deg;
        sea_params.az_step_deg = config.clutter.az_step_deg;
        sea_params.k_shape_nu = config.clutter.k_shape_nu;
        sea_params.doppler_center_hz = config.clutter.doppler_center_hz;
        sea_params.doppler_sigma_hz = config.clutter.doppler_sigma_hz;
        sea_params.sequence_mode = config.clutter.sequence_mode;
        sea_params.seed = config.clutter.seed;
        sea_params.pool_length_factor = config.clutter.pool_length_factor;
        sea_params.morchin.a0_db = config.clutter.morchin.a0_db;
        sea_params.morchin.a_g = config.clutter.morchin.a_g;
        sea_params.morchin.a_f = config.clutter.morchin.a_f;
        sea_params.morchin.a_s = config.clutter.morchin.a_s;
        sea_params.morchin.sea_state = config.clutter.morchin.sea_state;
        sea_params.morchin.sin_psi_floor = config.clutter.morchin.sin_psi_floor;
        clutter_engine.set_sea_params(sea_params);
    }

    // ========== 3. 天线初始化 ==========
    PhasedArrayAntenna antenna;
    // 转换 AntennaConfig 到 PhasedArrayAntennaConfig（向后兼容）
    radar::PhasedArrayAntennaConfig antenna_config;
    antenna_config.model_type = config.antenna.model_type;
    antenna_config.num_elements_az = config.antenna.num_elements_az;
    antenna_config.num_elements_el = config.antenna.num_elements_el;
    antenna_config.spacing_az_lambda = config.antenna.spacing_az_lambda;
    antenna_config.spacing_el_lambda = config.antenna.spacing_el_lambda;
    antenna_config.weight_type_az = config.antenna.weight_type_az;
    antenna_config.weight_type_el = config.antenna.weight_type_el;
    antenna_config.peak_gain_db = config.antenna.peak_gain_db;
    antenna.set_config(antenna_config);

    AntennaScanModel scan_model;
    scan_model.set_antenna(antenna);

    // 设置简单的波位表（方位扫描）
    std::vector<radar::BeamPoint> beam_table;
    for (int az = -30; az <= 30; az += 10) {
        beam_table.push_back({static_cast<Scalar>(az), 0.0});
    }
    scan_model.set_beam_table(beam_table);
    std::cout << "[Antenna] Beam table loaded: " << beam_table.size() << " beams\n\n";

    // ========== 4. 目标初始化 ==========
    std::vector<TargetState> initial_targets;
    // 目标1：静止目标
    initial_targets.push_back({
        1,  // id
        Vec3(50000.0, 0.0, 5000.0),  // position (50km, 0, 5km altitude)
        Vec3::Zero(),                 // velocity
        Vec3::Zero(),                 // acceleration
        MotionModel::Stationary,
        10.0,                         // rcs m^2
        SwerlingType::Swerling1,
        true
    });
    // 目标2：运动目标
    initial_targets.push_back({
        2,
        Vec3(30000.0, 10000.0, 3000.0),
        Vec3(100.0, 50.0, 0.0),       // 100 m/s velocity
        Vec3::Zero(),
        MotionModel::ConstantVelocity,
        5.0,
        SwerlingType::Swerling1,
        true
    });
    target_manager.set_targets(initial_targets);
    std::cout << "[Targets] Initialized " << initial_targets.size() << " targets\n\n";

    // ========== 5. 仿真参数 ==========
    const int total_cpi_count = static_cast<int>(scan_model.beam_count()) * 2;  // 扫描两圈
    const Scalar pri = config.system.pri_s;
    Scalar current_time = 0.0;

    std::cout << "=== Starting Simulation ===\n";
    std::cout << "Total CPIs: " << total_cpi_count << "\n\n";

    // ========== 6. Main Loop ==========
    for (int cpi = 0; cpi < total_cpi_count; ++cpi) {
        // 6.1 获取当前波位
        AzEl beam_pointing = scan_model.get_beam_pointing();
        int beam_index = static_cast<int>(scan_model.get_beam_index());

        std::cout << "[CPI " << cpi + 1 << "/" << total_cpi_count << "] "
                  << "Beam " << beam_index
                  << " (az=" << beam_pointing.azimuth << "°, el=" << beam_pointing.elevation << "°)\n";

        // 6.2 更新目标状态到当前时刻
        target_manager.update_to_time(current_time);
        TargetList active_targets = target_manager.get_current_targets();

        // 6.3 生成发射波形
        const ComplexVec& tx_waveform = waveform_gen.get_waveform();

        // 6.4 生成目标回波
        BeamView beam_view{beam_pointing, beam_index, &antenna};
        CpiEcho target_echo;
        if (config.target.enabled && !active_targets.empty()) {
            // 转换 RadarParams（向后兼容）
            radar::RadarParams radar_params;
            radar_params.fc_hz = config.system.fc_hz;
            radar_params.prf_hz = config.system.prf_hz;
            radar_params.fs_hz = config.system.fs_hz;
            radar_params.bw_hz = config.system.bw_hz;
            radar_params.pulse_width_s = config.system.pulse_width_s;
            radar_params.peak_power_w = config.system.peak_power_w;
            radar_params.pulses_per_cpi = config.system.pulses_per_cpi;
            radar_params.samples_per_pulse = config.system.samples_per_pulse;
            radar_params.min_range_m = config.system.min_range_m;
            radar_params.max_range_m = config.system.max_range_m;
            radar_params.wavelength_m = config.system.wavelength_m;
            radar_params.range_resolution_m = config.system.range_resolution_m;
            radar_params.velocity_resolution_mps = config.system.velocity_resolution_mps;
            radar_params.max_unambiguous_range_m = config.system.max_unambiguous_range_m;
            radar_params.max_unambiguous_velocity = config.system.max_unambiguous_velocity_mps;
            radar_params.noise_figure_db = config.system.noise_figure_db;
            radar_params.system_loss_db = config.system.system_loss_db;
            radar_params.noise_figure_linear = config.system.noise_figure_linear;
            radar_params.system_loss_linear = config.system.system_loss_linear;
            radar_params.antenna_height_m = config.system.antenna_height_m;
            radar_params.radar_location = config.system.radar_location;
            radar_params.target_params.enabled = config.target.enabled;
            radar_params.target_params.enable_beam_gain = config.target.enable_beam_gain;
            radar_params.target_params.enable_two_way_propagation_loss = config.target.enable_two_way_propagation_loss;
            radar_params.target_params.enable_phase = config.target.enable_phase;
            radar_params.target_params.enable_swerling = config.target.enable_swerling;
            radar_params.target_params.seed = config.target.seed;
            radar_params.target_params.beam_gate_threshold_db = config.target.beam_gate_threshold_db;
            radar_params.target_params.skip_out_of_beam_targets = config.target.skip_out_of_beam_targets;

            target_engine.generate(active_targets, beam_view, radar_params, tx_waveform, target_echo);
        }

        // 6.5 生成杂波回波
        CpiEcho clutter_echo;
        if (config.clutter.enabled) {
            radar::RadarParams radar_params;  // 同上转换
            radar_params.fc_hz = config.system.fc_hz;
            radar_params.prf_hz = config.system.prf_hz;
            radar_params.fs_hz = config.system.fs_hz;
            radar_params.bw_hz = config.system.bw_hz;
            radar_params.pulse_width_s = config.system.pulse_width_s;
            radar_params.pulses_per_cpi = config.system.pulses_per_cpi;
            radar_params.samples_per_pulse = config.system.samples_per_pulse;
            radar_params.min_range_m = config.system.min_range_m;
            radar_params.max_range_m = config.system.max_range_m;
            radar_params.wavelength_m = config.system.wavelength_m;
            radar_params.antenna_height_m = config.system.antenna_height_m;

            clutter_engine.generate_sea_clutter_cpi(
                radar_params, antenna, beam_pointing, beam_index, tx_waveform, clutter_echo);
        }

        // 6.6 合成总回波（目标 + 杂波）
        CpiEcho total_echo;
        total_echo.beam_index = beam_index;
        total_echo.azimuth_deg = beam_pointing.azimuth;
        total_echo.elevation_deg = beam_pointing.elevation;
        total_echo.pulses.assign(
            static_cast<std::size_t>(config.system.pulses_per_cpi),
            radar::PulseEcho(static_cast<std::size_t>(config.system.samples_per_pulse), Complex(0.0, 0.0)));

        // 加上目标回波
        for (std::size_t p = 0; p < target_echo.pulses.size() && p < total_echo.pulses.size(); ++p) {
            for (std::size_t n = 0; n < target_echo.pulses[p].size() && n < total_echo.pulses[p].size(); ++n) {
                total_echo.pulses[p][n] += target_echo.pulses[p][n];
            }
        }

        // 加上杂波回波
        for (std::size_t p = 0; p < clutter_echo.pulses.size() && p < total_echo.pulses.size(); ++p) {
            for (std::size_t n = 0; n < clutter_echo.pulses[p].size() && n < total_echo.pulses[p].size(); ++n) {
                total_echo.pulses[p][n] += clutter_echo.pulses[p][n];
            }
        }

        // 6.7 添加噪声
        noise_engine.add_noise(total_echo);

        // 6.8 计算回波功率统计
        Scalar total_power = 0.0;
        std::size_t total_samples = 0;
        for (const auto& pulse : total_echo.pulses) {
            for (const auto& sample : pulse) {
                total_power += std::norm(sample);
                ++total_samples;
            }
        }
        Scalar mean_power = total_power / static_cast<Scalar>(total_samples);

        std::cout << "  Targets: " << active_targets.size()
                  << ", Mean power: " << mean_power << "\n";

        // 6.9 推进时间和波位
        current_time += static_cast<Scalar>(config.system.pulses_per_cpi) * pri;
        scan_model.advance_one_cpi();
    }

    std::cout << "\n=== Simulation Complete ===\n";
    std::cout << "Total simulated time: " << current_time << " s\n";

    return 0;
}