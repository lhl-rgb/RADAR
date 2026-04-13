/**
 * @file simulation_engine.cpp
 * @brief 雷达仿真引擎实现
 */

#include "core/simulation_engine.h"

#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>

namespace radar {

// Using 声明
using core::ExportConfig;
using core::ScanData;

using CpiEcho = radar::CpiEcho;
using TargetList = radar::TargetList;
using BeamPoint = radar::BeamPoint;
using Scalar = radar::Scalar;

// ============================================================================
// 生命周期
// ============================================================================

SimulationEngine::SimulationEngine(const RadarConfig& config)
    : config_(config) {
}

SimulationEngine::~SimulationEngine() {
    stop();
    if (run_thread_.joinable()) {
        run_thread_.join();
    }
}

// ============================================================================
// 初始化
// ============================================================================

bool SimulationEngine::initialize() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (running_) {
        last_error_ = "Simulation is already running";
        return false;
    }

    if (initialized_) {
        return true;  // 已经初始化过
    }

    return initialize_engines();
}

bool SimulationEngine::initialize_engines() {
    // 同步 UDP 配置
    config_.sync_udp_config();

    // 1. 初始化波形生成器
    waveform_gen_ = std::make_unique<waveform::WaveformGenerator>();
    waveform_gen_->set_config(config_.waveform);
    waveform_gen_->set_system_params(config_.system);
    if (!waveform_gen_->initialize()) {
        last_error_ = "WaveformGenerator initialization failed: " + waveform_gen_->last_error();
        return false;
    }

    // 2. 初始化噪声引擎（enabled 开关）
    noise_engine_ = std::make_unique<noise::NoiseEngine>();
    noise_engine_->set_config(config_.noise);
    noise_engine_->set_system_params(config_.system);
    if (!noise_engine_->initialize()) {
        last_error_ = "NoiseEngine initialization failed: " + noise_engine_->last_error();
        return false;
    }

    // 3. 初始化杂波引擎（enabled 开关）
    clutter_engine_ = std::make_unique<clutter::ClutterEngine>();
    clutter_engine_->set_config(config_.clutter);
    if (!clutter_engine_->initialize()) {
        last_error_ = "ClutterEngine initialization failed: " + clutter_engine_->last_error();
        return false;
    }

    // 4. 初始化目标引擎
    target_engine_ = std::make_unique<target::TargetEngine>();
    target_engine_->set_config(config_.target);
    if (!target_engine_->initialize()) {
        last_error_ = "TargetEngine initialization failed: " + target_engine_->last_error();
        return false;
    }

    // 5. 设置初始目标（如果配置为空，TargetEngine 会自动生成默认目标）
    if (!config_.initial_targets.empty()) {
        initial_targets_ = config_.initial_targets;
        target_engine_->set_initial_targets(initial_targets_);
    } else {
        initial_targets_ = target_engine_->initial_targets();
    }

    // 6. 初始化波束扫描控制器
    beam_scanner_ = std::make_unique<antenna::BeamScanner>();
    beam_scanner_->set_antenna_config(config_.antenna);
    beam_scanner_->set_beam_table_config(config_.beam_table);
    if (!beam_scanner_->initialize()) {
        last_error_ = "BeamScanner initialization failed: " + beam_scanner_->last_error();
        return false;
    }
    beam_count_ = static_cast<int>(beam_scanner_->beam_count());

    // 8. 初始化数据导出器（可选模块）
    data_exporter_ = std::make_unique<core::DataExporter>(config_.data_export);
    if (config_.data_export.enabled) {
        if (!data_exporter_->initialize()) {
            last_error_ = "DataExporter initialization failed: " + data_exporter_->last_error();
            return false;
        }
    }

    if(data_exporter_ && config_.data_export.enabled) {
        // 导出配置参数
        if (!data_exporter_->export_config_params(config_.system, config_.target, initial_targets_)) {
            SPDLOG_ERROR("[DataExporter] Failed to export config params: {}", data_exporter_->last_error());
        } else {
            SPDLOG_INFO("[DataExporter] Config params exported.");
        }
        //导出波形
        const std::string& export_dir = config_.data_export.output_dir;
        if(!data_exporter_->export_complex_csv(export_dir + "/tx_waveform.csv", waveform_gen_->get_waveform(), "tx_waveform")) {
            SPDLOG_ERROR("[DataExporter] Failed to export tx waveform: {}", data_exporter_->last_error());
        } else {
            SPDLOG_INFO("[DataExporter] Tx waveform exported.");
        }
        // 导出匹配滤波器
        if(!data_exporter_->export_complex_csv(export_dir + "/matched_filter.csv", waveform_gen_->get_matched_filter(), "matched_filter")) {
            SPDLOG_ERROR("[DataExporter] Failed to export matched filter: {}", data_exporter_->last_error());
        } else {
            SPDLOG_INFO("[DataExporter] Matched filter exported.");
        }
    }

    // 9. 初始化 UDP 发送器（可选模块）
    if (config_.udp_output.enabled) {
        udp_sender_ = std::make_unique<core::UdpSender>(config_.udp_output);
        if (!udp_sender_->initialize()) {
            last_error_ = "UdpSender initialization failed: " + udp_sender_->last_error();
            return false;
        }
    }

    // 计算总扫描数和总 CPI 数
    const int scans = 2;  // 扫描两圈
    total_scan_count_ = scans;
    total_cpi_count_ = beam_count_ * scans;

    SPDLOG_INFO("[SimulationEngine] Initialized");
    SPDLOG_INFO("  Beams per scan: {}", beam_count_);
    SPDLOG_INFO("  Total scans: {}", total_scan_count_);
    SPDLOG_INFO("  Total CPIs: {}", total_cpi_count_);
    SPDLOG_INFO("  Initial targets: {}", initial_targets_.size());
    SPDLOG_INFO("  UDP output: {}", config_.udp_output.enabled ? "enabled" : "disabled");
    SPDLOG_INFO("  Data export: {}", config_.data_export.enabled ? "enabled" : "disabled");
    SPDLOG_INFO("  Clutter: {}", config_.clutter.enabled ? "enabled" : "disabled");

    initialized_ = true;
    return true;
}

// ============================================================================
// 核心功能
// ============================================================================

void SimulationEngine::run(int /* duration_sec */) {
    if (running_) {
        return;
    }

    if (!initialized_) {
        last_error_ = "SimulationEngine not initialized";
        return;
    }

    running_ = true;
    stop_requested_ = false;
    current_scan_index_ = 0;
    cpi_counter_ = 0;

    run_cpi_loop();

    running_ = false;
}

void SimulationEngine::run_async(int duration_sec) {
    if (running_) {
        return;
    }

    if (run_thread_.joinable()) {
        run_thread_.join();
    }

    run_thread_ = std::thread([this, duration_sec]() {
        run(duration_sec);
    });
}

void SimulationEngine::stop() {
    stop_requested_ = true;
}

// ============================================================================
// 状态查询
// ============================================================================

int SimulationEngine::progress_percent() const {
    if (total_cpi_count_ == 0) {
        return 0;
    }
    return (cpi_counter_ * 100) / total_cpi_count_;
}

// ============================================================================
// 内部实现
// ============================================================================

/**
 * @brief 处理单个 CPI
 */
void SimulationEngine::process_single_cpi(int cpi_index,
                                           Scalar current_time,
                                           std::vector<CpiEcho>& scan_cpi_echos,
                                           std::vector<TargetList>& scan_target_snapshots) {
    // 获取当前波位
    BeamPoint beam_pointing = beam_scanner_->get_beam_pointing();
    int beam_index = static_cast<int>(beam_scanner_->get_beam_index());

    SPDLOG_DEBUG("[CPI {}/{}] Beam {} (az={:.1f}°, el={:.1f}°)",
                 cpi_index + 1, total_cpi_count_, beam_index,
                 beam_pointing.azimuth_deg, beam_pointing.elevation_deg);

    // 更新目标状态到当前时刻（通过 TargetEngine 管理）
    target_engine_->update_targets(current_time);
    const TargetList& active_targets = target_engine_->get_current_targets();

    // 获取发射波形
    const ComplexVec& tx_waveform = waveform_gen_->get_waveform();

    // 生成目标回波
    target::BeamView beam_view{beam_pointing, beam_index, &beam_scanner_->antenna_model()};
    CpiEcho target_echo;
    if (config_.target.enabled && !active_targets.empty()) {
        target_engine_->generate(beam_view, config_.system, tx_waveform, target_echo);
    }

    // 生成杂波回波
    CpiEcho clutter_echo;
    if (config_.clutter.enabled) {
        clutter_engine_->generate_sea_clutter_cpi(
            config_.system, beam_scanner_->antenna_model(), beam_pointing, beam_index, tx_waveform, clutter_echo);
    }

    // 合成总回波（目标 + 杂波）
    CpiEcho total_echo;
    total_echo.beam_index = beam_index;
    total_echo.azimuth_deg = beam_pointing.azimuth_deg;
    total_echo.elevation_deg = beam_pointing.elevation_deg;
    total_echo.pulses.assign(
        static_cast<std::size_t>(config_.system.pulses_per_cpi),
        PulseEcho(static_cast<std::size_t>(config_.system.samples_per_pulse), Complex(0.0, 0.0)));

    // 加上目标回波
    for (std::size_t p = 0; p < target_echo.pulses.size() && p < total_echo.pulses.size(); ++p) {
        for (std::size_t n = 0; n < target_echo.pulses[p].size() && n < total_echo.pulses[p].size(); ++n) {
            total_echo.pulses[p][n] += target_echo.pulses[p][n];
        }
    }
    if(config_.clutter.enabled) {
        // 加上杂波回波
        for (std::size_t p = 0; p < clutter_echo.pulses.size() && p < total_echo.pulses.size(); ++p) {
            for (std::size_t n = 0; n < clutter_echo.pulses[p].size() && n < total_echo.pulses[p].size(); ++n) {
                total_echo.pulses[p][n] += clutter_echo.pulses[p][n];
            }
        }
    }
    
    if(config_.noise.enabled){
        // 添加噪声
        noise_engine_->add_noise(total_echo);
    }


    // 计算回波功率统计
    Scalar total_power = 0.0;
    std::size_t total_samples = 0;
    for (const auto& pulse : total_echo.pulses) {
        for (const auto& sample : pulse) {
            total_power += std::norm(sample);
            ++total_samples;
        }
    }
    [[maybe_unused]] Scalar mean_power = math::safe_div(total_power, static_cast<Scalar>(total_samples));

    SPDLOG_DEBUG("  Targets: {}, Mean power: {:.2e}", active_targets.size(), mean_power);

    // 数据收集
    scan_cpi_echos.push_back(total_echo);
    scan_target_snapshots.push_back(active_targets);

    // UDP 发送（每 CPI 发送）
    if (udp_sender_ && udp_sender_->is_initialized()) {
        udp_sender_->send_cpi(total_echo, current_scan_index_, cpi_index, config_.system);
    }
}

void SimulationEngine::run_scan_loop() {
    const Scalar pri = config_.system.pri_s;
    Scalar current_time = 0.0;
    int last_progress_log = 0;

    SPDLOG_INFO("=== Starting Simulation ===");
    SPDLOG_INFO("  Scan mode: {} scans × {} beams/scan = {} CPIs",
                total_scan_count_, beam_count_, total_cpi_count_);

    // 外层循环：扫描圈数
    for (int scan = 0; scan < total_scan_count_ && !stop_requested_; ++scan) {
        current_scan_index_ = scan;

        // 每圈扫描的数据收集
        std::vector<CpiEcho> scan_cpi_echos;
        std::vector<TargetList> scan_target_snapshots;
        scan_cpi_echos.reserve(static_cast<size_t>(beam_count_));
        scan_target_snapshots.reserve(static_cast<size_t>(beam_count_));

        SPDLOG_INFO("--- Starting Scan {} ---", scan);

        // 内层循环：每个扫描的 CPI
        for (int beam = 0; beam < beam_count_ && !stop_requested_; ++beam) {
            const int cpi_index = scan * beam_count_ + beam;

            // 处理单个 CPI
            process_single_cpi(cpi_index, current_time, scan_cpi_echos, scan_target_snapshots);

            // 推进时间
            current_time += static_cast<Scalar>(config_.system.pulses_per_cpi) * pri;

            // 推进波位
            beam_scanner_->advance_one_cpi();

            // 更新全局 CPI 计数器
            cpi_counter_++;

            // 进度日志（每 10%）
            int current_progress = progress_percent();
            if (current_progress >= last_progress_log + 10) {
                SPDLOG_INFO("Progress: {}%", current_progress);
                last_progress_log = (current_progress / 10) * 10;
            }
        }

        // 扫描完成，处理数据（每圈扫描结束后导出）
        process_scan_complete(scan, scan_cpi_echos, scan_target_snapshots);
    }

    SPDLOG_INFO("=== Simulation Complete ===");
    SPDLOG_INFO("Total simulated time: {:.3f} s", current_time);
    SPDLOG_INFO("Progress: {}%", progress_percent());

    // 关闭 UDP 发送器
    if (udp_sender_) {
        udp_sender_->shutdown();
    }
}

void SimulationEngine::run_cpi_loop() {
    // 兼容旧接口，调用新的两层循环
    run_scan_loop();
}

void SimulationEngine::process_scan_complete(int scan_index,
                                              const std::vector<CpiEcho>& cpi_echos,
                                              const std::vector<TargetList>& target_snapshots) {
    // 构建扫描数据
    core::ScanData scan_data;
    scan_data.scan_index = scan_index;
    scan_data.cpi_echos = cpi_echos;
    scan_data.target_snapshots_per_cpi = target_snapshots;
    scan_data.system_params = config_.system;
    scan_data.target_config = config_.target;
    scan_data.initial_targets = initial_targets_;

    // 导出扫描数据
    if (data_exporter_ && config_.data_export.enabled) {
        if (!data_exporter_->export_scan(scan_data)) {
            SPDLOG_ERROR("[DataExporter] Failed to export scan {}: {}", scan_index, data_exporter_->last_error());
        } else {
            SPDLOG_INFO("[DataExporter] Scan {} exported.", scan_index);
        }
    }

    SPDLOG_INFO("[Scan {} complete] CPIs processed: {}", scan_index, cpi_echos.size());
}

}  // namespace radar
