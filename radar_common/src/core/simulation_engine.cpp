/**
 * @file simulation_engine.cpp
 * @brief 雷达仿真引擎实现（逐脉冲处理）
 */

#include "core/simulation_engine.h"
#include "core/echo_frame.h"
#include "output/echo_sink.h"

#include <spdlog/spdlog.h>
#include <chrono>
#include <cmath>
#include <thread>

namespace radar {

namespace {
using Clock = std::chrono::steady_clock;

float elapsed_us(const Clock::time_point& start, const Clock::time_point& finish) {
    return static_cast<float>(std::chrono::duration<double, std::micro>(finish - start).count());
}
}

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

    if (initialized_) return true;

    return initialize_engines();
}

bool SimulationEngine::initialize_engines() {
    // 1. 波形生成器
    waveform_gen_ = std::make_unique<waveform::WaveformGenerator>();
    waveform_gen_->set_config(config_.waveform);
    waveform_gen_->set_system_params(config_.system);
    if (!waveform_gen_->initialize()) {
        last_error_ = "WaveformGenerator initialization failed: " + waveform_gen_->last_error();
        return false;
    }

    // 2. 噪声引擎
    noise_engine_ = std::make_unique<noise::NoiseEngine>();
    noise_engine_->set_config(config_.noise);
    noise_engine_->set_system_params(config_.system);
    if (!noise_engine_->initialize()) {
        last_error_ = "NoiseEngine initialization failed: " + noise_engine_->last_error();
        return false;
    }

    // 3. 杂波引擎（FIR 高斯谱整形 + 天线方向图加权）
    clutter_engine_ = std::make_unique<clutter::SeaClutterEngine>();
    clutter_engine_->set_config(config_.clutter);
    clutter_engine_->set_antenna_config(config_.antenna);
    if (!clutter_engine_->initialize(config_.system)) {
        last_error_ = "SeaClutterEngine initialization failed: " + clutter_engine_->last_error();
        return false;
    }

    // 4. 目标引擎
    target_engine_ = std::make_unique<target::TargetEngine>();
    target_engine_->set_config(config_.target);
    if (!target_engine_->initialize()) {
        last_error_ = "TargetEngine initialization failed: " + target_engine_->last_error();
        return false;
    }

    // 5. 初始目标
    if (!config_.initial_targets.empty()) {
        initial_targets_ = config_.initial_targets;
        target_engine_->set_initial_targets(initial_targets_);
    } else {
        initial_targets_ = target_engine_->initial_targets();
    }

    // 6. 机械扫描控制器
    mechanical_scanner_ = std::make_unique<antenna::MechanicalScanner>();
    mechanical_scanner_->set_antenna_config(config_.antenna);
    mechanical_scanner_->set_scan_config(config_.mech_scan);
    if (!mechanical_scanner_->initialize()) {
        last_error_ = "MechanicalScanner initialization failed: " + mechanical_scanner_->last_error();
        return false;
    }

    // 计算每圈旋转脉冲数
    pulses_per_scan_ = config_.mech_scan.pulses_per_rotation;
    if (pulses_per_scan_ <= 0) {
        last_error_ = "Invalid pulses_per_rotation (check rotation_rate and PRI)";
        return false;
    }

    total_scan_count_ = config_.simulation.scan_count;
    total_pulse_count_ = pulses_per_scan_ * total_scan_count_;

    // 7. 数据导出器
    data_exporter_ = std::make_unique<core::DataExporter>(config_.data_export);
    if (config_.data_export.enabled) {
        if (!data_exporter_->initialize()) {
            last_error_ = "DataExporter initialization failed: " + data_exporter_->last_error();
            return false;
        }
        // 导出配置和波形
        if (!data_exporter_->export_config_params(config_.system, config_.target, config_.mech_scan, initial_targets_)) {
            SPDLOG_ERROR("[DataExporter] Failed to export config params: {}", data_exporter_->last_error());
        }
        const std::string& export_dir = config_.data_export.output_dir;
        data_exporter_->export_complex_csv(export_dir + "/tx_waveform.csv", waveform_gen_->get_waveform(), "tx_waveform");
        data_exporter_->export_complex_csv(export_dir + "/matched_filter.csv", waveform_gen_->get_matched_filter(), "matched_filter");
    }

    // 8. UDP 发送器
    if (config_.udp_output.enabled) {
        udp_sender_ = std::make_unique<core::UdpSender>(config_.udp_output);
        if (!udp_sender_->initialize()) {
            last_error_ = "UdpSender initialization failed: " + udp_sender_->last_error();
            return false;
        }
    }

    // 9. 数据面 EchoSink
    echo_sink_ = output::create_echo_sink(config_.output);
    if (echo_sink_) {
        if (!echo_sink_->open(config_.output)) {
            last_error_ = "EchoSink initialization failed: " + echo_sink_->last_error();
            return false;
        }
    }

    // 10. 监控 preview
    preview_service_ = std::make_unique<preview::PreviewService>();
    preview_service_->set_config(config_.preview);
    if (!preview_service_->initialize(config_.system.samples_per_pulse)) {
        last_error_ = "PreviewService initialization failed: " + preview_service_->last_error();
        return false;
    }

    metrics_ = std::make_unique<core::MetricsCollector>();
    metrics_->reset(static_cast<std::uint64_t>(total_pulse_count_), config_.system.prf_hz);

    SPDLOG_INFO("[SimulationEngine] Initialized");
    SPDLOG_INFO("  Pulses per scan: {}", pulses_per_scan_);
    SPDLOG_INFO("  Total scans: {}", total_scan_count_);
    SPDLOG_INFO("  Total pulses: {}", total_pulse_count_);
    SPDLOG_INFO("  Initial targets: {}", initial_targets_.size());
    SPDLOG_INFO("  Clutter: {}", config_.clutter.enabled ? "enabled" : "disabled");

    initialized_ = true;
    return true;
}

// ============================================================================
// 核心功能
// ============================================================================

void SimulationEngine::run(int /* duration_sec */) {
    if (running_) return;
    if (!initialized_) {
        last_error_ = "SimulationEngine not initialized";
        return;
    }

    running_ = true;
    stop_requested_ = false;
    current_scan_index_ = 0;
    pulse_counter_ = 0;

    run_scan_loop();

    running_ = false;
}

void SimulationEngine::run_async(int duration_sec) {
    if (running_) return;
    if (run_thread_.joinable()) run_thread_.join();

    run_thread_ = std::thread([this, duration_sec]() {
        run(duration_sec);
    });
}

void SimulationEngine::stop() {
    stop_requested_ = true;
}

int SimulationEngine::progress_percent() const {
    if (total_pulse_count_ == 0) return 0;
    return (pulse_counter_ * 100) / total_pulse_count_;
}

core::SimulationMetrics SimulationEngine::metrics() const {
    std::lock_guard<std::mutex> lock(monitor_mutex_);
    if (!metrics_) {
        return core::SimulationMetrics{};
    }
    return metrics_->metrics();
}

std::optional<preview::RangeProfilePreview> SimulationEngine::latest_range_profile() const {
    std::lock_guard<std::mutex> lock(monitor_mutex_);
    if (!preview_service_) {
        return std::nullopt;
    }
    const auto* profile = preview_service_->latest_range_profile();
    if (!profile) {
        return std::nullopt;
    }
    return *profile;
}

std::optional<preview::PpiPreviewFrame> SimulationEngine::latest_ppi_frame() const {
    std::lock_guard<std::mutex> lock(monitor_mutex_);
    if (!preview_service_) {
        return std::nullopt;
    }
    const auto* frame = preview_service_->latest_ppi_frame();
    if (!frame) {
        return std::nullopt;
    }
    return *frame;
}

// ============================================================================
// 仿真主循环（逐脉冲）
// ============================================================================

void SimulationEngine::run_scan_loop() {
    const Scalar pri = config_.system.pri_s;
    Scalar current_time = 0.0;
    int last_progress_log = 0;

    SPDLOG_INFO("=== Starting Simulation ===");
    SPDLOG_INFO("  {} scans × {} pulses/scan = {} total pulses",
                total_scan_count_, pulses_per_scan_, total_pulse_count_);

    for (int scan = 0; scan < total_scan_count_ && !stop_requested_; ++scan) {
        current_scan_index_ = scan;
        mechanical_scanner_->reset();

        ScanEcho scan_echo;
        scan_echo.scan_index = scan;
        scan_echo.pulse_results.reserve(pulses_per_scan_);

        SPDLOG_INFO("--- Starting Scan {} ---", scan);

        // 内层循环：逐脉冲
        for (int prt = 0; prt < pulses_per_scan_ && !stop_requested_; ++prt) {
            process_single_prt(prt, current_time, scan_echo);

            current_time += pri;
            mechanical_scanner_->advance_one_prt();
            pulse_counter_++;

            // 进度日志（每 10%）
            int current_progress = progress_percent();
            if (current_progress >= last_progress_log + 10) {
                SPDLOG_INFO("Progress: {}%", current_progress);
                last_progress_log = (current_progress / 10) * 10;
            }
        }

        // 扫描完成，处理数据
        process_scan_complete(scan, scan_echo);
    }

    SPDLOG_INFO("=== Simulation Complete ===");
    SPDLOG_INFO("Total simulated time: {:.3f} s", current_time);

    if (udp_sender_) udp_sender_->shutdown();
    if (echo_sink_) echo_sink_->close();
}

void SimulationEngine::process_single_prt(int prt_index, Scalar current_time,
                                           ScanEcho& scan_echo) {
    // 获取当前波束位置
    BeamPoint beam = mechanical_scanner_->get_beam_pointing();

    // 更新目标状态
    target_engine_->update_targets(current_time);
    const TargetList& active_targets = target_engine_->get_current_targets();

    // 获取发射波形
    const ComplexVec& tx_waveform = waveform_gen_->get_waveform();
    const int samples_per_pulse = config_.system.samples_per_pulse;

    // 初始化单脉冲回波
    PulseEcho total_echo(static_cast<std::size_t>(samples_per_pulse), Complex(0.0, 0.0));

    const bool echo_active = mechanical_scanner_->is_echo_active();

    float target_us = 0.0f;
    float clutter_us = 0.0f;
    float noise_us = 0.0f;
    float compose_us = 0.0f;
    float output_us = 0.0f;
    float preview_us = 0.0f;

    // 1. 目标回波（使用当前脉冲的波束位置）
    if (echo_active && config_.target.enabled && !active_targets.empty()) {
        const auto start = Clock::now();
        target::BeamView beam_view{beam, &mechanical_scanner_->antenna_model()};
        target::PulseContext pulse_context{current_scan_index_, prt_index, pulse_counter_, current_time};
        PulseEcho target_pulse;
        if (target_engine_->generate_pulse(beam_view, pulse_context, config_.system, tx_waveform, target_pulse)) {
            for (std::size_t n = 0; n < target_pulse.size() && n < total_echo.size(); ++n) {
                total_echo[n] += target_pulse[n];
            }
        }
        target_us = elapsed_us(start, Clock::now());
    }

    // 2. 杂波回波（从杂波地图获取）
    if (echo_active && config_.clutter.enabled && clutter_engine_) {
        const auto start = Clock::now();
        PulseEcho clutter_echo;
        if (clutter_engine_->get_clutter_for_pulse(beam, prt_index, clutter_echo)) {
            for (std::size_t n = 0; n < clutter_echo.size() && n < total_echo.size(); ++n) {
                total_echo[n] += clutter_echo[n];
            }
        }
        clutter_us = elapsed_us(start, Clock::now());
    }

    // 3. 噪声
    if (config_.noise.enabled && noise_engine_) {
        const auto start = Clock::now();
        noise_engine_->add_noise(total_echo);
        noise_us = elapsed_us(start, Clock::now());
    }

    // 4. 构造 PulseResult 并加入扫描结果
    const auto compose_start = Clock::now();
    PulseResult result;
    result.pulse_index = prt_index;
    result.azimuth_deg = beam.azimuth_deg;
    result.elevation_deg = beam.elevation_deg;
    result.echo = total_echo;
    compose_us = elapsed_us(compose_start, Clock::now());

    core::EchoFrame frame;
    frame.pulse_index = static_cast<std::uint64_t>(pulse_counter_);
    frame.scan_index = static_cast<std::uint32_t>(current_scan_index_);
    frame.timestamp_s = current_time;
    frame.beam_az_deg = beam.azimuth_deg;
    frame.beam_el_deg = beam.elevation_deg;
    frame.range_bin_size_m = config_.system.range_bin_size_m;
    frame.iq = std::move(total_echo);

    if (echo_sink_) {
        const auto start = Clock::now();
        if (!echo_sink_->write_frame(frame)) {
            last_error_ = "EchoSink write failed: " + echo_sink_->last_error();
            stop_requested_ = true;
        }
        output_us = elapsed_us(start, Clock::now());
    }

    if (preview_service_) {
        const auto start = Clock::now();
        {
            std::lock_guard<std::mutex> lock(monitor_mutex_);
            preview_service_->consume_frame(frame);
        }
        preview_us = elapsed_us(start, Clock::now());
    }

    if (metrics_) {
        std::lock_guard<std::mutex> lock(monitor_mutex_);
        metrics_->record_pulse(static_cast<std::uint32_t>(current_scan_index_),
                               static_cast<std::uint64_t>(pulse_counter_),
                               target_us,
                               clutter_us,
                               noise_us,
                               compose_us,
                               output_us,
                               preview_us,
                               echo_sink_ ? echo_sink_->frames_written() : 0U,
                               preview_service_ ? preview_service_->dropped_frames() : 0U);
    }

    scan_echo.pulse_results.push_back(std::move(result));

    // 5. UDP 发送
    if (udp_sender_ && udp_sender_->is_initialized()) {
        // UDP 发送单脉冲数据（需要适配）
        // TODO: 适配 UDP 发送接口
    }
}

void SimulationEngine::process_scan_complete(int scan_index, const ScanEcho& scan_echo) {
    if (data_exporter_ && config_.data_export.enabled) {
        core::ScanData scan_data;
        scan_data.scan_echo = scan_echo;
        scan_data.system_params = config_.system;
        scan_data.target_config = config_.target;
        scan_data.mech_scan_config = config_.mech_scan;
        scan_data.initial_targets = initial_targets_;
        data_exporter_->export_scan(scan_data);
    }

    SPDLOG_INFO("[Scan {} complete] Pulses processed: {}", scan_index, scan_echo.pulse_results.size());
}

}  // namespace radar
