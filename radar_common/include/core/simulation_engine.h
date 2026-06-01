/**
 * @file simulation_engine.h
 * @brief 雷达仿真引擎 - 逐脉冲仿真主循环
 *
 * 设计说明：
 * - 生命周期：默认构造，禁用拷贝
 * - 配置注入：通过构造函数传入 RadarConfig
 * - 初始化：initialize() 初始化所有子引擎
 * - 核心功能：run(), run_async(), stop()
 * - 状态查询：is_running(), progress_percent() 等
 */

#pragma once

#include "core/radar_config.h"
#include "core/types.h"

#include "antenna/antenna_model.h"
#include "antenna/mechanical_scanner.h"
#include "clutter/sea_clutter_engine.h"
#include "core/tools/data_exporter.h"
#include "core/tools/udp_sender.h"
#include "core/metrics_collector.h"
#include "noise/noise_engine.h"
#include "output/echo_sink.h"
#include "preview/preview_service.h"
#include "target/target_engine.h"
#include "target/target_manager.h"
#include "waveform/waveform_generator.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace radar {

/**
 * @brief 雷达仿真引擎（逐脉冲处理）
 */
class SimulationEngine {
public:
    explicit SimulationEngine(const RadarConfig& config);
    ~SimulationEngine();

    SimulationEngine(const SimulationEngine&) = delete;
    SimulationEngine& operator=(const SimulationEngine&) = delete;
    SimulationEngine(SimulationEngine&&) = delete;
    SimulationEngine& operator=(SimulationEngine&&) = delete;

    /// @brief 获取仿真配置引用
    const RadarConfig& config() const { return config_; }

    /**
     * @brief 初始化所有子引擎
     * @return 成功返回 true
     */
    bool initialize();
    /// @brief 是否已初始化
    bool is_initialized() const { return initialized_; }
    /// @brief 最近错误信息
    const std::string& last_error() const { return last_error_; }

    /**
     * @brief 同步运行仿真（阻塞当前线程）
     * @param duration_sec 仿真时长（秒），0 表示按配置扫描圈数运行
     */
    void run(int duration_sec = 0);
    /**
     * @brief 异步运行仿真（在独立线程中执行）
     * @param duration_sec 仿真时长（秒），0 表示按配置扫描圈数运行
     */
    void run_async(int duration_sec = 0);
    /// @brief 请求停止仿真
    void stop();

    /// @brief 仿真是否正在运行
    bool is_running() const { return running_; }
    /// @brief 是否已请求停止
    bool stop_requested() const { return stop_requested_; }
    /// @brief 当前扫描圈编号
    int current_scan_index() const { return current_scan_index_; }
    /// @brief 总扫描圈数
    int total_scan_count() const { return total_scan_count_; }
    /// @brief 当前脉冲编号
    int current_pulse_index() const { return pulse_counter_; }
    /// @brief 总脉冲数
    int total_pulse_count() const { return total_pulse_count_; }
    /// @brief 仿真进度百分比 (0-100)
    int progress_percent() const;
    /// @brief 获取仿真性能指标
    core::SimulationMetrics metrics() const;
    /// @brief 获取最近一次距离剖面预览（可选）
    std::optional<preview::RangeProfilePreview> latest_range_profile() const;
    /// @brief 获取最近一次 PPI 显示预览帧（可选）
    std::optional<preview::PpiPreviewFrame> latest_ppi_frame() const;

private:
    /**
     * @brief 逐个初始化所有子引擎
     *
     * 按依赖顺序初始化：波形生成器 → 噪声引擎 → 杂波引擎 → 目标引擎 →
     * 机械扫描器 → UDP 发送器 → 数据导出器 → 回波接收器 → 预览服务 → 指标采集器。
     * 任一失败则立即返回 false。
     */
    bool initialize_engines();
    /// @brief 仿真主循环：逐圈推进 scan loop
    void run_scan_loop();
    /// @brief 处理单个 PRT 的回波生成与子引擎更新
    void process_single_prt(int prt_index, Scalar current_time, ScanEcho& scan_echo);
    /// @brief 完成一圈扫描后的收尾工作（导出、预览更新等）
    void process_scan_complete(int scan_index, const ScanEcho& scan_echo);

    // ---- 配置和状态 ----
    RadarConfig config_;
    std::atomic<bool> running_{false};          ///< 仿真运行标志
    std::atomic<bool> stop_requested_{false};   ///< 停止请求标志
    bool initialized_ = false;                  ///< 初始化完成标志

    int current_scan_index_ = 0;                ///< 当前扫描圈编号
    int total_scan_count_ = 0;                  ///< 计划扫描总圈数
    int total_pulse_count_ = 0;                 ///< 计划脉冲总数
    int pulses_per_scan_ = 0;                   ///< 每圈脉冲数
    int pulse_counter_ = 0;                     ///< 当前脉冲计数器

    std::string last_error_;                    ///< 最近错误信息
    mutable std::mutex mutex_;                  ///< 状态保护锁
    mutable std::mutex monitor_mutex_;          ///< 监控数据保护锁
    std::thread run_thread_;                    ///< 异步运行线程

    // ---- 子引擎 ----
    std::unique_ptr<waveform::WaveformGenerator> waveform_gen_;   ///< 波形生成器
    std::unique_ptr<noise::NoiseEngine> noise_engine_;            ///< 噪声引擎
    std::unique_ptr<clutter::SeaClutterEngine> clutter_engine_;   ///< 海杂波引擎
    std::unique_ptr<target::TargetEngine> target_engine_;         ///< 目标回波引擎
    std::unique_ptr<antenna::MechanicalScanner> mechanical_scanner_; ///< 机械扫描器
    std::unique_ptr<core::UdpSender> udp_sender_;                 ///< UDP 数据发送器
    std::unique_ptr<core::DataExporter> data_exporter_;           ///< 数据导出器
    std::unique_ptr<output::EchoSink> echo_sink_;                 ///< 回波数据接收器
    std::unique_ptr<preview::PreviewService> preview_service_;    ///< 预览服务
    std::unique_ptr<core::MetricsCollector> metrics_;             ///< 性能指标采集器

    TargetList initial_targets_;                                   ///< 初始目标列表
};

}  // namespace radar
