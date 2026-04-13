/**
 * @file simulation_engine.h
 * @brief 雷达仿真引擎 - 封装仿真主循环
 *
 * 设计说明：
 * - 生命周期：默认构造，禁用拷贝
 * - 配置注入：通过构造函数传入 RadarConfig
 * - 初始化：initialize() 初始化所有子引擎
 * - 核心功能：run(), run_async(), stop()
 * - 状态查询：is_running(), progress_percent() 等
 *
 * 使用示例：
 * @code
 * RadarConfig config = load_config();
 * SimulationEngine engine(config);
 *
 * if (engine.initialize()) {
 *     engine.run();  // 运行完整仿真
 *     engine.stop();
 * }
 * @endcode
 */

#pragma once

#include "core/radar_config.h"
#include "core/types.h"

#include "antenna/antenna_model.h"
#include "antenna/beam_scanner.h"
#include "clutter/clutter_engine.h"
#include "core/tools/data_exporter.h"
#include "core/tools/udp_sender.h"
#include "noise/noise_engine.h"
#include "target/target_engine.h"
#include "target/target_manager.h"
#include "waveform/waveform_generator.h"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace radar {

/**
 * @brief 雷达仿真引擎
 * @details
 * 封装完整的雷达仿真主循环，支持：
 * - 启动/停止控制
 * - 进度查询
 * - UDP 数据输出
 * - 文件数据导出
 */
class SimulationEngine {
public:
    // ========================================================================
    // 生命周期
    // ========================================================================

    /**
     * @brief 构造函数
     * @param config 雷达配置
     */
    explicit SimulationEngine(const RadarConfig& config);

    /**
     * @brief 析构函数
     */
    ~SimulationEngine();

    // 禁用拷贝操作
    SimulationEngine(const SimulationEngine&) = delete;
    SimulationEngine& operator=(const SimulationEngine&) = delete;
    SimulationEngine(SimulationEngine&&) = delete;
    SimulationEngine& operator=(SimulationEngine&&) = delete;

    // ========================================================================
    // 配置注入
    // ========================================================================

    /**
     * @brief 获取配置（只读）
     */
    const RadarConfig& config() const { return config_; }

    // ========================================================================
    // 初始化
    // ========================================================================

    /**
     * @brief 初始化仿真引擎
     * @details 初始化所有子引擎（波形、噪声、杂波、目标、天线等）
     * @return 成功返回 true
     */
    bool initialize();

    /**
     * @brief 检查是否已初始化
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief 获取最近一次错误信息
     */
    const std::string& last_error() const { return last_error_; }

    // ========================================================================
    // 核心功能
    // ========================================================================

    /**
     * @brief 运行仿真（阻塞直到完成）
     * @param duration_sec 运行时长（秒），0 表示运行完所有扫描
     */
    void run(int duration_sec = 0);

    /**
     * @brief 异步运行仿真（在独立线程中）
     * @param duration_sec 运行时长（秒），0 表示运行完所有扫描
     */
    void run_async(int duration_sec = 0);

    /**
     * @brief 停止仿真
     */
    void stop();

    // ========================================================================
    // 状态查询
    // ========================================================================

    /**
     * @brief 检查是否正在运行
     */
    bool is_running() const { return running_; }

    /**
     * @brief 检查是否请求停止
     */
    bool stop_requested() const { return stop_requested_; }

    /**
     * @brief 获取当前扫描索引
     */
    int current_scan_index() const { return current_scan_index_; }

    /**
     * @brief 获取总扫描数
     */
    int total_scan_count() const { return total_scan_count_; }

    /**
     * @brief 获取总 CPI 数
     */
    int total_cpi_count() const { return total_cpi_count_; }

    /**
     * @brief 获取当前 CPI 索引
     */
    int current_cpi_index() const { return cpi_counter_; }

    /**
     * @brief 获取每圈扫描的波位数
     */
    int beam_count() const { return beam_count_; }

    /**
     * @brief 获取进度百分比
     */
    int progress_percent() const;

private:
    /**
     * @brief 初始化各子引擎
     * @return 成功返回 true
     */
    bool initialize_engines();

    /**
     * @brief 运行扫描循环（两层循环：scan → cpi）
     */
    void run_scan_loop();

    /**
     * @brief 处理单个 CPI
     * @param cpi_index CPI 索引
     * @param current_time 当前时间
     * @param scan_cpi_echos 扫描 CPI 回波数据
     * @param scan_target_snapshots 扫描目标快照
     */
    void process_single_cpi(int cpi_index,
                            Scalar current_time,
                            std::vector<CpiEcho>& scan_cpi_echos,
                            std::vector<TargetList>& scan_target_snapshots);

    /**
     * @brief 主循环实现（兼容旧接口，调用 run_scan_loop）
     * @deprecated 使用 run_scan_loop() 替代
     */
    void run_cpi_loop();

    /**
     * @brief 处理扫描完成事件
     * @param scan_index 扫描索引
     * @param cpi_echos CPI 回波数据
     * @param target_snapshots 目标快照
     */
    void process_scan_complete(int scan_index,
                               const std::vector<CpiEcho>& cpi_echos,
                               const std::vector<TargetList>& target_snapshots);

    // 配置和状态
    RadarConfig config_;                        ///< 雷达配置
    std::atomic<bool> running_{false};          ///< 是否正在运行
    std::atomic<bool> stop_requested_{false};   ///< 是否请求停止
    bool initialized_ = false;                  ///< 是否已初始化

    int current_scan_index_ = 0;                ///< 当前扫描索引
    int total_scan_count_ = 0;                  ///< 总扫描数
    int total_cpi_count_ = 0;                   ///< 总 CPI 数
    int beam_count_ = 0;                        ///< 每圈扫描的波位数
    int cpi_counter_ = 0;                       ///< 当前 CPI 计数器

    std::string last_error_;                    ///< 最近错误信息
    mutable std::mutex mutex_;                  ///< 保护共享状态

    std::thread run_thread_;                    ///< 异步运行线程

    // 子引擎
    std::unique_ptr<waveform::WaveformGenerator> waveform_gen_;           ///< 波形生成器
    std::unique_ptr<noise::NoiseEngine> noise_engine_;          ///< 噪声引擎
    std::unique_ptr<clutter::ClutterEngine> clutter_engine_;    ///< 杂波引擎
    std::unique_ptr<target::TargetEngine> target_engine_;       ///< 目标引擎（统一管理器）
    std::unique_ptr<antenna::BeamScanner> beam_scanner_;        ///< 波束扫描控制器
    std::unique_ptr<core::UdpSender> udp_sender_;               ///< UDP 发送器
    std::unique_ptr<core::DataExporter> data_exporter_;         ///< 数据导出器

    // 初始目标配置（用于导出）
    TargetList initial_targets_;                                ///< 初始目标列表
};

}  // namespace radar
