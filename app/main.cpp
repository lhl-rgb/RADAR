/**
 * @file main.cpp
 * @brief 雷达仿真主程序 - 服务器端模式
 * @details
 * 服务器端模式：所有配置通过 JSON 文件加载，不在此代码中硬编码。
 * 支持：
 * - 配置加载/验证
 * - 仿真引擎初始化与运行
 * - 数据导出（IQ 数据、目标轨迹、配置参数）
 * - UDP 输出（发送到信号处理机）
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <filesystem>
#include <string>
#include <csignal>
#include <atomic>

#include "core/configuration_manager.h"
#include "core/simulation_engine.h"

namespace {

std::atomic<bool> g_stop_requested{false};

void signal_handler(int signum) {
    SPDLOG_WARN("Received signal {}, stopping simulation...", signum);
    g_stop_requested = true;
}

}  // namespace

int main() {
    // 初始化 spdlog - 同时输出到控制台和文件
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);

    std::vector<spdlog::sink_ptr> sinks{console_sink};

    // 如果日志目录存在，添加文件 sink
    const std::string log_dir = "out/log";
    if (std::filesystem::exists(log_dir) || std::filesystem::create_directories(log_dir)) {
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_dir + "/radar_sim.log", true);
        file_sink->set_level(spdlog::level::debug);
        sinks.push_back(file_sink);
    }

    auto logger = std::make_shared<spdlog::logger>("radar", sinks.begin(), sinks.end());
    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

    // 注册信号处理器
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    SPDLOG_INFO("╔════════════════════════════════════════╗");
    SPDLOG_INFO("║   Radar Echo Simulator (Server Mode)   ║");
    SPDLOG_INFO("╚════════════════════════════════════════╝");

    // ========== 1. 配置初始化 ==========
    radar::ConfigurationManager config_manager;

    // 获取可执行文件所在目录，向上找到项目根目录
    // 可执行文件路径：project_root/out/bin/radar
    std::filesystem::path exe_path = std::filesystem::canonical("/proc/self/exe");
    std::filesystem::path project_root = exe_path.parent_path().parent_path().parent_path();
    std::string config_path = (project_root / "config" / "radar_config.json").string();

    if (!std::filesystem::exists(config_path)) {
        SPDLOG_ERROR("Config file not found: {}", config_path);
        SPDLOG_ERROR("Please create config file or specify path via command line.");
        return 1;
    }

    if (!config_manager.load_from_json(config_path)) {
        SPDLOG_ERROR("Failed to load config: {}", config_manager.last_error());
        return 1;
    }

    SPDLOG_INFO("Config loaded from: {}", config_path);

    radar::RadarConfig& config = config_manager.mutable_config();
    config.compute_derived_params();

    // 配置验证
    std::string error;
    if (!config.validate(error)) {
        SPDLOG_ERROR("Configuration validation failed: {}", error);
        return 1;
    }

    // 打印配置摘要
    config.print();

    // ========== 2. 创建仿真引擎 ==========
    radar::SimulationEngine engine(config);

    // ========== 3. 初始化引擎 ==========
    if (!engine.initialize()) {
        SPDLOG_ERROR("[SimulationEngine] Initialization failed: {}", engine.last_error());
        return 1;
    }

    SPDLOG_INFO("Simulation initialized: {} beams/scan, {} CPIs total",
                engine.beam_count(), engine.total_cpi_count());

    // ========== 4. 运行仿真 ==========
    SPDLOG_INFO("Starting simulation...");

    // 同步运行模式 - 阻塞直到仿真完成
    engine.run(0);

    SPDLOG_INFO("Simulation finished successfully.");

    return 0;
}
