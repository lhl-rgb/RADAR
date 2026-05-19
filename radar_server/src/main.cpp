/**
 * @file radar_server.cpp
 * @brief Standalone gRPC radar simulation server
 *
 * Usage: ./radar_server [--port 50051]
 * Runs a headless gRPC server exposing RadarService for remote clients.
 */

#include <grpcpp/grpcpp.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <csignal>
#include <atomic>
#include <string>
#include <thread>

#include "grpc/radar_service.h"

namespace {

std::atomic<bool> g_shutdown{false};

void signal_handler(int) {
    g_shutdown = true;
}

}  // namespace

int main(int argc, char* argv[]) {
    // Default port
    int port = 50051;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = std::stoi(argv[++i]);
        }
    }

    // Logger
    auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto logger = std::make_shared<spdlog::logger>("radar_server", console);
    spdlog::set_default_logger(logger);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    spdlog::set_level(spdlog::level::info);

    // Signal handling
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string server_address = "0.0.0.0:" + std::to_string(port);

    radar::RadarServiceImpl service;
    auto server = radar::StartGrpcServer(server_address, &service);

    SPDLOG_INFO("radar_server listening on {}", server_address);
    SPDLOG_INFO("Press Ctrl+C to stop");

    // Wait for shutdown signal
    while (!g_shutdown) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    SPDLOG_INFO("Shutting down...");
    server->Shutdown();
    SPDLOG_INFO("Server stopped");

    return 0;
}
