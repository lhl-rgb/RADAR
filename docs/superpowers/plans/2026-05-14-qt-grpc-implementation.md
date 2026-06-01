# Qt + gRPC Radar Simulator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a gRPC-based configuration and control layer with a Qt desktop GUI to the existing radar simulator.

**Architecture:** A single `RadarServiceImpl` wraps `SimulationEngine` + `ConfigurationManager` and is exposed as a gRPC server (either embedded in the Qt app or running standalone as `radar_server`). The Qt UI always talks gRPC — local mode embeds an in-process server on a background thread, remote mode connects to an external `radar_server`. The existing CLI `radar_app` is preserved unchanged for headless batch usage.

**Tech Stack:** C++17, gRPC, Protobuf, Qt6 (fallback Qt5), existing nlohmann_json/SimulationEngine/RadarConfig

---

### Task 1: Proto definition and CMake infrastructure

**Files:**
- Create: `include/grpc/radar_service.proto`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the proto file**

```protobuf
syntax = "proto3";

package radar;

service RadarService {
    rpc SetConfig(RadarConfig) returns (Status);
    rpc GetConfig(Empty) returns (RadarConfig);
    rpc ValidateConfig(RadarConfig) returns (ValidationResult);
    rpc StartSimulation(Empty) returns (Status);
    rpc StopSimulation(Empty) returns (Status);
    rpc GetStatus(Empty) returns (SimStatus);
}

message Empty {}

message RadarConfig {
    string json_content = 1;
}

message Status {
    bool success = 1;
    string message = 2;
}

message ValidationResult {
    bool valid = 1;
    repeated string errors = 2;
}

message SimStatus {
    enum State {
        IDLE = 0;
        RUNNING = 1;
        STOPPED = 2;
        FINISHED = 3;
    }
    State state = 1;
    double progress = 2;
    int32 current_scan = 3;
    int32 total_scans = 4;
}
```

- [ ] **Step 2: Add gRPC, Protobuf, Qt dependencies to CMakeLists.txt**

Insert after the existing `find_package(GDAL REQUIRED)` block (around line 55):

```cmake
# ============================================================
# gRPC + Protobuf
# ============================================================
find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

# ============================================================
# Qt (GUI only, optional when building standalone server)
# ============================================================
find_package(QT NAMES Qt6 Qt5 QUIET COMPONENTS Widgets)
if(QT_FOUND)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
    set(RADAR_QT_FOUND ON)
endif()
```

- [ ] **Step 3: Add proto generation rules and new targets**

After the `radar_tests` section, add:

```cmake
# ============================================================
# gRPC service library
# ============================================================
set(RADAR_PROTO "${CMAKE_CURRENT_SOURCE_DIR}/include/grpc/radar_service.proto")
set(RADAR_PROTO_BUILD "${CMAKE_BINARY_DIR}/grpc")

file(MAKE_DIRECTORY "${RADAR_PROTO_BUILD}")

protobuf_generate(
    PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/grpc"
    OUT_VAR _proto_srcs
)
protobuf_generate(
    PATH "${CMAKE_CURRENT_SOURCE_DIR}/include/grpc"
    LANGUAGE grpc
    OUT_VAR _grpc_srcs
)

add_library(radar_grpc_service STATIC
    src/grpc/radar_service.cpp
    ${_proto_srcs}
    ${_grpc_srcs}
)
target_include_directories(radar_grpc_service
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_BINARY_DIR}"
)
target_link_libraries(radar_grpc_service
    PUBLIC
        radar::modules
        gRPC::grpc++
        protobuf::libprotobuf
)

# ============================================================
# Standalone gRPC server (headless, remote mode)
# ============================================================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/radar_server.cpp")
    add_executable(radar_server app/radar_server.cpp)
    target_link_libraries(radar_server PRIVATE radar_grpc_service radar::modules gRPC::grpc++)
endif()

# ============================================================
# Qt GUI application (with embedded gRPC server)
# ============================================================
if(RADAR_QT_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/radar_qt.cpp")
    add_executable(radar_qt app/radar_qt.cpp)
    target_link_libraries(radar_qt PRIVATE radar_grpc_service radar::modules
                          gRPC::grpc++ Qt${QT_VERSION_MAJOR}::Widgets)
endif()
```

- [ ] **Step 4: Verify CMake configuration**

Run: `cmake -B build -DCMAKE_BUILD_TYPE=Debug 2>&1 | head -50`
Expected: gRPC/Protobuf/Qt found, no errors. If gRPC or Qt aren't installed, install them:
```bash
sudo apt install protobuf-compiler libprotobuf-dev libgrpc++-dev libgrpc-dev protobuf-compiler-grpc
sudo apt install qt6-base-dev  # or qtbase5-dev for Qt5
```

- [ ] **Step 5: Commit**

```bash
git add include/grpc/radar_service.proto CMakeLists.txt
git commit -m "feat(grpc): add proto definition and CMake infrastructure for gRPC + Qt"
```

---

### Task 2: Implement RadarServiceImpl

**Files:**
- Create: `include/grpc/radar_service.h`
- Create: `src/grpc/radar_service.cpp`

- [ ] **Step 1: Create the header**

```cpp
#pragma once

#include "core/radar_config.h"
#include "core/simulation_engine.h"
#include "grpc/radar_service.grpc.pb.h"

#include <grpcpp/grpcpp.h>

#include <memory>
#include <mutex>
#include <string>

namespace radar {

class RadarServiceImpl final : public RadarService::Service {
public:
    RadarServiceImpl() = default;

    grpc::Status SetConfig(grpc::ServerContext* context,
                           const RadarConfig* request,
                           Status* reply) override;

    grpc::Status GetConfig(grpc::ServerContext* context,
                           const Empty* request,
                           RadarConfig* reply) override;

    grpc::Status ValidateConfig(grpc::ServerContext* context,
                                const RadarConfig* request,
                                ValidationResult* reply) override;

    grpc::Status StartSimulation(grpc::ServerContext* context,
                                 const Empty* request,
                                 Status* reply) override;

    grpc::Status StopSimulation(grpc::ServerContext* context,
                                const Empty* request,
                                Status* reply) override;

    grpc::Status GetStatus(grpc::ServerContext* context,
                           const Empty* request,
                           SimStatus* reply) override;

private:
    std::mutex mutex_;
    RadarConfig current_config_;
    std::unique_ptr<SimulationEngine> engine_;
};

/// Helper: start a gRPC server on a given address and return the server + service.
/// The caller must keep `service` alive as long as the server runs.
std::unique_ptr<grpc::Server> StartGrpcServer(
    const std::string& server_address,
    RadarServiceImpl* service);

}  // namespace radar
```

- [ ] **Step 2: Create the implementation**

```cpp
#include "grpc/radar_service.h"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <fstream>

namespace radar {

// ============================================================
// SetConfig
// ============================================================
grpc::Status RadarServiceImpl::SetConfig(
    grpc::ServerContext*, const RadarConfig* request, Status* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    // If simulation is running, reject
    if (engine_ && engine_->is_running()) {
        reply->set_success(false);
        reply->set_message("Cannot change config while simulation is running");
        return grpc::Status::OK;
    }

    try {
        const std::string& json_str = request->json_content();
        nlohmann::json j = nlohmann::json::parse(json_str);

        RadarConfig new_config = j.get<RadarConfig>();
        std::string error;
        if (!new_config.validate(error)) {
            reply->set_success(false);
            reply->set_message("Validation failed: " + error);
            return grpc::Status::OK;
        }

        current_config_ = new_config;

        // Destroy old engine and create new one with new config
        engine_ = std::make_unique<SimulationEngine>(current_config_);
        if (!engine_->initialize()) {
            reply->set_success(false);
            reply->set_message("Engine init failed: " + engine_->last_error());
            engine_.reset();
            return grpc::Status::OK;
        }

        SPDLOG_INFO("gRPC: config updated successfully");
        reply->set_success(true);
        reply->set_message("Config updated and engine initialized");
        return grpc::Status::OK;

    } catch (const std::exception& e) {
        reply->set_success(false);
        reply->set_message(std::string("JSON parse error: ") + e.what());
        return grpc::Status::OK;
    }
}

// ============================================================
// GetConfig
// ============================================================
grpc::Status RadarServiceImpl::GetConfig(
    grpc::ServerContext*, const Empty*, RadarConfig* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    nlohmann::json j;
    j["simulation"] = current_config_.simulation;
    j["system"] = current_config_.system;
    j["waveform"] = current_config_.waveform;
    j["antenna"] = current_config_.antenna;
    j["beam_table"] = current_config_.beam_table;
    j["noise"] = current_config_.noise;
    j["clutter"] = current_config_.clutter;
    j["target"] = current_config_.target;
    j["data_export"] = current_config_.data_export;
    j["udp_output"] = current_config_.udp_output;
    // initial_targets
    j["initial_targets"] = nlohmann::json::array();
    for (const auto& t : current_config_.initial_targets) {
        nlohmann::json tj;
        to_json(tj, t);
        j["initial_targets"].push_back(tj);
    }

    reply->set_json_content(j.dump(2));
    return grpc::Status::OK;
}

// ============================================================
// ValidateConfig
// ============================================================
grpc::Status RadarServiceImpl::ValidateConfig(
    grpc::ServerContext*, const RadarConfig* request, ValidationResult* reply) {

    try {
        const std::string& json_str = request->json_content();
        nlohmann::json j = nlohmann::json::parse(json_str);
        RadarConfig cfg = j.get<RadarConfig>();

        std::string error;
        if (cfg.validate(error)) {
            reply->set_valid(true);
        } else {
            reply->set_valid(false);
            reply->add_errors(error);
        }
    } catch (const std::exception& e) {
        reply->set_valid(false);
        reply->add_errors(std::string("JSON parse error: ") + e.what());
    }

    return grpc::Status::OK;
}

// ============================================================
// StartSimulation
// ============================================================
grpc::Status RadarServiceImpl::StartSimulation(
    grpc::ServerContext*, const Empty*, Status* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    if (!engine_) {
        reply->set_success(false);
        reply->set_message("No config set — call SetConfig first");
        return grpc::Status::OK;
    }

    if (engine_->is_running()) {
        reply->set_success(false);
        reply->set_message("Simulation already running");
        return grpc::Status::OK;
    }

    engine_->run_async(0);
    SPDLOG_INFO("gRPC: simulation started");
    reply->set_success(true);
    reply->set_message("Simulation started");
    return grpc::Status::OK;
}

// ============================================================
// StopSimulation
// ============================================================
grpc::Status RadarServiceImpl::StopSimulation(
    grpc::ServerContext*, const Empty*, Status* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    if (!engine_ || !engine_->is_running()) {
        reply->set_success(false);
        reply->set_message("No simulation running");
        return grpc::Status::OK;
    }

    engine_->stop();
    SPDLOG_INFO("gRPC: simulation stop requested");
    reply->set_success(true);
    reply->set_message("Stop requested");
    return grpc::Status::OK;
}

// ============================================================
// GetStatus
// ============================================================
grpc::Status RadarServiceImpl::GetStatus(
    grpc::ServerContext*, const Empty*, SimStatus* reply) {

    std::lock_guard<std::mutex> lock(mutex_);

    if (!engine_) {
        reply->set_state(SimStatus::IDLE);
        reply->set_progress(0.0);
        reply->set_current_scan(0);
        reply->set_total_scans(0);
        return grpc::Status::OK;
    }

    if (engine_->is_running()) {
        reply->set_state(SimStatus::RUNNING);
    } else if (engine_->stop_requested()) {
        reply->set_state(SimStatus::STOPPED);
    } else {
        // If engine exists but not running and not stopped, it finished
        reply->set_state(SimStatus::FINISHED);
    }

    reply->set_progress(engine_->progress_percent() / 100.0);
    reply->set_current_scan(engine_->current_scan_index());
    reply->set_total_scans(engine_->total_scan_count());
    return grpc::Status::OK;
}

// ============================================================
// StartGrpcServer helper
// ============================================================
std::unique_ptr<grpc::Server> StartGrpcServer(
    const std::string& server_address,
    RadarServiceImpl* service) {

    grpc::ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(service);
    return builder.BuildAndStart();
}

}  // namespace radar
```

- [ ] **Step 3: Commit**

```bash
git add include/grpc/radar_service.h src/grpc/radar_service.cpp
git commit -m "feat(grpc): implement RadarServiceImpl wrapping SimulationEngine"
```

---

### Task 3: Standalone gRPC server binary

**Files:**
- Create: `app/radar_server.cpp`

- [ ] **Step 1: Create radar_server.cpp**

```cpp
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
```

- [ ] **Step 2: Build and verify**

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target radar_server -j$(nproc)
ls out/bin/radar_server
```

Expected: `radar_server` binary built successfully.

- [ ] **Step 3: Quick smoke test**

```bash
# Start server in background
./out/bin/radar_server &
SERVER_PID=$!
sleep 1

# Use grpcurl to list services
grpcurl -plaintext localhost:50051 list

# Kill server
kill $SERVER_PID 2>/dev/null; wait $SERVER_PID 2>/dev/null
```

Expected: `grpcurl` prints `radar.RadarService`.

- [ ] **Step 4: Commit**

```bash
git add app/radar_server.cpp
git commit -m "feat(grpc): add standalone radar_server binary"
```

---

### Task 4: Qt GUI application

**Files:**
- Create: `app/radar_qt.cpp`

- [ ] **Step 1: Create the Qt application with embedded gRPC server**

```cpp
/**
 * @file radar_qt.cpp
 * @brief Qt GUI for radar simulator (local or remote)
 *
 * Usage:
 *   ./radar_qt                # Local mode (embedded server on localhost:50051)
 *   ./radar_qt --remote 192.168.1.100:50051  # Remote mode
 *   ./radar_qt --port 50052  # Override local port
 */

#include <grpcpp/grpcpp.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QTimer>
#include <QTextStream>
#include <QFile>
#include <QGroupBox>
#include <QThread>

#include <memory>
#include <string>
#include <fstream>
#include <sstream>

#include "grpc/radar_service.h"
#include "grpc/radar_service.grpc.pb.h"

// ============================================================
// Helpers
// ============================================================

/// Load file contents into a string.
static std::string load_file(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs) return {};
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return oss.str();
}

static bool save_file(const std::string& path, const std::string& content) {
    std::ofstream ofs(path);
    if (!ofs) return false;
    ofs << content;
    return true;
}

// ============================================================
// Main Window
// ============================================================

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(const std::string& server_addr, bool local_mode,
               QWidget* parent = nullptr)
        : QMainWindow(parent), server_addr_(server_addr), local_mode_(local_mode) {

        setWindowTitle("Radar Simulator Control");
        resize(800, 600);

        // Central widget
        auto* central = new QWidget(this);
        auto* main_layout = new QVBoxLayout(central);
        setCentralWidget(central);

        // --- Connection info ---
        auto* info_label = new QLabel(
            local_mode_ ? "Mode: Local (embedded server)"
                        : QString("Mode: Remote  (%1)").arg(server_addr.c_str()),
            this);
        info_label->setStyleSheet("font-weight: bold; padding: 4px;");
        main_layout->addWidget(info_label);

        // --- Config editor ---
        auto* config_group = new QGroupBox("Configuration (JSON)", this);
        auto* config_layout = new QVBoxLayout(config_group);

        editor_ = new QPlainTextEdit(this);
        editor_->setFont(QFont("Courier", 10));
        editor_->setPlaceholderText("Paste radar JSON config here...");
        config_layout->addWidget(editor_);

        auto* config_btn_row = new QHBoxLayout();
        auto* load_btn = new QPushButton("Load File", this);
        auto* save_btn = new QPushButton("Save File", this);
        auto* apply_btn = new QPushButton("Apply Config", this);
        auto* validate_btn = new QPushButton("Validate", this);
        config_btn_row->addWidget(load_btn);
        config_btn_row->addWidget(save_btn);
        config_btn_row->addWidget(apply_btn);
        config_btn_row->addWidget(validate_btn);
        config_layout->addLayout(config_btn_row);
        main_layout->addWidget(config_group);

        // --- Control ---
        auto* control_group = new QGroupBox("Simulation Control", this);
        auto* control_layout = new QHBoxLayout(control_group);

        start_btn_ = new QPushButton("Start", this);
        stop_btn_ = new QPushButton("Stop", this);
        stop_btn_->setEnabled(false);

        progress_bar_ = new QProgressBar(this);
        status_label_ = new QLabel("Idle", this);

        control_layout->addWidget(start_btn_);
        control_layout->addWidget(stop_btn_);
        control_layout->addWidget(progress_bar_, 1);
        control_layout->addWidget(status_label_);
        main_layout->addWidget(control_group);

        // --- Connections ---
        connect(load_btn, &QPushButton::clicked, this, &MainWindow::on_load);
        connect(save_btn, &QPushButton::clicked, this, &MainWindow::on_save);
        connect(apply_btn, &QPushButton::clicked, this, &MainWindow::on_apply_config);
        connect(validate_btn, &QPushButton::clicked, this, &MainWindow::on_validate);
        connect(start_btn_, &QPushButton::clicked, this, &MainWindow::on_start);
        connect(stop_btn_, &QPushButton::clicked, this, &MainWindow::on_stop);

        // --- Setup gRPC channel ---
        connect(this, &MainWindow::started, this, [this]() {
            start_btn_->setEnabled(false);
            stop_btn_->setEnabled(true);
        });
        connect(this, &MainWindow::stopped, this, [this]() {
            start_btn_->setEnabled(true);
            stop_btn_->setEnabled(false);
        });

        auto channel = grpc::CreateChannel(server_addr_,
                                           grpc::InsecureChannelCredentials());
        stub_ = radar::RadarService::NewStub(channel);

        // --- Status polling timer ---
        auto* timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &MainWindow::poll_status);
        timer->start(1000);
    }

signals:
    void started();
    void stopped();

private slots:
    void on_load() {
        QString path = QFileDialog::getOpenFileName(this, "Load Config",
                                                     "", "JSON (*.json)");
        if (path.isEmpty()) return;
        std::string content = load_file(path.toStdString());
        if (content.empty()) {
            QMessageBox::warning(this, "Error", "Failed to read file");
            return;
        }
        editor_->setPlainText(QString::fromStdString(content));
    }

    void on_save() {
        QString path = QFileDialog::getSaveFileName(this, "Save Config",
                                                     "", "JSON (*.json)");
        if (path.isEmpty()) return;
        if (!save_file(path.toStdString(), editor_->toPlainText().toStdString())) {
            QMessageBox::warning(this, "Error", "Failed to write file");
        }
    }

    void on_apply_config() {
        radar::RadarConfig request;
        request.set_json_content(editor_->toPlainText().toStdString());

        radar::Status reply;
        grpc::ClientContext ctx;
        auto status = stub_->SetConfig(&ctx, request, &reply);

        if (!status.ok()) {
            QMessageBox::critical(this, "gRPC Error",
                                  QString::fromStdString(status.error_message()));
            return;
        }

        if (!reply.success()) {
            QMessageBox::warning(this, "Config Error",
                                 QString::fromStdString(reply.message()));
            return;
        }

        status_label_->setText("Config applied");
    }

    void on_validate() {
        radar::RadarConfig request;
        request.set_json_content(editor_->toPlainText().toStdString());

        radar::ValidationResult reply;
        grpc::ClientContext ctx;
        stub_->ValidateConfig(&ctx, request, &reply);

        if (reply.valid()) {
            QMessageBox::information(this, "Validation", "Configuration is valid");
        } else {
            QString msg = "Validation errors:\n";
            for (const auto& e : reply.errors()) {
                msg += "  - " + QString::fromStdString(e) + "\n";
            }
            QMessageBox::warning(this, "Validation Failed", msg);
        }
    }

    void on_start() {
        radar::Empty request;
        radar::Status reply;
        grpc::ClientContext ctx;
        auto status = stub_->StartSimulation(&ctx, request, &reply);

        if (!status.ok()) {
            QMessageBox::critical(this, "gRPC Error",
                                  QString::fromStdString(status.error_message()));
            return;
        }

        if (!reply.success()) {
            QMessageBox::warning(this, "Start Error",
                                 QString::fromStdString(reply.message()));
            return;
        }

        emit started();
    }

    void on_stop() {
        radar::Empty request;
        radar::Status reply;
        grpc::ClientContext ctx;
        stub_->StopSimulation(&ctx, request, &reply);
        emit stopped();
    }

    void poll_status() {
        radar::Empty request;
        radar::SimStatus reply;
        grpc::ClientContext ctx;
        auto status = stub_->GetStatus(&ctx, request, &reply);

        if (!status.ok()) {
            status_label_->setText("Disconnected");
            progress_bar_->setValue(0);
            return;
        }

        // State label
        static const char* state_names[] = {"Idle", "Running", "Stopped", "Finished"};
        int s = reply.state();
        const char* state_name = (s >= 0 && s < 4) ? state_names[s] : "Unknown";
        status_label_->setText(state_name);

        // Progress
        progress_bar_->setValue(static_cast<int>(reply.progress() * 100));

        // Toggle buttons based on state
        if (reply.state() == radar::SimStatus::RUNNING) {
            start_btn_->setEnabled(false);
            stop_btn_->setEnabled(true);
        } else {
            start_btn_->setEnabled(true);
            stop_btn_->setEnabled(false);
        }
    }

private:
    std::string server_addr_;
    bool local_mode_;
    std::unique_ptr<radar::RadarService::Stub> stub_;

    QPlainTextEdit* editor_ = nullptr;
    QPushButton* start_btn_ = nullptr;
    QPushButton* stop_btn_ = nullptr;
    QProgressBar* progress_bar_ = nullptr;
    QLabel* status_label_ = nullptr;
};

// ============================================================
// main
// ============================================================

namespace {
    std::unique_ptr<grpc::Server> g_grpc_server;
    radar::RadarServiceImpl* g_grpc_service = nullptr;

    void cleanup_grpc() {
        if (g_grpc_server) {
            g_grpc_server->Shutdown();
            g_grpc_server.reset();
        }
        delete g_grpc_service;
        g_grpc_service = nullptr;
    }
}

int main(int argc, char* argv[]) {
    std::string remote_addr;
    int port = 50051;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--remote" && i + 1 < argc) {
            remote_addr = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            port = std::stoi(argv[++i]);
        }
    }

    bool local_mode = remote_addr.empty();
    std::string server_addr;

    if (local_mode) {
        server_addr = "localhost:" + std::to_string(port);

        g_grpc_service = new radar::RadarServiceImpl();
        g_grpc_server = radar::StartGrpcServer(server_addr, g_grpc_service);
        SPDLOG_INFO("Embedded gRPC server started on {}", server_addr);

        qAddPostRoutine(cleanup_grpc);
    } else {
        server_addr = remote_addr;
    }

    QApplication app(argc, argv);
    MainWindow window(server_addr, local_mode);
    window.show();

    return app.exec();
}

#include "radar_qt.moc"
```

Note: `#include "radar_qt.moc"` at the end is required because we define Q_OBJECT in a `.cpp` file. CMake's AUTOMOC handles the `.moc` generation.

- [ ] **Step 2: Enable AUTOMOC for the Qt target in CMakeLists.txt**

Replace the Qt target section with:

```cmake
if(RADAR_QT_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/radar_qt.cpp")
    add_executable(radar_qt app/radar_qt.cpp)
    set_target_properties(radar_qt PROPERTIES AUTOMOC ON)
    target_link_libraries(radar_qt PRIVATE radar_grpc_service radar::modules
                          gRPC::grpc++ Qt${QT_VERSION_MAJOR}::Widgets)
endif()
```

- [ ] **Step 3: Build**

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target radar_qt -j$(nproc)
ls out/bin/radar_qt
```

Expected: `radar_qt` binary built successfully. If Qt not found, verify Qt dev packages are installed.

- [ ] **Step 4: Commit**

```bash
git add app/radar_qt.cpp CMakeLists.txt
git commit -m "feat(qt): add Qt GUI application with embedded gRPC server"
```

---

### Task 5: Update build script and final touches

**Files:**
- Modify: `build.sh`

- [ ] **Step 1: Add Qt/gRPC server build targets to build.sh**

Replace the help text section:

```bash
echo "编译模式 (默认：--all):"
echo "  --main       只编译主程序 (radar, matlab_export_tool)"
echo "  --validate   只编译验证程序 (waveform_validation, antenna_validation, noise_validation, clutter_validation)"
echo "  --qt         编译 Qt GUI (radar_qt)"
echo "  --server     编译独立 gRPC server (radar_server)"
echo "  --all        编译全部目标"
```

Add corresponding case in the BUILD_MODE switch:

```bash
qt)
    CMAKE_OPTS="${CMAKE_OPTS} -DRADAR_BUILD_QT=ON"
    echo "→ 编译 Qt GUI..."
    ;;
server)
    CMAKE_OPTS="${CMAKE_OPTS} -DRADAR_BUILD_SERVER=ON"
    echo "→ 编译 gRPC Server..."
    ;;
```

And at the bottom of CMakeLists.txt, add options:

```cmake
option(RADAR_BUILD_QT "Build Qt GUI" OFF)
option(RADAR_BUILD_SERVER "Build standalone gRPC server" OFF)
```

Then wrap the Qt and server target creation in conditionals:

```cmake
if(RADAR_BUILD_SERVER OR (NOT RADAR_BUILD_QT AND NOT RADAR_BUILD_SERVER AND NOT RADAR_BUILD_MAIN_ONLY AND NOT RADAR_BUILD_VALIDATE_ONLY))
    # radar_server target...
endif()

if((RADAR_BUILD_QT OR (NOT RADAR_BUILD_QT AND NOT RADAR_BUILD_SERVER AND NOT RADAR_BUILD_MAIN_ONLY AND NOT RADAR_BUILD_VALIDATE_ONLY)) AND RADAR_QT_FOUND)
    # radar_qt target...
endif()
```

Simpler approach: always build both when no --main/--validate/--qt/--server flag is given:

```cmake
# Replace the standalone server target section with:
if(RADAR_BUILD_SERVER OR NOT (RADAR_BUILD_MAIN_ONLY OR RADAR_BUILD_VALIDATE_ONLY OR RADAR_BUILD_QT))
    add_executable(radar_server app/radar_server.cpp)
    target_link_libraries(radar_server PRIVATE radar_grpc_service gRPC::grpc++)
endif()

# Replace the Qt target section with:
if(RADAR_QT_FOUND AND (RADAR_BUILD_QT OR NOT (RADAR_BUILD_MAIN_ONLY OR RADAR_BUILD_VALIDATE_ONLY OR RADAR_BUILD_SERVER)))
    add_executable(radar_qt app/radar_qt.cpp)
    set_target_properties(radar_qt PROPERTIES AUTOMOC ON)
    target_link_libraries(radar_qt PRIVATE radar_grpc_service
                          gRPC::grpc++ Qt${QT_VERSION_MAJOR}::Widgets)
endif()
```

- [ ] **Step 2: Full build test**

```bash
./build.sh --all
```

Expected: all existing targets + `radar_server` build. `radar_qt` builds if Qt dev packages are installed.

- [ ] **Step 3: Commit**

```bash
git add CMakeLists.txt build.sh
git commit -m "build: add radar_server and radar_qt build support"
```

---

## Self-Review Checklist

- [ ] Every proto message has a corresponding handler in `RadarServiceImpl`
- [ ] Thread safety: `mutex_` protects all access to `engine_` and `current_config_`
- [ ] SimulationEngine lifecycle: destroyed + recreated on SetConfig, not modified in place
- [ ] Qt UI only ever talks through gRPC stub — no direct engine access
- [ ] Existing `radar_app` CLI binary is untouched
- [ ] MOC setup correct for `radar_qt.cpp` (AUTOMOC ON + `#include "radar_qt.moc"`)
- [ ] All gRPC error paths return `grpc::Status::OK` with `success=false` (no transport-level failures for business logic errors)
