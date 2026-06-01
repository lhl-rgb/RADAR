# Qt + gRPC Radar Simulator Architecture Design

## 1. Overview

Add a gRPC-based configuration and control layer to the existing radar simulator, wrapped by a Qt desktop application. The same Qt binary works in both local and remote modes via startup parameters, using a single gRPC interface for all interactions.

### Design Tenets

- **Single call path** — UI always talks gRPC, whether the server is in-process or remote
- **No raw echo display** — initial Qt UI covers only config + run control + progress status
- **No target streaming** — results output stays file-based (existing `.dat`/`.csv` export)
- **Minimal proto surface** — just config, control, and status polling

---

## 2. Architecture

```
┌─────────────────────────────────────────────────────┐
│                 Qt Application                        │
│                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐   │
│  │  配置面板  │  │  运行控制  │  │  状态/进度显示    │   │
│  └─────┬────┘  └────┬─────┘  └───────┬──────────┘   │
│        │             │               │              │
│  ┌─────┴─────────────┴───────────────┴──────────┐   │
│  │           gRPC Client (唯一调用路径)           │   │
│  │         target: localhost:50051               │   │
│  │            or --remote <addr>                 │   │
│  └─────────────────────┬────────────────────────┘   │
│                        │                             │
│  ┌─────────────────────▼────────────────────────┐   │
│  │         gRPC Server (内嵌，本地模式)            │   │
│  │        或独立进程 radar_server (远程模式)        │   │
│  │  ┌────────────────────────────────────────┐   │   │
│  │  │  RadarService (gRPC handler → 模拟器)    │   │   │
│  │  └────────────────────────────────────────┘   │   │
│  │  ┌────────────────────────────────────────┐   │   │
│  │  │  模拟器核心 (libradar 静态库)             │   │   │
│  │  │  ConfigurationManager + SimulationEngine │   │   │
│  │  └────────────────────────────────────────┘   │   │
│  └──────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
```

### Mode Switching

| Mode | Qt flag | Server location | gRPC target |
|------|---------|-----------------|-------------|
| Local | (default) | In-process, background thread | `localhost:50051` |
| Remote | `--remote <host:port>` | External `radar_server` process | user-specified |
| Port override | `--port 50052` | Overrides default 50051 (local mode) | `localhost:<port>` |

### File Layout Additions

```
app/
  radar_server.cpp        # Standalone gRPC server binary (remote mode)
include/
  grpc/
    radar_service.h       # C++ gRPC service handler declaration
    radar_service.proto   # Protobuf service definition
src/
  grpc/
    radar_service.cpp     # C++ gRPC service handler implementation
    radar_service.grpc.pb.cc  # Generated gRPC code
    radar_service.pb.cc       # Generated protobuf code
```

---

## 3. gRPC Interface

```protobuf
syntax = "proto3";

package radar;

service RadarService {
    // Set the simulation configuration (full JSON payload)
    rpc SetConfig(RadarConfig) returns (Status);

    // Get the current configuration
    rpc GetConfig(Empty) returns (RadarConfig);

    // Validate a configuration without applying it
    rpc ValidateConfig(RadarConfig) returns (ValidationResult);

    // Start the simulation run
    rpc StartSimulation(Empty) returns (Status);

    // Stop a running simulation
    rpc StopSimulation(Empty) returns (Status);

    // Poll current simulation status/progress
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

### Design Notes

- **Config as JSON string** — reuses existing `RadarConfig` serialization; avoids duplicating every field in proto
- **Status polling** — Qt side polls `GetStatus` at ~1 Hz; no streaming complexity
- **Synchronous control** — `StartSimulation` kicks off the engine on a server-side thread and returns immediately; progress is tracked via `GetStatus`
- **No streaming, no event push** — keeps the initial implementation simple

---

## 4. Server: Dual Mode (Embedded + Standalone)

The same `RadarServiceImpl` class is used in both modes.

### Embedded (Local Mode)

- Qt starts a `std::thread` running the gRPC server on `localhost:50051`
- Single process: no serialization overhead for config data
- Server thread shares the same `SimulationEngine` instance
- Qt's gRPC client connects to `localhost:50051`

### Standalone (Remote Mode)

- `radar_server` binary: links `radar::modules` + gRPC libraries
- Listens on `0.0.0.0:50051`
- Independently deployable headless process
- Can be managed via systemd or containerized

### Service Handler Skeleton

```cpp
class RadarServiceImpl final : public radar::RadarService::Service {
public:
    explicit RadarServiceImpl(SimulationEngine& engine);

    grpc::Status SetConfig(grpc::ServerContext*, const RadarConfig* request, Status* reply) override;
    grpc::Status GetConfig(grpc::ServerContext*, const Empty*, RadarConfig* reply) override;
    grpc::Status ValidateConfig(grpc::ServerContext*, const RadarConfig*, ValidationResult* reply) override;
    grpc::Status StartSimulation(grpc::ServerContext*, const Empty*, Status* reply) override;
    grpc::Status StopSimulation(grpc::ServerContext*, const Empty*, Status* reply) override;
    grpc::Status GetStatus(grpc::ServerContext*, const Empty*, SimStatus* reply) override;

private:
    SimulationEngine& engine_;
};
```

---

## 5. Qt Application Structure

### UI Components (Minimal v1)

| Component | Maps to gRPC call | Notes |
|-----------|-------------------|-------|
| Config panel (load/save JSON) | `SetConfig` / `GetConfig` | Probably a text editor or file-picker in v1 |
| Validation button | `ValidateConfig` | Shows errors inline |
| Start / Stop buttons | `StartSimulation` / `StopSimulation` | Toggle state based on `GetStatus` |
| Progress bar + status label | `GetStatus` (polled) | Updates every 1s via `QTimer` |

### Startup Flow

```
Qt main():
  └─> parse argv (--remote addr?)
  └─> if local mode:
  │     └─> start embedded gRPC server (background thread)
  └─> create gRPC channel → target address
  └─> create RadarService::Stub
  └─> show main window
  └─> QTimer (1s) → poll GetStatus → update progress
```

### CMake Changes

```
find_package(gRPC REQUIRED)
find_package(Protobuf REQUIRED)
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

protobuf_generate_cpp(proto_sources proto_headers radar_service.proto)
grpc_generate_cpp(grpc_sources grpc_headers radar_service.proto)

add_executable(radar_app
    app/main.cpp
    src/grpc/radar_service.cpp
    ${proto_sources} ${grpc_sources}
    # + Qt UI sources...
)
target_link_libraries(radar_app
    PRIVATE
    radar::modules
    gRPC::grpc++
    protobuf::libprotobuf
    Qt${QT_VERSION_MAJOR}::Widgets
)
```

---

## 6. Implementation Phases

### Phase 1: Proto + Service (core infrastructure)
- Define `radar_service.proto`
- Code-generate C++ stubs
- Implement `RadarServiceImpl` wrapping `SimulationEngine` + `ConfigurationManager`
- Build `radar_server` standalone binary
- Test with `grpcurl` or a minimal CLI client

### Phase 2: Qt UI (basic)
- Qt main window with config panel (JSON text editor), Start/Stop buttons, progress bar
- gRPC client integration
- Local embedded server mode
- Startup argument parsing

### Phase 3: Polish
- Config file open/save dialogs
- Error handling and connection status display
- Remote mode testing
- Build script updates

---

## 7. Dependencies Added

| Dependency | Purpose |
|------------|---------|
| gRPC | RPC framework (client + server) |
| Protobuf | Serialization (comes with gRPC) |
| Qt6 / Qt5 Widgets | Desktop UI |

---

## 8. Future Considerations (Explicitly Out of Scope for v1)

- Raw echo / B-scan display
- Target streaming to UI
- Real-time charting / plotting
- Web frontend via gRPC-web
- Authentication / TLS