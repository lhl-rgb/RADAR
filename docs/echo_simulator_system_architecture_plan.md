# 对海监视雷达回波模拟器系统架构优化规划

## 1. 设计目标

当前实现状态：

- 已完成 v1 数据面骨架：`EchoFrame`、`EchoSink`、`NullEchoSink`、`FileEchoSink`。
- 已完成 v1 UDP 数据面输出：`UdpEchoSink` 可按 PRT 分包发送完整 IQ 数据。
- 已完成 v1 监控面骨架：`PreviewService`、距离向功率 preview、PPI preview、`MetricsCollector`。
- 已接入 `SimulationEngine`：每个 PRT 构造 `EchoFrame`，写入 `EchoSink`，更新 preview 和 metrics。
- 已新增 gRPC `GetMetrics` 查询接口、`StreamMetrics`、`StreamRangeProfile`、`StreamPpiPreview` 流式监控接口。
- 尚未完成：Qt preview 控件、PCIe 输出、输出队列和独立输出线程。

当前工程已经具备波形、目标、噪声、海杂波和机械扫描等基础模块。后续系统优化的重点不再只是单个算法模块，而是把工程组织成一个可运行、可监控、可扩展的数据模拟系统。

核心目标：

- gRPC/Qt 作为低速控制面和监控面。
- 完整原始 IQ 回波不通过 Qt/gRPC 回传。
- 完整 IQ 通过文件、UDP、后续 PCIe 等数据面输出。
- Qt 只显示低速 preview，例如距离向功率曲线、PPI 预览图和实时性能指标。
- 主仿真流程按 PRT 输出，减少整圈缓存压力。

整体原则：

```text
控制面：参数配置、启动停止、状态查询
数据面：完整 IQ 回波输出
监控面：低速 preview 和 metrics
```

## 2. 总体架构

建议系统结构：

```text
Qt Client
  |
  | gRPC 控制 + preview + metrics
  v
Radar Server
  |
  |-- SimulationController
  |     |-- RadarConfig / ScenarioConfig
  |     |-- SimulationEngine
  |
  |-- SimulationEngine
  |     |-- BeamScheduler / MechanicalScanner
  |     |-- WaveformGenerator
  |     |-- TargetEngine
  |     |-- SeaClutterEngine
  |     |-- NoiseEngine
  |     |-- EchoComposer
  |
  |-- EchoSink 数据输出
  |     |-- NullEchoSink
  |     |-- FileEchoSink
  |     |-- UdpEchoSink
  |     |-- PcieEchoSink（预留）
  |
  |-- PreviewService 监控显示
  |     |-- RangeProfilePreview
  |     |-- PpiPreviewBuilder
  |     |-- MetricsCollector
```

每个 PRT 的数据流：

```text
当前波束指向
  -> 目标回波
  -> 海杂波回波
  -> 热噪声
  -> 合成完整 EchoFrame
  -> EchoSink 输出完整 IQ
  -> PreviewService 抽取低速功率预览
  -> MetricsCollector 更新状态
```

## 3. 关键接口设计

### 3.1 EchoFrame

统一单 PRT 输出结构：

```cpp
struct EchoFrame {
    uint64_t pulse_index = 0;
    uint32_t scan_index = 0;
    float timestamp_s = 0.0f;
    float beam_az_deg = 0.0f;
    float beam_el_deg = 0.0f;
    float range_bin_size_m = 0.0f;
    std::vector<Complex> iq;
};
```

用途：

- 表示一个 PRT 的完整复基带回波。
- 作为数据面输出的统一输入。
- 作为 preview 和 metrics 的数据来源。

### 3.2 EchoSink

统一输出接口：

```cpp
class EchoSink {
public:
    virtual ~EchoSink() = default;
    virtual bool open(const OutputConfig& cfg) = 0;
    virtual bool write_frame(const EchoFrame& frame) = 0;
    virtual void close() = 0;
};
```

v1 实现：

- `NullEchoSink`：不输出，用于纯生成速度测试。
- `FileEchoSink`：写 `echo_iq.dat` 和 `metadata.json`。
- `UdpEchoSink`：按 PRT 发送 UDP 数据包，单帧过大时按距离采样分段。

后续预留：

- `PcieEchoSink`：对接 PCIe 或硬件后端。

### 3.3 PreviewService

低速监控数据结构：

```cpp
struct RangeProfilePreview {
    uint64_t pulse_index = 0;
    float beam_az_deg = 0.0f;
    std::vector<float> power_db;
};

struct PpiPreviewFrame {
    uint32_t scan_index = 0;
    int range_bins = 0;
    int az_bins = 0;
    float dynamic_range_db = 50.0f;
    std::vector<uint8_t> image;
};
```

配置：

```cpp
struct PreviewConfig {
    bool enabled = true;
    int range_profile_stride_pulses = 50;
    int ppi_range_bins = 512;
    int ppi_az_bins = 720;
    float dynamic_range_db = 50.0f;
};
```

行为：

- 每隔若干 PRT 生成一次距离向功率曲线。
- 每圈生成或更新一张低分辨率 PPI preview。
- preview 允许丢帧，不阻塞主仿真。
- 不保存或传输完整复数 IQ。

### 3.4 MetricsCollector

运行指标：

```text
current_scan
current_pulse
progress_percent
avg_us_per_prt
realtime_ratio
target_us
clutter_us
noise_us
compose_us
output_us
preview_us
output_queue_depth
dropped_preview_frames
last_error
```

用途：

- gRPC `GetMetrics` 查询。
- Qt 状态面板显示。
- 性能测试和论文实验记录。

## 4. gRPC 与 Qt 设计

### 4.1 gRPC 控制面

保留已有参数设置接口：

```text
SetSystemParams
SetWaveformParams
SetAntennaParams
SetMechScanParams
SetNoiseParams
SetClutterParams
AddTarget / RemoveTarget / ClearTargets
ValidateAll
StartSimulation
StopSimulation
GetStatus
```

新增监控接口：

```text
GetMetrics
StreamMetrics
StreamRangeProfile
StreamPpiPreview
```

默认不提供完整 IQ 的 gRPC stream。若后续需要调试，可单独做小规模 DebugRawIq 接口，不作为实时主链路。

### 4.2 Qt 界面

Qt 作为控制台：

- 参数配置区。
- 启动、停止、验证控制区。
- 实时状态区。
- 距离向功率曲线。
- PPI preview。
- 日志区。

Qt 显示的数据来自 gRPC preview 和 metrics，不直接接收原始 IQ 数据。

## 5. SimulationEngine 改造方案

当前 `SimulationEngine` 逐 PRT 生成回波并缓存到 `ScanEcho`，扫描结束后再通过 `DataExporter` 导出。后续改造为：

```text
process_single_prt
  -> 生成目标回波
  -> 生成杂波回波
  -> 加噪声
  -> 构造 EchoFrame
  -> EchoSink::write_frame
  -> PreviewService::consume_frame
  -> MetricsCollector::update
```

兼容策略：

- `DataExporter` 保留，用于离线 MATLAB 验证。
- 新主路径优先使用 `EchoSink`。
- 若启用旧导出模式，可继续缓存 `ScanEcho`。
- 若使用 `NullEchoSink`，不输出文件，只统计性能。

输出失败处理：

- `EchoSink` 输出失败：主仿真停止，记录 `last_error`。
- preview 生成失败或队列满：丢弃 preview，记录丢帧数，不停止仿真。

## 6. 建议代码结构

新增模块建议：

```text
radar_common/include/core/
  echo_frame.h
  metrics_collector.h

radar_common/include/output/
  output_config.h
  echo_sink.h
  null_echo_sink.h
  file_echo_sink.h
  udp_echo_sink.h

radar_common/include/preview/
  preview_config.h
  preview_types.h
  preview_service.h

radar_common/src/output/
  null_echo_sink.cpp
  file_echo_sink.cpp
  udp_echo_sink.cpp

radar_common/src/preview/
  preview_service.cpp

radar_common/src/core/
  metrics_collector.cpp
```

`RadarConfig` 增加：

```cpp
OutputConfig output;
PreviewConfig preview;
```

`SimulationEngine` 增加：

```cpp
std::unique_ptr<output::EchoSink> echo_sink_;
std::unique_ptr<preview::PreviewService> preview_service_;
std::unique_ptr<core::MetricsCollector> metrics_;
```

## 7. 实施顺序

### 阶段 1：数据面骨架

- 定义 `EchoFrame`。
- 定义 `OutputConfig` 和 `EchoSink`。
- 实现 `NullEchoSink`、`FileEchoSink`。
- `SimulationEngine` 每个 PRT 构造 `EchoFrame`。
- 新增 `NullEchoSink` 性能测试。

### 阶段 2：Preview 与 Metrics

- 定义 `PreviewConfig`。
- 实现距离向功率曲线 preview。
- 实现 PPI preview 累计。
- 实现 `MetricsCollector`。
- `SimulationEngine` 每 PRT 更新 preview 和 metrics。

### 阶段 3：gRPC 监控接口

- proto 增加 `SimMetrics`、`RangeProfilePreviewMsg`、`PpiPreviewMsg`。
- 实现 `GetMetrics`。
- 实现 `StreamMetrics`。
- 实现 preview stream。

### 阶段 4：Qt 控制台

- 增加运行状态面板。
- 增加距离向功率曲线。
- 增加 PPI preview。
- 增加日志面板。

### 阶段 5：UDP / PCIe 扩展

- 完成 `UdpEchoSink`。
- 预留 `PcieEchoSink` 接口。
- 增加输出队列和输出线程。

当前完成情况：

- 阶段 1 已完成：数据面统一为 `EchoFrame` + `EchoSink`。
- 阶段 2 已完成：preview 与 metrics 已接入逐 PRT 主循环。
- 阶段 3 后端已完成：gRPC 已提供 metrics、距离向曲线、PPI preview 的查询/流式监控接口。
- 阶段 5 已完成 UDP 基础输出：`UdpEchoSink` 已接入工厂；PCIe、输出队列和独立输出线程仍作为后续扩展。
- 阶段 4 待完成：Qt 客户端需要接入上述 stream，并实现曲线、PPI 和状态面板显示。

## 8. 测试计划

单元测试：

- `EchoFrame` 元数据正确。
- `FileEchoSink` 文件头、metadata 和 IQ 数据长度正确。
- `NullEchoSink` 不输出但帧计数正确。
- `PreviewService` 对固定 IQ 生成确定性功率曲线。
- `MetricsCollector` 进度、平均耗时和实时倍率计算正确。

集成测试：

- 关闭目标、杂波、噪声分别运行一圈。
- 开启目标、杂波、噪声运行一圈并输出文件。
- `NullEchoSink` 下测试生成速度。
- gRPC 启动仿真后 `GetMetrics` 能看到进度变化。
- preview stream 能收到距离向曲线和 PPI preview。

性能测试：

- `NullEchoSink`：纯生成速度。
- `FileEchoSink`：磁盘输出开销。
- `UdpEchoSink`：网络发送吞吐。
- preview 开启/关闭对实时倍率的影响。

## 9. 论文和汇报表述

可以总结为：

```text
为避免 GUI 和 RPC 通道占用高速回波数据带宽，系统采用控制面、数据面和监控面分离的架构。
完整复基带 IQ 回波通过文件、UDP 或后续 PCIe 接口输出；
Qt 客户端通过 gRPC 接收参数状态、性能指标和低速功率预览图。
该设计既保证了实时数据输出能力，又保留了仿真过程的可视化监控能力。
```

可作为论文系统设计章节的重要内容：

- 控制面与数据面分离。
- 按 PRT 的 `EchoFrame` 数据模型。
- 可扩展 `EchoSink` 输出接口。
- 低速 preview 监控机制。
- 实时 metrics 和性能评估机制。
