# 引擎接口统一与配置重构总结

**日期**: 2026-04-01  
**作者**: lhlj  
**状态**: 已完成

---

## 一、重构背景

本次重构源于两个主要需求：

1. **配置结构混乱**：原有配置结构混合了行为选项和物理参数，存在大量冗余的便捷字段
2. **Engine 接口不统一**：各 Engine 类的接口风格不一致，缺乏统一的生命周期和管理模式
3. **主循环结构不清晰**：仿真主循环使用单层 CPI 循环，扫描完成判断逻辑简化，无法支持每扫描圈数的独立处理

---

## 二、重构目标

### 2.1 配置结构重构
- 将所有配置结构分离为 `Options`（行为选项）+ `Params`（物理参数）
- 删除冗余的便捷字段，消除数据重复存储
- 简化配置维护成本

### 2.2 Engine 接口统一
- 标准化所有 Engine 类的接口结构
- 采用一致的生命周期模式
- 明确配置注入、初始化、核心功能、状态查询的职责划分

### 2.3 主循环结构优化（2026-04-02）
- 将单层 CPI 循环重构为两层循环（scan → cpi）
- 每圈扫描独立收集和导出数据
- 清晰标识每圈扫描的开始和结束

---

## 二、重构目标

### 2.1 配置结构重构
- 将所有配置结构分离为 `Options`（行为选项）+ `Params`（物理参数）
- 删除冗余的便捷字段，消除数据重复存储
- 简化配置维护成本

### 2.2 Engine 接口统一
- 标准化所有 Engine 类的接口结构
- 采用一致的生命周期模式
- 明确配置注入、初始化、核心功能、状态查询的职责划分

---

## 三、配置结构重构详情

### 3.1 重构模式

**重构前**（混合模式 + 便捷字段）：
```cpp
struct NoiseConfig {
    bool enabled = true;
    NoiseLevelMode mode = NoiseLevelMode::ComplexSigma;
    uint64_t seed = 12345;
    
    // 物理参数
    Scalar sigma_complex = 1.0e-3;
    Scalar noise_power_w = 1.0e-6;
    Scalar system_temperature_k = 290.0;
    Scalar noise_bandwidth_hz = 20.0e6;
    
    // ❌ 便捷字段（冗余）
    bool enabled_copy = true;  // 重复存储
    NoiseLevelMode mode_copy = NoiseLevelMode::ComplexSigma;  // 重复存储
};
```

**重构后**（Options + Params 分离）：
```cpp
struct NoiseOptions {
    bool enabled = true;
    NoiseLevelMode mode = NoiseLevelMode::ComplexSigma;
    uint64_t seed = 12345;
};

struct NoiseParams {
    Scalar sigma_complex = 1.0e-3;
    Scalar noise_power_w = 1.0e-6;
    Scalar system_temperature_k = 290.0;
    Scalar noise_bandwidth_hz = 20.0e6;
};

struct NoiseConfig {
    NoiseOptions options;    // 行为选项
    NoiseParams params;      // 物理参数
    // ✅ 无便捷字段，数据只存储一份
};
```

### 3.2 重构的配置结构清单

| 原结构 | 新结构 | 说明 |
|--------|--------|------|
| `WaveformConfig` | `WaveformConfig` + `` | 波形配置 |
| `AntennaConfig` | `AntennaOptions` + `AntennaPhysicalParams` | 天线配置 |
| `NoiseConfig` | `NoiseOptions` + `NoiseParams` | 噪声配置 |
| `SeaClutterConfig` | `ClutterOptions` + `ClutterPhysicalParams` | 海杂波配置 |
| `TargetConfig` | `TargetOptions` + `TargetParams` | 目标配置 |

### 3.3 删除的便捷字段

**WaveformConfig 删除的字段**：
```cpp
// 删除以下冗余字段
WaveformType waveform_type;
PhaseCodeType phase_code_type;
WindowType nlfm_window_type;
PolarizationType polarization;
void sync_from_options();  // 同步函数也删除
```

**AntennaConfig 删除的字段**：
```cpp
// 删除以下冗余字段
PhasedArrayModelType model_type;
int num_elements_az;
int num_elements_el;
Scalar spacing_az_lambda;
Scalar spacing_el_lambda;
AntennaWeightType weight_type_az;
AntennaWeightType weight_type_el;
Scalar peak_gain_db;
std::string beam_table_path;
void sync_from_parts();  // 同步函数也删除
```

### 3.4 配置访问方式变更

| 旧访问方式 | 新访问方式 |
|-----------|-----------|
| `config.waveform_type` | `config.options.waveform_type` |
| `config.model_type` | `config.options.model_type` |
| `config.num_elements_az` | `config.params.num_elements_az` |
| `config.enabled` (noise) | `config.options.enabled` |
| `config.sigma_complex` | `config.params.sigma_complex` |

### 3.5 JSON 配置格式变更

**旧格式**（扁平化）：
```json
{
  "waveform": {
    "waveform_type": 0,
    "phase_code_type": 0
  },
  "antenna": {
    "model_type": 0,
    "num_elements_az": 16
  },
  "noise": {
    "enabled": true,
    "mode": 0,
    "sigma_complex": 1.0
  }
}
```

**新格式**（嵌套结构）：
```json
{
  "waveform": {
    "options": {
      "waveform_type": 0,
      "phase_code_type": 0
    }
  },
  "antenna": {
    "options": {
      "model_type": 0
    },
    "params": {
      "num_elements_az": 16,
      "peak_gain_db": 30.0
    }
  },
  "noise": {
    "options": {
      "enabled": true,
      "mode": 0,
      "seed": 42
    },
    "params": {
      "sigma_complex": 1.0
    }
  }
}
```

**向后兼容**：JSON 解析器同时支持扁平化和嵌套两种格式。

---

## 四、Engine 接口统一详情

### 4.1 标准 Engine 接口模式

```cpp
class XxxEngine {
public:
    // ========================================================================
    // 生命周期
    // ========================================================================
    XxxEngine() = default;
    ~XxxEngine() = default;
    
    // 禁用拷贝操作
    XxxEngine(const XxxEngine&) = delete;
    XxxEngine& operator=(const XxxEngine&) = delete;

    // ========================================================================
    // 配置注入
    // ========================================================================
    void set_config(const XxxConfig& config);
    void set_system_params(const RadarSystemParams& params);

    // ========================================================================
    // 初始化
    // ========================================================================
    bool initialize();
    bool is_initialized() const;
    const std::string& last_error() const;

    // ========================================================================
    // 核心功能
    // ========================================================================
    // 各 Engine 特定的核心方法

    // ========================================================================
    // 状态查询
    // ========================================================================
    // 各 Engine 特定的状态查询方法
};
```

### 4.2 已统一的 Engine 列表

| Engine 类 | 头文件 | 状态 |
|-----------|--------|------|
| `WaveformGenerator` | `waveform/waveform_generator.h` | ✅ 已统一 |
| `NoiseEngine` | `noise/noise_engine.h` | ✅ 已统一 |
| `ClutterEngine` | `clutter/clutter_engine.h` | ✅ 已统一 |
| `TargetEngine` | `target/target_engine.h` | ✅ 已统一 |
| `PhasedArrayAntenna` | `antenna/phased_array_antenna.h` | ✅ 已统一 |
| `AntennaScanModel` | `antenna/antenna_scan_model.h` | ✅ 已统一 |
| `SimulationEngine` | `core/simulation_engine.h` | ✅ 已统一 |

### 4.3 SimulationEngine 重构详情

**重构内容**：
1. 添加 `initialized_` 状态标志
2. 添加 `is_initialized()` 方法
3. 添加 `current_cpi_index()` 方法
4. 禁用拷贝操作
5. 明确划分接口区域（生命周期、配置注入、初始化、核心功能、状态查询）

**新的接口结构**：
```cpp
class SimulationEngine {
public:
    // 生命周期
    explicit SimulationEngine(const RadarConfig& config);
    ~SimulationEngine();
    SimulationEngine(const SimulationEngine&) = delete;
    SimulationEngine& operator=(const SimulationEngine&) = delete;

    // 配置注入
    const RadarConfig& config() const;

    // 初始化
    bool initialize();
    bool is_initialized() const;
    const std::string& last_error() const;

    // 核心功能
    void run(int duration_sec = 0);
    void run_async(int duration_sec = 0);
    void stop();

    // 状态查询
    bool is_running() const;
    bool stop_requested() const;
    int current_scan_index() const;
    int current_cpi_index() const;  // 新增
    int progress_percent() const;
    // ...
};
```

---

## 五、修改的文件清单

### 5.1 头文件修改

| 文件 | 修改类型 | 说明 |
|------|---------|------|
| `include/waveform/waveform_config.hpp` | 重构 | 分离 Options + Params |
| `include/antenna/antenna_config.hpp` | 重构 | 分离 Options + Params |
| `include/noise/noise_config.hpp` | 重构 | 分离 Options + Params |
| `include/clutter/sea_clutter_config.hpp` | 重构 | 分离 Options + Params |
| `include/target/target_config.hpp` | 重构 | 分离 Options + Params |
| `include/core/simulation_engine.h` | 重构 | 统一接口结构 + 两层循环 |
| `include/target/target_engine.h` | 修改 | 更新字段访问 |
| `include/core/radar_config.hpp` | 修改 | 更新字段访问 |

### 5.2 实现文件修改

| 文件 | 修改类型 | 说明 |
|------|---------|------|
| `src/core/configuration_manager.cpp` | 重构 | JSON 解析支持新格式 |
| `src/core/simulation_engine.cpp` | 重构 | 统一接口实现 + 两层循环 |
| `src/waveform/waveform_generator.cpp` | 修改 | 更新字段访问 |
| `src/antenna/phased_array_antenna.cpp` | 修改 | 更新字段访问 |
| `src/noise/noise_engine.cpp` | 修改 | 更新字段访问 |
| `src/clutter/clutter_engine.cpp` | 修改 | 更新字段访问 |
| `src/clutter/sea_clutter_model.cpp` | 修改 | 更新字段访问 |
| `src/target/target_engine.cpp` | 修改 | 更新字段访问 |
| `src/target/target_kinematics.cpp` | 修改 | 更新字段访问 |
| `src/target/target_echo_synthesizer.cpp` | 修改 | 更新字段访问 |
| `src/core/data_exporter.cpp` | 修改 | 更新字段访问 |

---

## 四、主循环两层循环重构（2026-04-02）

### 4.1 问题描述

**重构前**：使用单层 CPI 循环，扫描完成判断逻辑简化
```cpp
// ❌ 旧代码：单层循环
for (int cpi = 0; cpi < total_cpi_count_ && !stop_requested_; ++cpi) {
    // 处理 CPI...
    
    // 扫描完成判断简化
    bool is_last_cpi = (cpi == total_cpi_count_ - 1);
    bool scan_complete = is_last_cpi;  // 只有最后一个 CPI 才触发
    
    if (scan_complete) {
        process_scan_complete(current_scan_index_, ...);
        current_scan_index_++;
    }
}
```

**问题**：
- 无法清晰区分每圈扫描
- 每圈扫描的数据导出逻辑不清晰
- 日志输出无法显示每圈扫描的开始/结束

### 4.2 重构方案

**新代码**：两层循环（scan → cpi）
```cpp
// ✅ 新代码：两层循环
// 外层循环：扫描圈数
for (int scan = 0; scan < total_scan_count_ && !stop_requested_; ++scan) {
    current_scan_index_ = scan;
    
    std::vector<CpiEcho> scan_cpi_echos;
    std::vector<TargetList> scan_target_snapshots;
    
    SPDLOG_INFO("--- Starting Scan {} ---", scan);
    
    // 内层循环：每个扫描的 CPI
    for (int beam = 0; beam < beam_count_ && !stop_requested_; ++beam) {
        const int cpi_index = scan * beam_count_ + beam;
        process_single_cpi(cpi_index, current_time, ...);
        
        // 推进时间和波位
        current_time += pulses_per_cpi * pri;
        scan_model_->advance_one_cpi();
        cpi_counter_++;
    }
    
    // 扫描完成，处理数据
    if (!stop_requested_) {
        process_scan_complete(scan, scan_cpi_echos, scan_target_snapshots);
    }
}
```

### 4.3 新增方法

| 方法 | 说明 |
|------|------|
| `run_scan_loop()` | 新的两层循环主逻辑 |
| `process_single_cpi()` | 处理单个 CPI 的回波生成和数据收集 |
| `run_cpi_loop()` | 兼容旧接口，调用 `run_scan_loop()` |

### 4.4 日志输出对比

**重构前**：
```
[info] === Starting Simulation ===
[info] Progress: 14%
[info] Progress: 21%
...
[info] [DataExporter] Scan 0 exported.
[info] [Scan 0 complete] CPIs processed: 14
[info] === Simulation Complete ===
```

**重构后**：
```
[info] === Starting Simulation ===
[info]   Scan mode: 2 scans × 7 beams/scan = 14 CPIs
[info] --- Starting Scan 0 ---
[info] Progress: 14%
...
[info] [DataExporter] Scan 0 exported.
[info] [Scan 0 complete] CPIs processed: 7
[info] --- Starting Scan 1 ---
[info] Progress: 64%
...
[info] [DataExporter] Scan 1 exported.
[info] [Scan 1 complete] CPIs processed: 7
[info] === Simulation Complete ===
```

### 4.5 优势

1. **清晰的扫描边界**：每圈扫描有明确的开始和结束日志
2. **独立的数据导出**：每圈扫描完成后立即导出数据
3. **易于扩展**：未来可在每圈扫描之间添加额外逻辑（如雷达位置调整、目标更新等）
4. **正确的 CPI 计数**：每圈扫描独立计数，便于调试和监控

---

## 六、重构收益

### 6.1 消除冗余
- 删除约 **40+** 个便捷字段
- 删除 **5+** 个同步函数 (`sync_from_options()`, `sync_from_parts()`)
- 数据只存储一份，修改配置只需改一处

### 6.2 语义清晰
- `options` 明确是行为选项
- `params` 明确是物理参数
- 不再困惑该用哪个字段

### 6.3 接口统一
- 所有 Engine 遵循相同的接口模式
- 新用户更容易理解和使用
- 便于后续扩展和维护

### 6.4 向后兼容
- JSON 配置同时支持扁平化和嵌套格式
- 旧配置文件仍可正常加载

---

## 七、验证结果

### 7.1 编译测试
```bash
cd build && cmake .. && make -j4
# 结果：编译成功，仅有少量未使用参数警告
```

### 7.2 功能测试
```bash
./out/bin/radar
# 结果：仿真正常运行，完成 2 圈扫描（14 个 CPI）
# 输出：
# [info] Simulation initialized: 7 beams/scan, 14 CPIs total
# [info] === Simulation Complete ===
# [info] Simulation finished successfully.
```

---

## 八、注意事项

### 8.1 代码迁移时需要注意
1. 访问配置字段时，需要根据字段类型判断是在 `options` 还是 `params` 中
2. 行为选项（enabled, mode, seed, type 等）→ `options`
3. 物理参数（数量、间距、增益、功率等）→ `params`

### 8.2 新增 Engine 时的规范
1. 配置结构遵循 `XxxOptions` + `XxxParams` + `XxxConfig` 模式
2. Engine 接口遵循标准五段式结构
3. 禁用拷贝操作，使用 `std::unique_ptr` 管理

---

## 九、后续工作建议

1. **可选**：考虑为配置结构添加 `to_json()` 序列化函数
2. **可选**：为配置验证添加单元测试
3. **可选**：考虑使用 constexpr 验证部分配置参数

---

## 十、UDP 模块独立（2026-04-02）

### 10.1 重构背景

原有 `UdpConfig` 同时出现在两个地方：
1. `RadarSystemParams::udp_output` - 作为系统参数的成员
2. `RadarConfig::udp_output` - 作为顶层配置

这导致：
1. 语义不清晰：UDP 输出是仿真结果的输出方式，不是雷达物理参数
2. 数据冗余：两个字段存储相同的数据，需要通过 `sync_udp_config()` 同步
3. 耦合度高：`RadarSystemParams` 不应该依赖 UDP 输出这种应用层概念

### 10.2 重构方案

**第 1 步：创建独立的 UDP 模块**
```
include/core/udp/
├── udp_config.h      # UdpConfig 配置结构
└── udp_sender.h      # UdpSender 类定义

src/core/udp/
└── udp_sender.cpp    # UdpSender 实现
```

**第 2 步：从 RadarSystemParams 中移除 udp_output**
```cpp
// 修改前
struct RadarSystemParams {
    // ... 物理参数
    UdpConfig udp_output;  // ❌ 移除
};

// 修改后
struct RadarConfig {
    RadarSystemParams system;
    // ...
    UdpConfig udp_output;  // ✅ 只在这里保留
};
```

**第 3 步：更新相关代码**
- `simulation_engine.cpp`: `config_.system.udp_output` → `config_.udp_output`
- `configuration_manager.cpp`: 移除 `system.udp_output` 的 JSON 解析和序列化
- `RadarConfig::sync_udp_config()`: 标记为 deprecated（空实现）

### 10.3 依赖关系

**新的依赖关系**：
- `udp_config.h`：无内部依赖，只包含标准库
- `udp_sender.h`：包含 `udp_config.h`、`types.h`、`radar_system_params.hpp`
- `radar_system_params.hpp`：包含 `udp_config.h`（仅使用类型，不作为成员）
- `radar_config.hpp`：包含 `udp_config.h` 和 `udp_sender.h`

### 10.4 优势

1. **解耦**：`RadarSystemParams` 不再包含应用层概念（UDP 输出）
2. **语义清晰**：UDP 配置独立成模块，职责明确
3. **消除冗余**：只有一个 `udp_output` 字段，不需要同步
4. **可扩展**：未来添加更多 UDP 相关功能时，都在 `udp/` 目录下

---

## 十一、JSON 配置扩展：波位表和初始目标（2026-04-02）

### 11.1 新增配置项

在 `config/radar_config.json` 中新增两个顶级配置项：

**1. beam_table - 波位表配置**
```json
"beam_table": {
  "type": "azimuth_scan",
  "az_start_deg": -30,
  "az_end_deg": 30,
  "az_step_deg": 10,
  "elevation_deg": 0.0,
  "custom_beams": []
}
```

**2. initial_targets - 初始目标列表**
```json
"initial_targets": [
  {
    "id": 1,
    "position_m": [50000.0, 0.0, 5000.0],
    "velocity_mps": [0.0, 0.0, 0.0],
    "acceleration_mps2": [0.0, 0.0, 0.0],
    "motion_model": 0,
    "rcs_mean_m2": 10.0,
    "swerling": 1,
    "enabled": true
  }
]
```

### 11.2 代码实现

**1. 新增 BeamTableConfig 结构体** (`include/core/radar_config.hpp`)
```cpp
struct BeamTableConfig {
    std::string type = "azimuth_scan";   // 波位表类型："azimuth_scan" 或 "custom"
    Scalar az_start_deg = -30.0;         // 方位扫描起始角度（度）
    Scalar az_end_deg = 30.0;            // 方位扫描结束角度（度）
    Scalar az_step_deg = 10.0;           // 方位扫描步长（度）
    Scalar elevation_deg = 0.0;          // 俯仰角（度）
    std::vector<AzEl> custom_beams;      // 自定义波位列表
};
```

**2. 添加到 RadarConfig**
```cpp
struct RadarConfig {
    // ... 其他成员
    BeamTableConfig beam_table;        // 波位表配置
    TargetList initial_targets;        // 初始目标列表
};
```

**3. JSON 解析支持** (`src/core/configuration_manager.cpp`)
```cpp
// BeamTableConfig 解析
void from_json(const nlohmann::json& j, BeamTableConfig& cfg);

// TargetState 解析
void from_json(const nlohmann::json& j, TargetState& target);

// RadarConfig 解析中新增
if (j.contains("beam_table")) {
    j.at("beam_table").get_to(cfg.beam_table);
}
if (j.contains("initial_targets")) {
    const auto& targets = j.at("initial_targets");
    for (const auto& t : targets) {
        TargetState target;
        from_json(t, target);
        cfg.initial_targets.push_back(target);
    }
}
```

**4. SimulationEngine 使用配置** (`src/core/simulation_engine.cpp`)
```cpp
// 从配置加载波位表
auto beam_table = create_beam_table_from_config(config_.beam_table);
scan_model_->set_beam_table(beam_table);

// 从配置加载初始目标
initial_targets_ = create_targets_from_config(config_.initial_targets);
target_manager_->set_targets(initial_targets_);
```

### 11.3 支持的模式

**波位表支持两种模式**：
1. **azimuth_scan**：方位扫描模式，根据 `az_start_deg`、`az_end_deg`、`az_step_deg` 自动生成波位
2. **custom**：自定义波位模式，使用 `custom_beams` 数组中定义的波位

**目标配置支持所有 TargetState 字段**：
- `id`：目标唯一标识
- `position_m`：位置数组 [x, y, z]（米）
- `velocity_mps`：速度数组 [vx, vy, vz]（米/秒）
- `acceleration_mps2`：加速度数组 [ax, ay, az]（米/秒²）
- `motion_model`：运动模型（0=Stationary, 1=ConstantVelocity, 2=ConstantAcceleration）
- `rcs_mean_m2`：平均 RCS（平方米）
- `swerling`：Swerling 起伏模型（0-4）
- `enabled`：是否启用

### 11.4 向后兼容

如果 `initial_targets` 为空，SimulationEngine 会使用默认的两个目标（向后兼容）：
- 目标 1：静止目标（50km, 0, 5km），RCS=10 m²
- 目标 2：匀速目标（30km, 10km, 3km），速度 100 m/s，RCS=5 m²

---

## 十二、相关文档

- [配置架构重构设计](../superpowers/specs/2026-03-31-configuration-architecture-refactoring-design.md)
- [配置重构计划](../superpowers/plans/2026-03-31-parameter-config-refactoring.md)