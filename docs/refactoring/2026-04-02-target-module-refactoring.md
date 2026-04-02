# 目标模块架构重构

**日期**: 2026-04-02
**状态**: ✅ 已完成
**编译**: 通过 (100% Built target radar_app)
**运行**: 正常（无崩溃）

---

## 重构动机

用户指出目标模块的状态管理混乱：
- `TargetManager` 在 `SimulationEngine` 中独立存在，与 `TargetEngine` 没有隶属关系
- 目标状态更新逻辑散落在外层循环
- 配置结构过度拆分（`TargetOptions` + `TargetParams`）

参考天线模块的重构经验，实现统一的架构设计。

---

## 架构变更

### 重构前

```
┌─────────────────────────────────────────────────────────────┐
│ 旧架构（混乱）                                               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  SimulationEngine ────────────────────────┐                 │
│    ├─ target_engine_ (只负责回波生成)      │                 │
│    └─ target_manager_ (独立存在) ──────────┤                 │
│                                             │                 │
│  问题：                                     │                 │
│  - TargetManager 与 TargetEngine 无隶属关系 │                 │
│  - 状态更新逻辑散落在 run_scan_loop()       │                 │
│  - 配置拆分为 Options + Params              │                 │
└─────────────────────────────────────────────────────────────┘
```

### 重构后

```
┌─────────────────────────────────────────────────────────────┐
│ 新架构（清晰）                                               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  SimulationEngine                                           │
│    └─ target_engine_ (统一管理器)                           │
│         ├─ TargetManager (状态管理，被拥有)                 │
│         ├─ TargetKinematics (运动学计算)                    │
│         └─ TargetEchoSynthesizer (回波合成)                 │
│                                                             │
│  改进：                                                      │
│  - TargetEngine 拥有 TargetManager                          │
│  - 状态更新通过 TargetEngine.update_targets()               │
│  - 配置扁平化（精简为 TargetConfig）                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 配置结构变更

### 旧格式
```cpp
struct TargetConfig {
    TargetOptions options;  // enabled, enable_swerling, seed...
    TargetParams params;    // enable_beam_gain, beam_gate_threshold_db...
};
```

### 新格式（扁平化）
```cpp
struct TargetConfig {
    // 行为选项
    bool enabled = true;
    bool enable_swerling = true;
    bool skip_out_of_beam_targets = false;
    uint64_t seed = 20260330ULL;

    // 物理参数
    bool enable_beam_gain = true;
    bool enable_two_way_propagation_loss = true;
    bool enable_phase = true;
    Scalar beam_gate_threshold_db = -20.0;
};
```

---

## 接口变更

### TargetEngine 新增方法

| 方法 | 说明 |
|-----|------|
| `set_initial_targets()` | 设置初始目标状态 |
| `update_targets(time)` | 更新目标状态到指定时刻 |
| `get_current_targets()` | 获取当前目标状态 |
| `clear_targets()` | 清空目标列表 |
| `target_count()` | 获取目标数量 |

### TargetEngine generate() 方法变更

**旧签名：**
```cpp
bool generate(const TargetList& active_targets,
              const BeamView& beam,
              const RadarSystemParams& system,
              const ComplexVec& tx_waveform,
              CpiEcho& out_echo);
```

**新签名：**
```cpp
bool generate(const BeamView& beam,
              const RadarSystemParams& system,
              const ComplexVec& tx_waveform,
              CpiEcho& out_echo);
```

**说明：** `active_targets` 参数移除，由 `TargetEngine` 内部通过`get_current_targets()` 获取。

---

## 调用方式变更

### SimulationEngine 中

**旧代码：**
```cpp
// 初始化
target_manager_->set_targets(initial_targets_);

// 运行时
target_manager_->update_to_time(current_time);
TargetList active_targets = target_manager_->get_current_targets();
target_engine_->generate(active_targets, beam_view, ...);
```

**新代码：**
```cpp
// 初始化
target_engine_->set_initial_targets(initial_targets_);

// 运行时
target_engine_->update_targets(current_time);
const TargetList& active_targets = target_engine_->get_current_targets();
target_engine_->generate(beam_view, ...);
```

---

## 文件变更

### 修改的文件

| 文件 | 变更内容 |
|-----|---------|
| `include/target/target_config.hpp` | 精简配置结构，添加 JSON 序列化 |
| `include/target/target_engine.h` | 添加 TargetManager 成员，新增状态管理接口 |
| `src/target/target_engine.cpp` | 实现状态管理方法，修改 generate 签名 |
| `include/core/simulation_engine.h` | 移除 target_manager_ 成员 |
| `src/core/simulation_engine.cpp` | 更新为目标引擎统一管理 |
| `src/core/configuration_manager.cpp` | 更新 JSON 解析/序列化 |
| `include/core/radar_config.hpp` | 更新 print() 函数 |
| `src/core/data_exporter.cpp` | 更新配置访问方式 |
| `src/target/target_kinematics.cpp` | 更新配置访问方式 |
| `src/target/target_echo_synthesizer.cpp` | 更新配置访问方式 |

---

## 运行时 Bug 修复

在验证运行时发现程序崩溃，定位并修复了 `BeamScanner` 未初始化 `AntennaModel` 的问题。

### 问题描述

**崩溃位置**: `AntennaModel::normalized_power_upa()` 访问空 `weights_az_` 向量

**堆栈跟踪**:
```
#0  radar::antenna::AntennaModel::normalized_power_upa()
#1  radar::antenna::AntennaModel::normalized_power()
#2  radar::antenna::AntennaModel::gain()
#3  radar::target::TargetKinematics::generate_trajectory()
```

**根本原因**: `BeamScanner::initialize()` 未调用 `antenna_model_.set_config()` 和 `initialize()`，导致 `AntennaModel` 内部权重向量为空。

### 修复方案

**1. 添加天线配置成员** (`include/antenna/beam_scanner.h`)
```cpp
private:
    BeamTable beam_table_;
    BeamTableConfig beam_table_config_;
    AntennaConfig antenna_config_;  // 新增
    AntennaModel antenna_model_;
    // ...
```

**2. 添加配置接口** (`include/antenna/beam_scanner.h`)
```cpp
void set_antenna_config(const AntennaConfig& config) { antenna_config_ = config; }
```

**3. 初始化天线模型** (`src/antenna/beam_scanner.cpp`)
```cpp
bool BeamScanner::initialize() {
    // 1. 初始化天线模型
    antenna_model_.set_config(antenna_config_);
    if (!antenna_model_.initialize()) {
        last_error_ = "AntennaModel initialization failed";
        return false;
    }
    
    // 2. 加载波位表...
}
```

**4. 更新调用方** (`src/core/simulation_engine.cpp`)
```cpp
beam_scanner_->set_antenna_config(config_.antenna);
beam_scanner_->set_beam_table_config(config_.beam_table);
if (!beam_scanner_->initialize()) { ... }
```

### 验证结果

```bash
$ cmake --build build
[100%] Built target radar_app

$ ./out/bin/radar --duration 1
[info] === Starting Simulation ===
[info]   Scan mode: 2 scans × 7 beams/scan = 14 CPIs
[info] --- Starting Scan 0 ---
[info] Progress: 14%
[info] Progress: 21%
# 程序正常运行，无崩溃
```

---

## 设计原则总结

### 1. 统一管理
- `TargetEngine` 是目标的唯一对外接口
- 所有目标操作（状态管理、回波生成）都通过 `TargetEngine`

### 2. 内部拥有
- `TargetEngine` **直接拥有** `TargetManager`
- 外部（`SimulationEngine`）不直接接触 `TargetManager`

### 3. 配置扁平化
- 移除 `TargetOptions` + `TargetParams` 嵌套结构
- 使用单一的 `TargetConfig` 结构

### 4. 接口简化
- `generate()` 方法不再需要外部传入 `active_targets`
- 由 `TargetEngine` 内部管理目标状态

---

## 与天线模块的一致性

两个模块现在遵循相同的设计模式：

| 模块 | 物理模型 | 控制器 | 配置归属 |
|-----|---------|-------|---------|
| 天线 | `AntennaModel` | `BeamScanner` | `BeamTableConfig` 独立 |
| 目标 | `TargetKinematics`/`TargetEchoSynthesizer` | `TargetEngine` | `TargetConfig` 扁平化 |

---

## 编译输出

```
[ 22%] Built target radar_core
[ 36%] Built target radar_antenna
[ 45%] Built target radar_waveform
[ 59%] Built target radar_clutter
[ 81%] Built target radar_target
[ 90%] Built target radar_noise
[100%] Built target radar_app
```

**警告**: 仅有少量未使用参数警告，不影响功能。
