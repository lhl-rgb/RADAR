# 目标模块重构总结

**日期**: 2026-04-02
**状态**: ✅ 已完成

---

## 架构对比

### 重构前

```
┌─────────────────────────────────────────────────────────────┐
│ SimulationEngine                                             │
│   ├─ target_engine_    (只负责回波生成)                      │
│   └─ target_manager_   (独立存在，与 TargetEngine 无隶属关系) │
│                                                              │
│ 问题：                                                       │
│ - TargetManager 与 TargetEngine 无隶属关系                   │
│ - 状态更新逻辑散落在 run_scan_loop()                         │
│ - 配置拆分为 Options + Params 两层                           │
└─────────────────────────────────────────────────────────────┘
```

### 重构后

```
┌─────────────────────────────────────────────────────────────┐
│ SimulationEngine                                             │
│   └─ target_engine_ (统一管理器)                             │
│        ├─ TargetManager (状态管理，被拥有)                   │
│        ├─ TargetKinematics (运动学计算)                      │
│        └─ TargetEchoSynthesizer (回波合成)                   │
│                                                              │
│ 优点：                                                       │
│ - TargetEngine 拥有 TargetManager                            │
│ - 状态更新通过 TargetEngine.update_targets()                 │
│ - 配置扁平化（精简为 TargetConfig）                          │
└─────────────────────────────────────────────────────────────┘
```

---

## 核心改动

### 1. TargetConfig 扁平化

**修改文件**: `include/target/target_config.hpp`

#### 旧格式
```cpp
struct TargetConfig {
    TargetOptions options;  // enabled, enable_swerling, seed...
    TargetParams params;    // enable_beam_gain, beam_gate_threshold_db...
};
```

#### 新格式
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

**JSON 序列化**: 同时支持扁平化和嵌套两种格式（向后兼容）

---

### 2. TargetEngine 新增状态管理接口

**修改文件**: `include/target/target_engine.h`, `src/target/target_engine.cpp`

#### 新增方法

| 方法 | 说明 | 委托给 |
|-----|------|--------|
| `set_initial_targets()` | 设置初始目标状态 | `target_manager_.set_targets()` |
| `update_targets(time)` | 更新目标状态到指定时刻 | `target_manager_.update_to_time()` |
| `get_current_targets()` | 获取当前目标状态 | `target_manager_.get_current_targets()` |
| `clear_targets()` | 清空目标列表 | `target_manager_.clear()` |
| `target_count()` | 获取目标数量 | `target_manager_.size()` |

#### 新增成员
```cpp
TargetManager target_manager_;  ///< 目标状态管理器（被拥有）
```

---

### 3. generate() 方法签名简化

**修改文件**: `include/target/target_engine.h`

#### 旧签名
```cpp
bool generate(const TargetList& active_targets,  // ← 移除
              const BeamView& beam,
              const RadarSystemParams& system,
              const ComplexVec& tx_waveform,
              CpiEcho& out_echo);
```

#### 新签名
```cpp
bool generate(const BeamView& beam,
              const RadarSystemParams& system,
              const ComplexVec& tx_waveform,
              CpiEcho& out_echo);
```

**说明**: `active_targets` 参数移除，由 `TargetEngine` 内部通过 `get_current_targets()` 获取。

---

### 4. SimulationEngine 调用方式变更

**修改文件**: `src/core/simulation_engine.cpp`

#### 旧代码
```cpp
// 初始化
target_manager_->set_targets(initial_targets_);

// 运行时
target_manager_->update_to_time(current_time);
TargetList active_targets = target_manager_->get_current_targets();
target_engine_->generate(active_targets, beam_view, ...);
```

#### 新代码
```cpp
// 初始化
target_engine_->set_initial_targets(initial_targets_);

// 运行时
target_engine_->update_targets(current_time);
const TargetList& active_targets = target_engine_->get_current_targets();
target_engine_->generate(beam_view, ...);
```

---

### 5. 配置访问方式统一

**修改文件**:
- `src/core/data_exporter.cpp`
- `src/target/target_kinematics.cpp`
- `src/target/target_echo_synthesizer.cpp`

#### 变更前
```cpp
config.options.enabled
config.params.enable_beam_gain
config.params.enable_phase
config.options.seed
```

#### 变更后
```cpp
config.enabled
config.enable_beam_gain
config.enable_phase
config.seed
```

---

## 文件变更清单

| 文件 | 变更类型 | 说明 |
|-----|---------|------|
| `include/target/target_config.hpp` | 修改 | 精简配置结构，添加 JSON 序列化 |
| `include/target/target_engine.h` | 修改 | 添加 TargetManager 成员，新增状态管理接口 |
| `src/target/target_engine.cpp` | 修改 | 实现状态管理方法，修改 generate 签名 |
| `include/core/simulation_engine.h` | 修改 | 移除 target_manager_ 成员 |
| `src/core/simulation_engine.cpp` | 修改 | 更新为目标引擎统一管理 |
| `src/core/configuration_manager.cpp` | 修改 | 更新 JSON 解析/序列化 |
| `src/core/data_exporter.cpp` | 修改 | 更新配置访问方式 |
| `src/target/target_kinematics.cpp` | 修改 | 更新配置访问方式 |
| `src/target/target_echo_synthesizer.cpp` | 修改 | 更新配置访问方式 |

---

## 设计原则

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

## 编译验证

```bash
$ cmake --build build
[ 22%] Built target radar_core
[ 36%] Built target radar_antenna
[ 45%] Built target radar_waveform
[ 59%] Built target radar_clutter
[ 81%] Built target radar_target
[ 90%] Built target radar_noise
[100%] Built target radar_app
```

---

## 运行验证

```bash
$ ./out/bin/radar --duration 1
[info] === Starting Simulation ===
[info]   Scan mode: 2 scans × 7 beams/scan = 14 CPIs
[info] --- Starting Scan 0 ---
[info] Progress: 14%
[info] Progress: 21%
# 程序正常运行，无崩溃
```
