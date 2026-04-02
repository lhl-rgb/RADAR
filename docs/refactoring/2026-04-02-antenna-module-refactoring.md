# 天线模块架构重构

**日期**: 2026-04-02
**状态**: ✅ 已完成
**编译**: 通过 (100% Built target radar_app)

---

## 重构动机

用户指出天线模块的设计可能存在职责不清的问题，希望进行架构优化：

1. `PhasedArrayAntenna` 名字冗长，且配置中混入了不属于它的波位表配置
2. `AntennaScanModel` 名为 "Model" 但实际是扫描控制器
3. `AntennaConfig` 中 `beam_table_cfg` 归属不当（天线物理模型不需要波位表）
4. 目标模块存在类似问题，希望先重构天线作为参考

---

## 架构变更

### 重构前

```
┌─────────────────────────────────────────────────────────────┐
│ 旧架构（混乱）                                               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  AntennaConfig ────┐                                        │
│    - options       │                                        │
│    - params        ↓                                        │
│    - beam_table_cfg  PhasedArrayAntenna                     │
│                      (物理天线模型)                          │
│                      - 计算增益                              │
│                      - 不需要波位表但配置里有                │
│                            ↑                                │
│                      (被引用)                                │
│                            ↑                                │
│  (波位表配置在这里)    AntennaScanModel                     │
│                   (扫描模型 - 职责不清)                      │
│                   - 拥有天线指针（不是拥有）                 │
│                   - 管理波位表                               │
└─────────────────────────────────────────────────────────────┘
```

### 重构后

```
┌─────────────────────────────────────────────────────────────┐
│ 新架构（清晰）                                               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  RadarConfig ────────────┐                                  │
│    - antenna: AntennaConfig (只含物理配置)                   │
│    - beam_table: BeamTableConfig (独立配置)                  │
│                                                             │
│  ┌─────────────────────────────────────────┐                │
│  │      AntennaModel                       │                │
│  │      (相控阵天线物理模型)               │                │
│  │                                         │                │
│  │  - 计算增益                             │                │
│  │  - 计算方向图                           │                │
│  │  - 配置精简：model_type, weights,       │                │
│  │    num_elements, spacing, peak_gain     │                │
│  └─────────────────────────────────────────┘                │
│                     ↑                                       │
│              (被拥有/被调用)                                │
│                     ↑                                       │
│  ┌──────────────────┴───────────────┐                       │
│  │       BeamScanner                │                       │
│  │       (波束扫描控制器)            │                       │
│  │                                  │                       │
│  │  - 直接拥有 AntennaModel         │                       │
│  │  - 拥有 BeamTableConfig          │                       │
│  │  - 管理扫描顺序                  │                       │
│  │  - 对外提供统一接口              │                       │
│  └──────────────────────────────────┘                       │
└─────────────────────────────────────────────────────────────┘
```

---

## 命名变更

| 旧命名 | 新命名 | 理由 |
|-------|-------|------|
| `PhasedArrayAntenna` | `AntennaModel` | 简洁，强调是"物理模型" |
| `AntennaScanModel` | `BeamScanner` | 准确反映"扫描控制器"职责 |
| `AntennaOptions` + `AntennaPhysicalParams` | `AntennaConfig` (扁平化) | 合并，避免过度拆分 |
| `cfg.antenna.beam_table_cfg` | `cfg.beam_table` | 配置归属清晰 |

---

## 文件变更

### 新增文件
- `include/antenna/antenna_model.h`
- `src/antenna/antenna_model.cpp`
- `include/antenna/beam_scanner.h`
- `src/antenna/beam_scanner.cpp`

### 删除文件
- `include/antenna/phased_array_antenna.h`
- `src/antenna/phased_array_antenna.cpp`
- `include/antenna/antenna_scan_model.h`
- `src/antenna/antenna_scan_model.cpp`

### 更新的文件
- `include/antenna/antenna_config.hpp` - 精简配置结构，添加 JSON 序列化
- `include/core/radar_config.hpp` - `beam_table` 独立为顶层成员
- `src/core/configuration_manager.cpp` - 更新 JSON 解析/序列化逻辑
- `src/core/simulation_engine.cpp` - 更新调用 `beam_scanner_`
- `include/core/simulation_engine.h` - 更新成员变量
- `include/clutter/clutter_engine.h` - 更新天线类型引用
- `src/clutter/clutter_engine.cpp` - 更新天线类型引用
- `include/clutter/sea_clutter_model.h` - 更新天线类型引用
- `src/clutter/sea_clutter_model.cpp` - 更新天线类型引用
- `include/target/target_kinematics.h` - 更新 `BeamView.antenna` 类型
- `include/target/target_engine.h` - 更新 include
- `config/radar_config.json` - 更新配置格式

---

## 配置格式变更

### 旧格式
```json
{
  "antenna": {
    "options": {
      "model_type": 0,
      "weight_type_az": 0,
      "weight_type_el": 0
    },
    "params": {
      "num_elements_az": 16,
      "num_elements_el": 8,
      "spacing_az_lambda": 0.5,
      "spacing_el_lambda": 0.5,
      "peak_gain_db": 30.0,
      "beam_table_path": ""
    },
    "beam_table_cfg": {
      "type": "azimuth_scan",
      "az_start_deg": -30,
      "az_end_deg": 30,
      "az_step_deg": 10,
      "elevation_deg": 0.0,
      "beams": [[-30, 0], [-20, 0], ...]
    }
  }
}
```

### 新格式
```json
{
  "antenna": {
    "model_type": 0,
    "weight_type_az": 0,
    "weight_type_el": 0,
    "num_elements_az": 16,
    "num_elements_el": 8,
    "spacing_az_lambda": 0.5,
    "spacing_el_lambda": 0.5,
    "peak_gain_db": 30.0
  },
  "beam_table": {
    "type": "azimuth_scan",
    "az_start_deg": -30,
    "az_end_deg": 30,
    "az_step_deg": 10,
    "elevation_deg": 0.0,
    "beams": [[-30, 0], [-20, 0], ...]
  }
}
```

---

## 设计原则总结

### 1. 职责分离
- `AntennaModel` 只负责物理计算（增益、方向图）
- `BeamScanner` 负责扫描逻辑（波位切换、顺序管理）

### 2. 配置归属清晰
- 天线物理配置属于 `AntennaConfig`
- 波位表配置独立为 `BeamTableConfig`，与 `AntennaConfig` 同级

### 3. 所有权清晰
- `BeamScanner` **直接拥有** `AntennaModel`（不是指针引用）
- 避免外部注入导致的依赖混乱

### 4. 命名反映意图
- `Model` → 物理模型
- `Scanner`/`Controller` → 控制器/协调器

---

## 后续计划

下一步重构**目标模块**，参考天线模块的设计原则：

```
┌─────────────────────────────────────────────────────────────┐
│ 目标模块期望架构                                             │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  TargetEngine (统一管理器)                                  │
│    - 拥有 TargetManager (状态管理)                          │
│    - 拥有 TargetKinematics (运动学计算)                     │
│    - 拥有 TargetEchoSynthesizer (回波合成)                  │
│    - 对外提供统一接口                                       │
│                                                             │
│  当前问题：                                                  │
│  - TargetManager 在 SimulationEngine 中独立存在              │
│  - 与 TargetEngine 没有隶属关系                             │
│  - 状态更新逻辑散落在外层循环                               │
└─────────────────────────────────────────────────────────────┘
```

---

## 编译输出

```
[  4%] Building CXX object CMakeFiles/radar_core.dir/src/core/configuration_manager.cpp.o
[ 22%] Built target radar_core
[ 27%] Building CXX object CMakeFiles/radar_antenna.dir/src/antenna/antenna_model.cpp.o
[ 36%] Built target radar_antenna
[ 59%] Built target radar_clutter
[ 81%] Built target radar_target
[ 90%] Built target radar_noise
[100%] Built target radar_app
```

**警告**: 仅有少量未使用参数警告，不影响功能。
