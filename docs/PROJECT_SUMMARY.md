# 雷达仿真系统项目总结

**日期**: 2026-04-02
**版本**: v2.0 (重构版)

---

## 项目概览

这是一个基于 C++17 的雷达仿真系统，用于生成相控阵雷达的 IQ 回波数据。

### 核心功能

- **波形生成**: LFM、NLFM、相位编码等
- **天线波束扫描**: 相控阵天线方向图仿真
- **目标回波**: 点目标运动学与 Swerling 起伏模型
- **海杂波**: Morchin 模型 + K 分布
- **噪声**: 多种噪声功率计算模式
- **数据导出**: IQ 数据、目标轨迹、配置参数

---

## 模块架构

```
┌─────────────────────────────────────────────────────────────┐
│                     SimulationEngine                         │
│   (仿真引擎 - 统一调度)                                      │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│   ┌─────────────────┐  ┌─────────────────┐                  │
│   │ WaveformGenerator│  │   BeamScanner   │                  │
│   │  (波形生成器)    │  │  (波束扫描器)    │                  │
│   ├─────────────────┤  ├─────────────────┤                  │
│   │ - WaveformConfig│  │ - AntennaConfig │                  │
│   │                 │  │ - BeamTableConfig                  │
│   └─────────────────┘  │ - AntennaModel   │                  │
│                        └─────────────────┘                  │
│                                                              │
│   ┌─────────────────┐  ┌─────────────────┐                  │
│   │  TargetEngine   │  │  ClutterEngine  │                  │
│   │  (目标引擎)      │  │  (杂波引擎)      │                  │
│   ├─────────────────┤  ├─────────────────┤                  │
│   │ - TargetManager │  │ - SeaClutterModel│                 │
│   │ - TargetConfig  │  │ - SeaClutterConfig│                │
│   │ - Kinematics    │  └─────────────────┘                  │
│   │ - EchoSynthesizer│                                      │
│   └─────────────────┘  ┌─────────────────┐                  │
│                        │   NoiseEngine   │                  │
│                        │   (噪声引擎)     │                  │
│                        ├─────────────────┤                  │
│                        │ - NoiseConfig   │                  │
│                        └─────────────────┘                  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

---

## 目录结构

```
Radar/
├── CMakeLists.txt              # 根配置
├── rebuild.sh                  # 重建脚本
├── config/
│   └── radar_config.json       # 配置文件
├── include/                    # 头文件
│   ├── core/                   # 核心模块
│   │   ├── radar_config.hpp    # 总配置
│   │   ├── radar_system_params.hpp
│   │   ├── simulation_engine.h
│   │   ├── configuration_manager.h
│   │   ├── data_exporter.h
│   │   ├── types.h
│   │   ├── math_utils.h
│   │   └── udp/                # UDP 发送
│   ├── antenna/                # 天线模块（已重构）
│   │   ├── antenna_config.hpp  # 扁平化配置
│   │   ├── antenna_model.h     # 物理模型
│   │   └── beam_scanner.h      # 波束控制器
│   ├── target/                 # 目标模块（已重构）
│   │   ├── target_config.hpp   # 扁平化配置
│   │   ├── target_engine.h     # 统一引擎
│   │   ├── target_manager.h
│   │   ├── target_kinematics.h
│   │   └── target_echo_synthesizer.h
│   ├── clutter/                # 杂波模块
│   │   ├── sea_clutter_config.hpp
│   │   ├── sea_clutter_model.h
│   │   └── clutter_engine.h
│   ├── noise/                  # 噪声模块
│   │   ├── noise_config.hpp
│   │   └── noise_engine.h
│   └── waveform/               # 波形模块
│       ├── waveform_config.hpp
│       └── waveform_generator.h
├── src/                        # 源代码
├── app/
│   └── main.cpp                # 程序入口
├── docs/                       # 文档
│   ├── refactoring/            # 重构日志
│   ├── superpowers/            # 设计规范
│   └── ...
└── out/                        # 输出目录
    ├── bin/radar               # 可执行文件
    └── lib/*.a                 # 静态库
```

---

## 配置结构

### 扁平化模块（已完成）

| 模块 | 配置文件 | 状态 |
|-----|---------|------|
| 天线 | `AntennaConfig` | ✅ 扁平化 |
| 波位表 | `BeamTableConfig` | ✅ 独立配置 |
| 目标 | `TargetConfig` | ✅ 扁平化 |

### 嵌套结构模块（保留）

| 模块 | 配置文件 | 结构 |
|-----|---------|------|
| 噪声 | `NoiseConfig` | `options` + `params` |
| 杂波 | `SeaClutterConfig` | `options` + `params` |
| 波形 | `WaveformConfig` | `options` + `params` |

---

## 设计模式

### 1. 控制器 + 物理模型分离

```
┌─────────────────────────────────────┐
│ Controller (控制器)                  │
│ - 负责编排和调度                     │
│ - 拥有物理模型                       │
│ - 对外提供统一接口                   │
├─────────────────────────────────────┤
│ PhysicalModel (物理模型)             │
│ - 负责纯物理计算                     │
│ - 无状态或轻状态                     │
│ - 被控制器拥有                       │
└─────────────────────────────────────┘
```

**示例**:
- `BeamScanner` (控制器) + `AntennaModel` (物理模型)
- `TargetEngine` (控制器) + `TargetKinematics`/`TargetEchoSynthesizer` (物理模型)

### 2. 配置扁平化

将嵌套的 `Options` + `Params` 结构合并为单一的扁平结构：

```cpp
// 旧
struct Config { Options opt; Params p; };
config.options.enabled
config.params.num_elements

// 新
struct Config {
    bool enabled = true;
    int num_elements = 16;
};
config.enabled
config.num_elements
```

### 3. 所有权模式

- 控制器 **直接拥有** 物理模型（值语义，非指针）
- 外部通过控制器访问，不直接接触物理模型

---

## 模块职责

| 模块 | 职责 | 关键类 |
|-----|------|--------|
| `SimulationEngine` | 仿真流程调度 | 扫描循环、CPI 处理 |
| `BeamScanner` | 波束扫描控制 | 波位表管理、天线增益查询 |
| `AntennaModel` | 天线方向图计算 | UPA/ULA 阵列因子 |
| `TargetEngine` | 目标全生命周期管理 | 状态管理、回波生成 |
| `TargetManager` | 目标运动学更新 | 位置/速度/加速度外推 |
| `TargetKinematics` | 轨迹生成 | 逐脉冲几何量计算 |
| `TargetEchoSynthesizer` | 回波合成 | 延迟、相位、幅度调制 |
| `ClutterEngine` | 杂波回波生成 | 海杂波功率计算 |
| `SeaClutterModel` | 杂波物理模型 | Morchin 模型、K 分布 |
| `NoiseEngine` | 噪声添加 | 高斯白噪声生成 |
| `WaveformGenerator` | 发射波形生成 | LFM/NLFM/相位编码 |
| `DataExporter` | 数据导出 | IQ 数据、CSV、JSON |
| `UdpSender` | 网络发送 | 实时 IQ 数据传输 |

---

## 构建系统

### CMake 配置

```bash
# 重建
./rebuild.sh

# 或手动执行
rm -rf build && mkdir build && cd build
cmake ..
cmake --build .
```

### 输出

```
out/
├── bin/radar           # 可执行文件
└── lib/
    ├── libradar_core.a
    ├── libradar_antenna.a
    ├── libradar_target.a
    ├── libradar_clutter.a
    └── libradar_noise.a
```

---

## 运行方式

```bash
# 运行仿真
./out/bin/radar

# 指定持续时间
./out/bin/radar --duration 1
```

### 配置修改

编辑 `config/radar_config.json`：
- 雷达参数（频率、PRF、带宽等）
- 天线参数（阵元数、增益等）
- 波位表（扫描范围、步长）
- 目标参数（位置、速度、RCS）
- 海杂波参数（海况、模型参数）

---

## 重构历史

### 2026-04-02: 天线模块重构
- 分离 `AntennaModel` (物理) 和 `BeamScanner` (控制器)
- 扁平化 `AntennaConfig`
- 独立 `BeamTableConfig`

### 2026-04-02: 目标模块重构
- `TargetEngine` 统一拥有 `TargetManager`
- 扁平化 `TargetConfig`
- 简化 `generate()` 接口
- `TargetEngine` 自动生成默认目标

### 2026-04-02: Bug 修复
- 修复 `BeamScanner` 未初始化 `AntennaModel` 导致的段错误

---

## 代码统计

| 类别 | 数量 |
|-----|------|
| 头文件 (.h/.hpp) | 17 |
| 源文件 (.cpp) | ~15 |
| 模块 | 6 (core/antenna/target/clutter/noise/waveform) |
| 文档 | ~10 |

---

## 后续工作

### 可选优化

1. **配置统一**: 将噪声和杂波模块也扁平化
2. **单元测试**: 增加各模块覆盖率
3. **性能优化**: 并行化 CPI 处理
4. **新特性**: 
   - 地杂波模型
   - 多径效应
   - 极化仿真

### 技术债务

- `NoiseConfig` 和 `SeaClutterConfig` 仍为嵌套结构
- `print()` 函数中部分访问仍使用旧格式

---

## 参考资料

- [天线模块重构日志](refactoring/2026-04-02-antenna-module-refactoring.md)
- [目标模块重构总结](refactoring/2026-04-02-target-module-summary.md)
- [目标模块重构日志](refactoring/2026-04-02-target-module-refactoring.md)
- [配置架构设计](superpowers/specs/2026-03-31-configuration-architecture-refactoring-design.md)
