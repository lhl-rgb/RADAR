# Radar 系统架构图与数据流转图

本目录包含 Radar 回波仿真系统的完整架构图和数据流转图，每种图提供两种风格：**工程实现视角**和**汇报展示视角**。

---

## 📁 文件列表

### 系统架构图

| 文件 | 风格 | 适用场景 |
|------|------|----------|
| [`architecture-engineering.svg`](./architecture-engineering.svg) | 工程实现视角 | 开发文档、代码审查、新成员 onboarding |
| [`architecture-presentation.svg`](./architecture-presentation.svg) | 汇报展示视角 | PPT 汇报、项目展示、对外交流 |

### 数据流转图

| 文件 | 风格 | 适用场景 |
|------|------|----------|
| [`dataflow-engineering.svg`](./dataflow-engineering.svg) | 工程实现视角 | 详细设计、调试参考、数据流分析 |
| [`dataflow-presentation.svg`](./dataflow-presentation.svg) | 汇报展示视角 | 流程演示、方案讲解 |

---

## 🏗️ 系统架构概览

```
┌─────────────────────────────────────────────────────────────┐
│                        app/main.cpp                          │
│                        程序入口                               │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                  ConfigurationManager                        │
│                  JSON 配置加载/验证/参数计算                    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   SimulationEngine                           │
│              仿真引擎核心 - CPI 循环调度 / 波束扫描               │
├─────────────────────────────────────────────────────────────┤
│  ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────┐   │
│  │ waveform  │ │ antenna   │ │ target    │ │ clutter   │   │
│  │ 波形生成  │ │ 波束扫描  │ │ 目标引擎  │ │ 杂波引擎  │   │
│  └───────────┘ └───────────┘ └───────────┘ └───────────┘   │
│                                    ┌───────────┐            │
│                                    │ noise     │            │
│                                    │ 噪声引擎  │            │
│                                    └───────────┘            │
├─────────────────────────────────────────────────────────────┤
│  ┌───────────┐ ┌───────────┐                                 │
│  │ Data      │ │ Udp       │                                 │
│  │ Export    │ │ Sender    │                                 │
│  └───────────┘ └───────────┘                                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 📊 核心数据流

```
配置加载 → 引擎初始化 → 扫描循环 → 回波生成 → 数据输出

详细流程：
1. 加载 radar_config.json → RadarConfig 对象
2. 创建 SimulationEngine → 初始化所有子模块
3. for scan in 0..total_scans:
     for cpi in 0..beams_per_scan:
       - 获取当前波位 (azimuth, elevation)
       - 更新目标状态到当前时刻
       - 生成发射波形 (LFM/NLFM/相位编码)
       - 生成目标回波 (雷达方程 + Swerling 起伏)
       - 生成海杂波 (Morchin 模型 + K 分布调制)
       - 生成噪声 (复高斯白噪声)
       - 叠加：y_rx = y_target + y_clutter + y_noise
       - 收集 CPI 回波
4. 导出扫描数据：
   - IQ 数据 → data/iq_scan_X.dat
   - 目标快照 → data/targets_scan_X.csv
   - 配置参数 → data/config_params.json
   - UDP 发送 → 127.0.0.1:9000 (可选)
```

---

## 🔧 模块职责

| 模块 | 职责 | 关键类 |
|------|------|--------|
| **core** | 仿真引擎调度、配置管理、数据导出、UDP 发送 | `SimulationEngine`, `ConfigurationManager`, `DataExporter`, `UdpSender` |
| **waveform** | 发射波形生成（LFM/NLFM/相位编码） | `WaveformGenerator`, `WaveformConfig` |
| **antenna** | 相控阵天线方向图计算、波束扫描控制 | `BeamScanner`, `AntennaModel`, `AntennaConfig` |
| **target** | 目标运动学更新、回波合成 | `TargetEngine`, `TargetManager`, `TargetEchoSynthesizer` |
| **clutter** | 海杂波回波生成（Morchin 模型 + K 分布） | `ClutterEngine`, `SeaClutterModel` |
| **noise** | 复高斯白噪声生成 | `NoiseEngine`, `NoiseConfig` |

---

## 📐 默认系统参数

| 参数 | 值 |
|------|-----|
| 载波频率 | 10 GHz |
| 脉冲重复频率 (PRF) | 1.6 kHz |
| 信号带宽 | 25 MHz |
| 脉冲宽度 | 2 μs |
| 采样率 | 60 MHz |
| 峰值功率 | 5 kW |
| CPI 脉冲数 | 32 |
| 天线阵列 | 16×8 UPA |
| 扫描范围 | 60° ~ 120° |
| 方位步长 | 2° |

---

## 🎯 如何使用

### 在浏览器中查看
直接用浏览器打开任意 `.svg` 文件即可查看。

### 在 VSCode 中查看
安装 "SVG Viewer" 扩展后，右键 SVG 文件选择 "Open Preview"。

### 导出为 PNG
```bash
# 使用 Inkscape
inkscape --export-type=png --export-width=1920 architecture-engineering.svg

# 或使用 online-convert.com 等在线工具
```

---

## 📝 版本信息

- **项目版本**: Radar Echo Simulator v0.1.0
- **绘图日期**: 2026-04-13
- **作者**: Claude Code
