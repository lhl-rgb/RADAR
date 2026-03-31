---
name: Parameter Configuration Refactoring
description: Refactor radar parameter structure into layered architecture with module-private configs
type: project
created: 2026-03-31
---

# 雷达参数配置重构设计

## 概述

将现有 `RadarParams` 大结构拆分为分层架构：
- 全局共享的 `RadarSystemParams`
- 各模块私有的 `*Config` 结构
- 组合容器 `RadarConfig` 作为配置入口

## 目标

1. **模块隔离**：TargetConfig 只对 target 模块可见，SeaClutterConfig 只对 clutter 模块可见
2. **依赖清晰**：每个 Engine 只接收自己需要的参数
3. **参数分类**：区分"配置参数"（Config）和"运行时状态"（State）

---

## 1. 命名空间结构

```
namespace radar {
    // 全局共享参数
    struct RadarSystemParams { ... };

    // 组合容器
    struct RadarConfig { ... };

    // 各模块私有参数（嵌套命名空间）
    namespace antenna {
        struct AntennaConfig { ... };
    }
    namespace waveform {
        struct WaveformConfig { ... };
    }
    namespace noise {
        struct NoiseConfig { ... };
    }
    namespace clutter {
        struct SeaClutterConfig { ... };
    }
    namespace target {
        struct TargetConfig { ... };
        struct TargetState { ... };  // 运行时状态，保持不变
    }
}
```

**命名约定**：
- `*Params` → 改为 `*Config`（配置参数）
- `*State` 保持不变（运行时状态）

---

## 2. 文件结构

```
include/
├── core/
│   ├── types.h                      # 保持（枚举定义）
│   ├── radar_system_params.h        # 新增：全局共享参数
│   ├── radar_config.h               # 新增：参数组合容器
│   └── waveform_config.h            # 新增：波形配置
│
├── antenna/
│   └── antenna_config.h             # 新增：天线配置
│
├── noise/
│   └── noise_config.h               # 新增
│
├── clutter/
│   └── sea_clutter_config.h         # 新增
│
├── target/
│   ├── target_config.h              # 新增：目标配置
│   └── target_state.h               # 保持
│
src/
├── core/
│   ├── radar_system_params.cpp      # 新增
│   ├── radar_config.cpp             # 新增
│   └── waveform_config.cpp          # 新增
│
├── antenna/
│   └── antenna_config.cpp           # 新增
│
├── noise/
│   └── noise_config.cpp             # 新增
│
├── clutter/
│   └── sea_clutter_config.cpp       # 新增
│
├── target/
│   ├── target_config.cpp            # 新增
│   └── target_state.cpp             # 保持
```

**删除文件**：
- `include/core/radar_params.h`
- `src/core/radar_params.cpp`

---

## 3. RadarSystemParams 定义

全局共享参数，所有模块都依赖的基础物理参数。

```cpp
// include/core/radar_system_params.h
#pragma once

#include "types.h"
#include <string>

namespace radar {

struct RadarSystemParams {
    // ===== 基础物理参数 =====
    Scalar fc_hz = 10.0e9;              // 载频
    Scalar prf_hz = 1600.0;             // PRF
    Scalar fs_hz = 40.0e6;              // 采样率
    Scalar bw_hz = 20.0e6;              // 带宽
    Scalar pulse_width_s = 20.0e-6;     // 脉冲宽度
    Scalar peak_power_w = 5000.0;       // 峰值功率
    int pulses_per_cpi = 32;            // CPI脉冲数

    // ===== 场景参数 =====
    Scalar min_range_m = 1000.0;
    Scalar max_range_m = 120000.0;
    GeoCoord radar_location{0.0, 0.0, 0.0};
    Scalar antenna_height_m = 15.0;

    // ===== 系统损耗 =====
    Scalar noise_figure_db = 4.0;
    Scalar system_loss_db = 6.0;

    // ===== 派生参数 =====
    Scalar wavelength_m;                    // 波长 = C / fc
    Scalar pri_s;                           // 脉冲重复间隔 = 1 / prf
    Scalar duty_cycle;                      // 占空比 = pulse_width / pri
    Scalar cpi_duration_s;                  // CPI时长 = pulses_per_cpi / prf
    Scalar average_power_w;                 // 平均功率 = peak_power * duty_cycle
    Scalar energy_per_pulse_j;              // 每脉冲能量 = peak_power * pulse_width
    Scalar bandwidth_time_product;          // 时带宽积 = bw * pulse_width

    Scalar range_resolution_m;              // 距离分辨率 = C / (2 * bw)
    Scalar range_bin_size_m;                // 距离bin = C / (2 * fs)
    Scalar velocity_resolution_mps;         // 速度分辨率 = λ * prf / (2 * N)
    Scalar velocity_bin_size_mps;           // 速度bin = λ / (2 * cpi_duration)

    Scalar max_unambiguous_range_m;         // 最大非模糊距离 = C / (2 * prf)
    Scalar max_unambiguous_velocity_mps;    // 最大非模糊速度 = λ * prf / 4
    Scalar max_doppler_hz;                  // 最大非模糊多普勒 = prf / 2

    int samples_per_pulse;                  // 每脉冲采样点数

    // ===== 派生线性值 =====
    Scalar noise_figure_linear;
    Scalar system_loss_linear;

    RadarSystemParams();

    void compute_derived_params();
    bool validate(std::string& error) const;
    void print() const;
};

}  // namespace radar
```

---

## 4. 各模块私有配置

### 4.1 antenna::AntennaConfig

```cpp
// include/antenna/antenna_config.h
#pragma once

#include "core/types.h"
#include <string>

namespace radar::antenna {

struct AntennaConfig {
    PhasedArrayModelType model_type = PhasedArrayModelType::UPA_2D;
    int num_elements_az = 16;
    int num_elements_el = 12;
    Scalar spacing_az_lambda = 0.5;
    Scalar spacing_el_lambda = 0.5;
    AntennaWeightType weight_type_az = AntennaWeightType::Uniform;
    AntennaWeightType weight_type_el = AntennaWeightType::Uniform;
    Scalar peak_gain_db = 35.0;
    std::string beam_table_path;

    bool validate(std::string& error) const;
};

}  // namespace radar::antenna
```

### 4.2 waveform::WaveformConfig

```cpp
// include/core/waveform_config.h
#pragma once

#include "types.h"
#include <string>

namespace radar::waveform {

struct WaveformConfig {
    WaveformType waveform_type = WaveformType::LFM;
    PhaseCodeType phase_code_type = PhaseCodeType::Barker13;
    WindowType nlfm_window_type = WindowType::Hamming;
    PolarizationType polarization = PolarizationType::HH;

    bool validate(std::string& error) const;
};

}  // namespace radar::waveform
```

### 4.3 noise::NoiseConfig

```cpp
// include/noise/noise_config.h
#pragma once

#include "core/types.h"
#include <cstdint>
#include <string>

namespace radar::noise {

struct NoiseConfig {
    NoiseLevelMode mode = NoiseLevelMode::ComplexSigma;
    Scalar sigma_complex = 1.0e-3;
    Scalar noise_power_w = 1.0e-6;
    Scalar system_temperature_k = 290.0;
    Scalar noise_bandwidth_hz = 20.0e6;
    uint64_t seed = 12345;

    bool validate(std::string& error) const;
};

}  // namespace radar::noise
```

**注意**：删除原 `NoiseParams::noise_figure_db`，系统噪声系数统一使用 `RadarSystemParams::noise_figure_db`。

### 4.4 clutter::SeaClutterConfig

```cpp
// include/clutter/sea_clutter_config.h
#pragma once

#include "core/types.h"
#include <cstdint>
#include <string>

namespace radar::clutter {

struct MorchinConfig {
    Scalar a0_db = -40.0;
    Scalar a_g = 10.0;
    Scalar a_f = 0.0;
    Scalar a_s = 1.0;
    Scalar sea_state = 3.0;
    Scalar sin_psi_floor = 1e-4;
};

struct SeaClutterConfig {
    bool enabled = true;
    Scalar ground_range_min_m = -1.0;  // -1 表示跟随 RadarSystemParams
    Scalar ground_range_max_m = -1.0;
    Scalar range_step_m = 100.0;
    Scalar beam_az_width_deg = 3.0;
    Scalar az_step_deg = 0.1;
    Scalar k_shape_nu = 0.8;
    Scalar doppler_center_hz = 0.0;
    Scalar doppler_sigma_hz = 20.0;
    SeaClutterSequenceMode sequence_mode = SeaClutterSequenceMode::InTimeMode;
    uint64_t seed = 2026;
    int pool_length_factor = 32;
    MorchinConfig morchin;

    bool validate(std::string& error) const;
};

}  // namespace radar::clutter
```

### 4.5 target::TargetConfig

```cpp
// include/target/target_config.h
#pragma once

#include "core/types.h"
#include <cstdint>
#include <string>

namespace radar::target {

struct TargetConfig {
    bool enabled = true;
    bool enable_beam_gain = true;
    bool enable_two_way_propagation_loss = true;
    bool enable_phase = true;
    bool enable_swerling = true;
    uint64_t seed = 20260330ULL;
    Scalar beam_gate_threshold_db = -20.0;
    bool skip_out_of_beam_targets = false;

    bool validate(std::string& error) const;
};

}  // namespace radar::target
```

---

## 5. RadarConfig 组合容器

配置入口，用于一次性加载所有配置。

```cpp
// include/core/radar_config.h
#pragma once

#include "core/radar_system_params.h"
#include "core/waveform_config.h"
#include "antenna/antenna_config.h"
#include "noise/noise_config.h"
#include "clutter/sea_clutter_config.h"
#include "target/target_config.h"
#include <string>

namespace radar {

struct RadarConfig {
    RadarSystemParams system;
    waveform::WaveformConfig waveform;
    antenna::AntennaConfig antenna;
    noise::NoiseConfig noise;
    clutter::SeaClutterConfig clutter;
    target::TargetConfig target;

    RadarConfig();

    void compute_derived_params();  // 计算 system 中的派生参数
    bool validate(std::string& error) const;  // 全局验证（含跨模块一致性）
    bool validate_full(std::string& error) const;  // 跨模块一致性检查
    void print() const;

    // JSON 加载/保存（未来启用）
    bool load_from_json(const std::string& filepath);
    bool save_to_json(const std::string& filepath) const;
};

}  // namespace radar
```

---

## 6. 派生参数计算公式

| 派生参数 | 公式 | 说明 |
|---|---|---|
| `wavelength_m` | C / fc | 波长 |
| `pri_s` | 1 / prf | 脉冲重复间隔 |
| `duty_cycle` | pulse_width / pri | 占空比 |
| `cpi_duration_s` | pulses_per_cpi / prf | CPI时长 |
| `average_power_w` | peak_power * duty_cycle | 平均功率 |
| `energy_per_pulse_j` | peak_power * pulse_width | 每脉冲能量 |
| `bandwidth_time_product` | bw * pulse_width | 时带宽积 |
| `range_resolution_m` | C / (2 * bw) | 距离分辨率 |
| `range_bin_size_m` | C / (2 * fs) | 采样bin大小 |
| `velocity_resolution_mps` | λ * prf / (2 * N) | N = pulses_per_cpi |
| `velocity_bin_size_mps` | λ / (2 * cpi_duration) | CPI决定的bin |
| `max_unambiguous_range_m` | C / (2 * prf) | 非模糊距离 |
| `max_unambiguous_velocity_mps` | λ * prf / 4 | 非模糊速度 |
| `max_doppler_hz` | prf / 2 | 非模糊多普勒 |
| `samples_per_pulse` | ceil(fast_time_window * fs) | 现有计算方式 |
| `noise_figure_linear` | 10^(noise_figure_db/10) | 线性值 |
| `system_loss_linear` | 10^(system_loss_db/10) | 线性值 |

---

## 7. Engine 构造函数接口变化

| Engine | 现状 | 改后 |
|---|---|---|
| `WaveformGenerator` | `WaveformGenerator(const RadarParams& params)` | `WaveformGenerator(const RadarSystemParams& sys, const waveform::WaveformConfig& cfg)` |
| `NoiseEngine` | `NoiseEngine(const NoiseParams& params)` | `NoiseEngine(const RadarSystemParams& sys, const noise::NoiseConfig& cfg)` |
| `ClutterEngine` | `ClutterEngine(const SeaClutterParams& params)` | `ClutterEngine(const RadarSystemParams& sys, const clutter::SeaClutterConfig& cfg)` |
| `TargetEngine` | 无构造函数参数 | `TargetEngine(const RadarSystemParams& sys, const target::TargetConfig& cfg)` |
| `AntennaSet` | `AntennaSet(const RadarParams& params)` | `AntennaSet(const RadarSystemParams& sys, const antenna::AntennaConfig& cfg)` |

**设计原则**：
1. 构造时传入配置，避免运行时频繁设置
2. generate 方法不再传入配置，只传入动态数据

---

## 8. 参数验证职责划分

### 各模块自验证

```cpp
namespace radar::antenna {
    bool validate(const AntennaConfig& cfg, std::string& error);
}
namespace radar::waveform {
    bool validate(const WaveformConfig& cfg, std::string& error);
}
namespace radar::clutter {
    bool validate(const SeaClutterConfig& cfg, std::string& error);
}
namespace radar::target {
    bool validate(const TargetConfig& cfg, std::string& error);
}
namespace radar::noise {
    bool validate(const NoiseConfig& cfg, std::string& error);
}
```

### 全局验证

```cpp
bool validate(const RadarSystemParams& sys, std::string& error);
bool RadarConfig::validate_full(std::string& error) const;  // 跨模块一致性
```

### 跨模块一致性检查

| 检查项 | 说明 |
|---|---|
| `fs >= bw` | 采样率 >= 带宽 |
| `pulse_width < pri` | 脉冲宽度 < PRI |
| `doppler_center < prf/2` | 杂波多普勒中心在非模糊范围内 |
| `clutter_range <= max_range` | 杂波范围不超过雷达最大距离 |

---

## 9. 类型定义（枚举）归属

**保持在全局 `types.h`**，不移动到模块命名空间。

理由：
1. 枚举可能跨模块使用
2. 类型定义和配置数据是不同概念
3. 减少头文件依赖复杂度

---

## 10. RadarParamManager 处理

现有 `RadarParamManager` 改为 `RadarConfigManager`：

```cpp
class RadarConfigManager {
public:
    void add_config(const std::string& name, const RadarConfig& config);
    std::optional<RadarConfig> get_config(const std::string& name) const;
    void set_active(const std::string& name);
    const RadarConfig& get_active() const;
private:
    std::map<std::string, RadarConfig> configs_;
    std::string active_name_;
    RadarConfig active_config_;
};
```

---

## 11. 重构影响范围

### 需要修改的文件

1. **Engine 类**（构造函数、成员变量）：
   - `WaveformGenerator`
   - `NoiseEngine`
   - `ClutterEngine`
   - `TargetEngine`
   - `AntennaSet`

2. **调用方**（main.cpp, tests）：
   - 使用 `RadarConfig` 代替 `RadarParams`
   - 构造 Engine 时传入 `(sys, cfg)` 分离参数

3. **依赖关系**：
   - 各模块头文件不再依赖 `radar_params.h`
   - 只依赖自己需要的 config.h 和 `radar_system_params.h`

### 向后兼容（可选）

可保留 `radar_params.h` 作为 deprecated alias：

```cpp
// include/core/radar_params.h (deprecated)
#pragma once

#include "radar_config.h"

namespace radar {

// 向后兼容别名
using RadarParams = RadarConfig;
using PhasedArrayAntennaConfig = antenna::AntennaConfig;
using NoiseParams = noise::NoiseConfig;
using SeaClutterParams = clutter::SeaClutterConfig;
using TargetParams = target::TargetConfig;

}  // namespace radar

// 编译警告
#ifdef __GNUC__
#pragma GCC deprecated("RadarParams, use RadarConfig instead")
#endif
```

---

## 12. 实施顺序建议

1. 创建新文件（config.h/cpp）
2. 实现 RadarSystemParams + 各模块 Config
3. 实现 RadarConfig 组合容器
4. 修改 Engine 构造函数
5. 修改调用方
6. 删除旧文件
7. 更新 CMakeLists.txt
8. 运行测试验证

---

## 附录：参数归属对照表

| 原 RadarParams 字段 | 新归属 |
|---|---|
| `waveform_type` | waveform::WaveformConfig |
| `phase_code_type` | waveform::WaveformConfig |
| `nlfm_window_type` | waveform::WaveformConfig |
| `polarization` | waveform::WaveformConfig |
| `fc_hz` | RadarSystemParams |
| `bw_hz` | RadarSystemParams |
| `pulse_width_s` | RadarSystemParams |
| `prf_hz` | RadarSystemParams |
| `peak_power_w` | RadarSystemParams |
| `fs_hz` | RadarSystemParams |
| `pulses_per_cpi` | RadarSystemParams |
| `samples_per_pulse` | RadarSystemParams (派生) |
| `noise_figure_db` | RadarSystemParams |
| `noise_figure_linear` | RadarSystemParams (派生) |
| `system_loss_db` | RadarSystemParams |
| `system_loss_linear` | RadarSystemParams (派生) |
| `PhasedArrayAntennaConfig` | antenna::AntennaConfig |
| `antenna_height_m` | RadarSystemParams |
| `beam_table_path` | antenna::AntennaConfig |
| `min_range_m` | RadarSystemParams |
| `max_range_m` | RadarSystemParams |
| `radar_location` | RadarSystemParams |
| `SeaClutterParams` | clutter::SeaClutterConfig |
| `TargetParams` | target::TargetConfig |
| `wavelength_m` | RadarSystemParams (派生) |
| `range_resolution_m` | RadarSystemParams (派生) |
| `velocity_resolution_mps` | RadarSystemParams (派生) |
| `max_unambiguous_range_m` | RadarSystemParams (派生) |
| `max_unambiguous_velocity` | RadarSystemParams (派生) |