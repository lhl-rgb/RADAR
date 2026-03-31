# Parameter Configuration Refactoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor RadarParams into layered architecture with RadarSystemParams for global shared parameters and module-private configs.

**Architecture:** Split large RadarParams struct into RadarSystemParams (global) + nested namespace configs (antenna, waveform, noise, clutter, target). RadarConfig serves as composition container. Each Engine receives only the parameters it needs.

**Tech Stack:** C++17, Eigen, Google Test

---

## File Structure

### New files to create:
```
include/
├── core/
│   ├── radar_system_params.h    # Global shared params
│   ├── waveform_config.h        # Waveform module config
│   └── radar_config.h           # Composition container
├── antenna/
│   └── antenna_config.h         # Antenna module config
├── noise/
│   └── noise_config.h           # Noise module config
├── clutter/
│   └── sea_clutter_config.h     # Sea clutter config
├── target/
│   └── target_config.h          # Target module config

src/
├── core/
│   ├── radar_system_params.cpp
│   ├── waveform_config.cpp
│   └── radar_config.cpp
├── antenna/
│   └── antenna_config.cpp
├── noise/
│   └── noise_config.cpp
├── clutter/
│   └── sea_clutter_config.cpp
├── target/
│   └── target_config.cpp
```

### Files to modify:
- `include/core/waveform_generator.h` — change constructor signature
- `src/core/waveform_generator.cpp` — update implementation
- `include/noise/noise_engine.h` — change constructor signature
- `src/noise/noise_engine.cpp` — update implementation
- `include/clutter/clutter_engine.h` — change constructor signature
- `src/clutter/clutter_engine.cpp` — update implementation
- `include/target/target_engine.h` — change constructor signature
- `src/target/target_engine.cpp` — update implementation
- `include/core/antenna_set.h` — change constructor signature (if exists)
- `src/core/antenna_set.cpp` — update implementation
- `test/radar_tests.cpp` — update test cases
- `app/main.cpp` — update usage
- `CMakeLists.txt` — add new source files

### Files to delete:
- `include/core/radar_params.h`
- `src/core/radar_params.cpp`

---

## Task 1: Create RadarSystemParams header

**Files:**
- Create: `include/core/radar_system_params.h`

- [ ] **Step 1: Write RadarSystemParams header file**

```cpp
// include/core/radar_system_params.h
#pragma once

#include "types.h"
#include <string>

namespace radar {

/**
 * @brief 全局共享雷达系统参数
 * @details 所有模块都依赖的基础物理参数和派生参数
 */
struct RadarSystemParams {
    // ===== 基础物理参数 =====
    Scalar fc_hz = 10.0e9;              ///< 载频 (Hz)
    Scalar prf_hz = 1600.0;             ///< 脉冲重复频率 (Hz)
    Scalar fs_hz = 40.0e6;              ///< ADC采样率 (Hz)
    Scalar bw_hz = 20.0e6;              ///< 信号带宽 (Hz)
    Scalar pulse_width_s = 20.0e-6;     ///< 脉冲宽度 (s)
    Scalar peak_power_w = 5000.0;       ///< 峰值发射功率 (W)
    int pulses_per_cpi = 32;            ///< 每 CPI 脉冲数

    // ===== 场景参数 =====
    Scalar min_range_m = 1000.0;        ///< 最小作用距离 (m)
    Scalar max_range_m = 120000.0;      ///< 最大作用距离 (m)
    GeoCoord radar_location{0.0, 0.0, 0.0}; ///< 雷达地理位置
    Scalar antenna_height_m = 15.0;     ///< 天线安装高度 (m)

    // ===== 系统损耗 =====
    Scalar noise_figure_db = 4.0;       ///< 接收机噪声系数 (dB)
    Scalar system_loss_db = 6.0;        ///< 系统总损耗 (dB)

    // ===== 派生参数（由 compute_derived_params 计算） =====
    Scalar wavelength_m;                ///< 波长 = C / fc
    Scalar pri_s;                       ///< 脉冲重复间隔 = 1 / prf
    Scalar duty_cycle;                  ///< 占空比 = pulse_width / pri
    Scalar cpi_duration_s;              ///< CPI 时长 = pulses_per_cpi / prf
    Scalar average_power_w;             ///< 平均功率 = peak_power * duty_cycle
    Scalar energy_per_pulse_j;          ///< 每脉冲能量 = peak_power * pulse_width
    Scalar bandwidth_time_product;      ///< 时带宽积 = bw * pulse_width

    Scalar range_resolution_m;          ///< 距离分辨率 = C / (2 * bw)
    Scalar range_bin_size_m;            ///< 距离 bin = C / (2 * fs)
    Scalar velocity_resolution_mps;     ///< 速度分辨率 = λ * prf / (2 * N)
    Scalar velocity_bin_size_mps;       ///< 速度 bin = λ / (2 * cpi_duration)

    Scalar max_unambiguous_range_m;     ///< 最大非模糊距离 = C / (2 * prf)
    Scalar max_unambiguous_velocity_mps;///< 最大非模糊速度 = λ * prf / 4
    Scalar max_doppler_hz;              ///< 最大非模糊多普勒 = prf / 2

    int samples_per_pulse;              ///< 每脉冲采样点数

    Scalar noise_figure_linear;         ///< 噪声系数线性值
    Scalar system_loss_linear;          ///< 系统损耗线性值

    RadarSystemParams();

    void compute_derived_params();
    bool validate(std::string& error) const;
    void print() const;
};

}  // namespace radar
```

- [ ] **Step 2: Commit header file**

```bash
git add include/core/radar_system_params.h
git commit -m "feat(params): add RadarSystemParams header"
```

---

## Task 2: Create RadarSystemParams implementation

**Files:**
- Create: `src/core/radar_system_params.cpp`
- Test: `test/radar_tests.cpp` (add test in next task)

- [ ] **Step 1: Write RadarSystemParams implementation**

```cpp
// src/core/radar_system_params.cpp
#include "core/radar_system_params.h"
#include "core/math_utils.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace radar {

RadarSystemParams::RadarSystemParams() {
    compute_derived_params();
}

void RadarSystemParams::compute_derived_params() {
    const Scalar safe_fc = math::clamp_positive_eps(fc_hz);
    const Scalar safe_bw = math::clamp_positive_eps(bw_hz);
    const Scalar safe_prf = math::clamp_positive_eps(prf_hz);
    const Scalar safe_fs = math::clamp_positive_eps(fs_hz);
    const Scalar safe_pulse_width = math::clamp_positive_eps(pulse_width_s);
    const Scalar safe_pulses = std::max<Scalar>(static_cast<Scalar>(pulses_per_cpi), 1.0);

    // 波长
    wavelength_m = C / safe_fc;

    // PRI 和占空比
    pri_s = 1.0 / safe_prf;
    duty_cycle = safe_pulse_width / pri_s;

    // CPI 时长
    cpi_duration_s = safe_pulses / safe_prf;

    // 功率和能量
    average_power_w = peak_power_w * duty_cycle;
    energy_per_pulse_j = peak_power_w * safe_pulse_width;

    // 时带宽积
    bandwidth_time_product = safe_bw * safe_pulse_width;

    // 距离相关
    range_resolution_m = C / (2.0 * safe_bw);
    range_bin_size_m = C / (2.0 * safe_fs);
    max_unambiguous_range_m = C / (2.0 * safe_prf);

    // 速度和多普勒相关
    velocity_resolution_mps = wavelength_m * safe_prf / (2.0 * safe_pulses);
    velocity_bin_size_mps = wavelength_m / (2.0 * cpi_duration_s);
    max_unambiguous_velocity_mps = wavelength_m * safe_prf / 4.0;
    max_doppler_hz = safe_prf / 2.0;

    // 线性值
    noise_figure_linear = math::db_to_linear(noise_figure_db);
    system_loss_linear = math::db_to_linear(system_loss_db);

    // 每脉冲采样点数
    const Scalar range_span_m = math::clamp_nonnegative(max_range_m - min_range_m);
    const Scalar fast_time_window_s =
        (2.0 * range_span_m / C) + math::clamp_nonnegative(pulse_width_s);
    samples_per_pulse =
        std::max(1, static_cast<int>(std::ceil(fast_time_window_s * safe_fs)));
}

bool RadarSystemParams::validate(std::string& error) const {
    // 基础参数检查
    if (fc_hz <= 0.0 || bw_hz <= 0.0 || pulse_width_s <= 0.0 ||
        prf_hz <= 0.0 || peak_power_w <= 0.0) {
        error = "Invalid basic parameters (fc, bw, pulse_width, prf, peak_power must be positive)";
        return false;
    }

    if (fs_hz <= 0.0 || pulses_per_cpi <= 0) {
        error = "Invalid sampling parameters (fs must be positive, pulses_per_cpi must be >= 1)";
        return false;
    }

    if (noise_figure_db < 0.0 || system_loss_db < 0.0) {
        error = "Invalid loss parameters (noise_figure, system_loss must be non-negative)";
        return false;
    }

    if (antenna_height_m < 0.0) {
        error = "Invalid antenna height (must be non-negative)";
        return false;
    }

    if (min_range_m < 0.0 || max_range_m <= min_range_m) {
        error = "Invalid range parameters (min >= 0, max > min)";
        return false;
    }

    // 脉冲宽度不得超过 PRI
    if (pulse_width_s > pri_s) {
        error = "Pulse width exceeds PRI";
        return false;
    }

    // 复基带建模要求 fs >= bw
    if (fs_hz < bw_hz) {
        error = "Sampling rate must be >= bandwidth for complex baseband modeling";
        return false;
    }

    if (samples_per_pulse <= 0) {
        error = "Invalid samples_per_pulse";
        return false;
    }

    return true;
}

void RadarSystemParams::print() const {
    std::cout
        << "RadarSystemParams:\n"
        << "  fc_hz: " << fc_hz << "\n"
        << "  prf_hz: " << prf_hz << "\n"
        << "  fs_hz: " << fs_hz << "\n"
        << "  bw_hz: " << bw_hz << "\n"
        << "  pulse_width_s: " << pulse_width_s << "\n"
        << "  peak_power_w: " << peak_power_w << "\n"
        << "  pulses_per_cpi: " << pulses_per_cpi << "\n"
        << "  min_range_m: " << min_range_m << "\n"
        << "  max_range_m: " << max_range_m << "\n"
        << "  antenna_height_m: " << antenna_height_m << "\n"
        << "  noise_figure_db: " << noise_figure_db << "\n"
        << "  system_loss_db: " << system_loss_db << "\n"
        << "  wavelength_m: " << wavelength_m << "\n"
        << "  pri_s: " << pri_s << "\n"
        << "  duty_cycle: " << duty_cycle << "\n"
        << "  cpi_duration_s: " << cpi_duration_s << "\n"
        << "  average_power_w: " << average_power_w << "\n"
        << "  energy_per_pulse_j: " << energy_per_pulse_j << "\n"
        << "  bandwidth_time_product: " << bandwidth_time_product << "\n"
        << "  range_resolution_m: " << range_resolution_m << "\n"
        << "  range_bin_size_m: " << range_bin_size_m << "\n"
        << "  velocity_resolution_mps: " << velocity_resolution_mps << "\n"
        << "  velocity_bin_size_mps: " << velocity_bin_size_mps << "\n"
        << "  max_unambiguous_range_m: " << max_unambiguous_range_m << "\n"
        << "  max_unambiguous_velocity_mps: " << max_unambiguous_velocity_mps << "\n"
        << "  max_doppler_hz: " << max_doppler_hz << "\n"
        << "  samples_per_pulse: " << samples_per_pulse << "\n"
        << "  noise_figure_linear: " << noise_figure_linear << "\n"
        << "  system_loss_linear: " << system_loss_linear << "\n";
}

}  // namespace radar
```

- [ ] **Step 2: Commit implementation**

```bash
git add src/core/radar_system_params.cpp
git commit -m "feat(params): add RadarSystemParams implementation"
```

---

## Task 3: Add RadarSystemParams tests

**Files:**
- Modify: `test/radar_tests.cpp`

- [ ] **Step 1: Add RadarSystemParams test cases**

在 `test/radar_tests.cpp` 中添加以下测试：

```cpp
// === RadarSystemParams Tests ===

TEST(RadarSystemParamsTest, DefaultConstructionComputesDerived) {
    radar::RadarSystemParams params;

    // 检查派生参数已计算
    EXPECT_GT(params.wavelength_m, 0.0);
    EXPECT_GT(params.pri_s, 0.0);
    EXPECT_GT(params.range_resolution_m, 0.0);
    EXPECT_GT(params.samples_per_pulse, 0);

    // 检查波长计算正确 (C / fc)
    EXPECT_NEAR(params.wavelength_m, radar::C / 10.0e9, 1e-6);

    // 检查 PRI 计算 (1 / prf)
    EXPECT_NEAR(params.pri_s, 1.0 / 1600.0, 1e-9);
}

TEST(RadarSystemParamsTest, ValidateDetectsInvalidParams) {
    radar::RadarSystemParams params;
    std::string error;

    // 默认参数应有效
    EXPECT_TRUE(params.validate(error));

    // 无效载频
    params.fc_hz = -1.0;
    EXPECT_FALSE(params.validate(error));
    EXPECT_FALSE(error.empty());

    // 恢复并测试其他无效情况
    params.fc_hz = 10.0e9;
    params.pulse_width_s = 1.0;  // 超过 PRI
    EXPECT_FALSE(params.validate(error));
}

TEST(RadarSystemParamsTest, DerivedParamsFormulas) {
    radar::RadarSystemParams params;
    params.fc_hz = 5.0e9;
    params.bw_hz = 10.0e6;
    params.prf_hz = 2000.0;
    params.pulses_per_cpi = 64;
    params.compute_derived_params();

    // 波长 = C / fc
    EXPECT_NEAR(params.wavelength_m, radar::C / 5.0e9, 1e-6);

    // 距离分辨率 = C / (2 * bw)
    EXPECT_NEAR(params.range_resolution_m, radar::C / (2.0 * 10.0e6), 1e-4);

    // 最大非模糊距离 = C / (2 * prf)
    EXPECT_NEAR(params.max_unambiguous_range_m, radar::C / (2.0 * 2000.0), 1e-2);

    // 时带宽积 = bw * pulse_width
    EXPECT_NEAR(params.bandwidth_time_product, 10.0e6 * 20.0e-6, 1.0);
}
```

- [ ] **Step 2: Run tests to verify**

```bash
cd /home/lhlj/Radar && cmake --build build && ./build/test/radar_tests --gtest_filter="RadarSystemParams*"
```
Expected: All tests PASS

- [ ] **Step 3: Commit tests**

```bash
git add test/radar_tests.cpp
git commit -m "test(params): add RadarSystemParams test cases"
```

---

## Task 4: Create waveform::WaveformConfig

**Files:**
- Create: `include/core/waveform_config.h`
- Create: `src/core/waveform_config.cpp`

- [ ] **Step 1: Write WaveformConfig header**

```cpp
// include/core/waveform_config.h
#pragma once

#include "types.h"
#include <string>

namespace radar::waveform {

/**
 * @brief 波形模块私有配置参数
 */
struct WaveformConfig {
    WaveformType waveform_type = WaveformType::LFM;
    PhaseCodeType phase_code_type = PhaseCodeType::Barker13;
    WindowType nlfm_window_type = WindowType::Hamming;
    PolarizationType polarization = PolarizationType::HH;

    bool validate(std::string& error) const;
};

}  // namespace radar::waveform
```

- [ ] **Step 2: Write WaveformConfig implementation**

```cpp
// src/core/waveform_config.cpp
#include "core/waveform_config.h"

namespace radar::waveform {

bool WaveformConfig::validate(std::string& error) const {
    // WaveformConfig 的字段都是枚举类型，默认值必然有效
    // 无需额外检查
    return true;
}

}  // namespace radar::waveform
```

- [ ] **Step 3: Commit WaveformConfig**

```bash
git add include/core/waveform_config.h src/core/waveform_config.cpp
git commit -m "feat(params): add waveform::WaveformConfig"
```

---

## Task 5: Create antenna::AntennaConfig

**Files:**
- Create: `include/antenna/antenna_config.h`
- Create: `src/antenna/antenna_config.cpp`

- [ ] **Step 1: Write AntennaConfig header**

```cpp
// include/antenna/antenna_config.h
#pragma once

#include "core/types.h"
#include <string>

namespace radar::antenna {

/**
 * @brief 相控阵天线配置参数
 */
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

- [ ] **Step 2: Write AntennaConfig implementation**

```cpp
// src/antenna/antenna_config.cpp
#include "antenna/antenna_config.h"

namespace radar::antenna {

bool AntennaConfig::validate(std::string& error) const {
    if (num_elements_az <= 0) {
        error = "num_elements_az must be positive";
        return false;
    }
    if (spacing_az_lambda <= 0.0) {
        error = "spacing_az_lambda must be positive";
        return false;
    }
    if (peak_gain_db < 0.0) {
        error = "peak_gain_db must be non-negative";
        return false;
    }

    if (model_type == PhasedArrayModelType::UPA_2D) {
        if (num_elements_el <= 0) {
            error = "num_elements_el must be positive for UPA_2D";
            return false;
        }
        if (spacing_el_lambda <= 0.0) {
            error = "spacing_el_lambda must be positive for UPA_2D";
            return false;
        }
    }

    return true;
}

}  // namespace radar::antenna
```

- [ ] **Step 3: Commit AntennaConfig**

```bash
git add include/antenna/antenna_config.h src/antenna/antenna_config.cpp
git commit -m "feat(params): add antenna::AntennaConfig"
```

---

## Task 6: Create noise::NoiseConfig

**Files:**
- Create: `include/noise/noise_config.h`
- Create: `src/noise/noise_config.cpp`

- [ ] **Step 1: Write NoiseConfig header**

```cpp
// include/noise/noise_config.h
#pragma once

#include "core/types.h"
#include <cstdint>
#include <string>

namespace radar::noise {

/**
 * @brief 噪声模块私有配置参数
 * @details 注意：noise_figure_db 已移至 RadarSystemParams
 */
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

- [ ] **Step 2: Write NoiseConfig implementation**

```cpp
// src/noise/noise_config.cpp
#include "noise/noise_config.h"

namespace radar::noise {

bool NoiseConfig::validate(std::string& error) const {
    if (mode == NoiseLevelMode::ComplexSigma) {
        if (sigma_complex <= 0.0) {
            error = "sigma_complex must be positive for ComplexSigma mode";
            return false;
        }
    } else if (mode == NoiseLevelMode::NoisePower) {
        if (noise_power_w <= 0.0) {
            error = "noise_power_w must be positive for NoisePower mode";
            return false;
        }
    } else if (mode == NoiseLevelMode::ThermalKTB) {
        if (system_temperature_k <= 0.0) {
            error = "system_temperature_k must be positive for ThermalKTB mode";
            return false;
        }
        if (noise_bandwidth_hz <= 0.0) {
            error = "noise_bandwidth_hz must be positive for ThermalKTB mode";
            return false;
        }
    }

    return true;
}

}  // namespace radar::noise
```

- [ ] **Step 3: Commit NoiseConfig**

```bash
git add include/noise/noise_config.h src/noise/noise_config.cpp
git commit -m "feat(params): add noise::NoiseConfig"
```

---

## Task 7: Create clutter::SeaClutterConfig

**Files:**
- Create: `include/clutter/sea_clutter_config.h`
- Create: `src/clutter/sea_clutter_config.cpp`

- [ ] **Step 1: Write SeaClutterConfig header**

```cpp
// include/clutter/sea_clutter_config.h
#pragma once

#include "core/types.h"
#include <cstdint>
#include <string>

namespace radar::clutter {

/**
 * @brief Morchin 归一化后向散射模型配置
 */
struct MorchinConfig {
    Scalar a0_db = -40.0;
    Scalar a_g = 10.0;
    Scalar a_f = 0.0;
    Scalar a_s = 1.0;
    Scalar sea_state = 3.0;
    Scalar sin_psi_floor = 1e-4;

    bool validate(std::string& error) const;
};

/**
 * @brief 海杂波模块私有配置参数
 */
struct SeaClutterConfig {
    bool enabled = true;
    Scalar ground_range_min_m = -1.0;  ///< -1 表示跟随 RadarSystemParams
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

- [ ] **Step 2: Write SeaClutterConfig implementation**

```cpp
// src/clutter/sea_clutter_config.cpp
#include "clutter/sea_clutter_config.h"
#include "core/math_utils.h"

namespace radar::clutter {

bool MorchinConfig::validate(std::string& error) const {
    if (!math::is_finite(a0_db) || !math::is_finite(a_g) ||
        !math::is_finite(a_f) || !math::is_finite(a_s) ||
        !math::is_finite(sea_state) || !math::is_finite(sin_psi_floor)) {
        error = "MorchinConfig contains non-finite values";
        return false;
    }
    if (sin_psi_floor <= 0.0) {
        error = "sin_psi_floor must be positive";
        return false;
    }
    return true;
}

bool SeaClutterConfig::validate(std::string& error) const {
    if (!math::is_finite(ground_range_min_m) || !math::is_finite(ground_range_max_m) ||
        !math::is_finite(range_step_m) || !math::is_finite(beam_az_width_deg) ||
        !math::is_finite(az_step_deg) || !math::is_finite(k_shape_nu) ||
        !math::is_finite(doppler_center_hz) || !math::is_finite(doppler_sigma_hz)) {
        error = "SeaClutterConfig contains non-finite values";
        return false;
    }

    if (range_step_m <= 0.0) {
        error = "range_step_m must be positive";
        return false;
    }
    if (beam_az_width_deg <= 0.0) {
        error = "beam_az_width_deg must be positive";
        return false;
    }
    if (az_step_deg <= 0.0) {
        error = "az_step_deg must be positive";
        return false;
    }
    if (k_shape_nu <= 0.0) {
        error = "k_shape_nu must be positive";
        return false;
    }
    if (doppler_sigma_hz <= 0.0) {
        error = "doppler_sigma_hz must be positive";
        return false;
    }
    if (pool_length_factor <= 0) {
        error = "pool_length_factor must be positive";
        return false;
    }

    return morchin.validate(error);
}

}  // namespace radar::clutter
```

- [ ] **Step 3: Commit SeaClutterConfig**

```bash
git add include/clutter/sea_clutter_config.h src/clutter/sea_clutter_config.cpp
git commit -m "feat(params): add clutter::SeaClutterConfig"
```

---

## Task 8: Create target::TargetConfig

**Files:**
- Create: `include/target/target_config.h`
- Create: `src/target/target_config.cpp`

- [ ] **Step 1: Write TargetConfig header**

```cpp
// include/target/target_config.h
#pragma once

#include "core/types.h"
#include <cstdint>
#include <string>

namespace radar::target {

/**
 * @brief 目标模块私有配置参数
 */
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

- [ ] **Step 2: Write TargetConfig implementation**

```cpp
// src/target/target_config.cpp
#include "target/target_config.h"
#include "core/math_utils.h"

namespace radar::target {

bool TargetConfig::validate(std::string& error) const {
    if (!math::is_finite(beam_gate_threshold_db)) {
        error = "beam_gate_threshold_db must be finite";
        return false;
    }
    return true;
}

}  // namespace radar::target
```

- [ ] **Step 3: Commit TargetConfig**

```bash
git add include/target/target_config.h src/target/target_config.cpp
git commit -m "feat(params): add target::TargetConfig"
```

---

## Task 9: Create RadarConfig composition container

**Files:**
- Create: `include/core/radar_config.h`
- Create: `src/core/radar_config.cpp`

- [ ] **Step 1: Write RadarConfig header**

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

/**
 * @brief 雷达配置组合容器
 * @details 作为配置入口，组合所有模块配置
 */
struct RadarConfig {
    RadarSystemParams system;
    waveform::WaveformConfig waveform;
    antenna::AntennaConfig antenna;
    noise::NoiseConfig noise;
    clutter::SeaClutterConfig clutter;
    target::TargetConfig target;

    RadarConfig();

    void compute_derived_params();
    bool validate(std::string& error) const;
    bool validate_full(std::string& error) const;
    void print() const;

    bool load_from_json(const std::string& filepath);
    bool save_to_json(const std::string& filepath) const;
};

/**
 * @brief 雷达配置管理器
 */
class RadarConfigManager {
public:
    RadarConfigManager() = default;

    void add_config(const std::string& name, const RadarConfig& config);
    std::optional<RadarConfig> get_config(const std::string& name) const;
    void set_active(const std::string& name);
    const RadarConfig& get_active() const;

private:
    std::map<std::string, RadarConfig> configs_;
    std::string active_name_;
    RadarConfig active_config_;
};

}  // namespace radar
```

- [ ] **Step 2: Write RadarConfig implementation**

```cpp
// src/core/radar_config.cpp
#include "core/radar_config.h"

#include <iostream>
#include <stdexcept>

namespace radar {

RadarConfig::RadarConfig() {
    compute_derived_params();
}

void RadarConfig::compute_derived_params() {
    system.compute_derived_params();
}

bool RadarConfig::validate(std::string& error) const {
    if (!system.validate(error)) {
        return false;
    }
    if (!waveform.validate(error)) {
        return false;
    }
    if (!antenna.validate(error)) {
        return false;
    }
    if (!noise.validate(error)) {
        return false;
    }
    if (!clutter.validate(error)) {
        return false;
    }
    if (!target.validate(error)) {
        return false;
    }
    return validate_full(error);
}

bool RadarConfig::validate_full(std::string& error) const {
    // 跨模块一致性检查

    // 采样率 >= 带宽
    if (system.fs_hz < system.bw_hz) {
        error = "fs_hz must be >= bw_hz for complex baseband modeling";
        return false;
    }

    // 脉冲宽度 < PRI
    if (system.pulse_width_s > system.pri_s) {
        error = "pulse_width_s must be < pri_s";
        return false;
    }

    // 杂波多普勒中心在非模糊范围内
    const Scalar nyquist_hz = system.max_doppler_hz;
    if (clutter.doppler_center_hz < -nyquist_hz ||
        clutter.doppler_center_hz >= nyquist_hz) {
        error = "clutter doppler_center_hz must be within [-prf/2, prf/2)";
        return false;
    }

    // 杂波范围不超过雷达最大距离
    const Scalar clutter_min = (clutter.ground_range_min_m < 0.0)
        ? system.min_range_m : clutter.ground_range_min_m;
    const Scalar clutter_max = (clutter.ground_range_max_m < 0.0)
        ? system.max_range_m : clutter.ground_range_max_m;
    if (clutter_min < 0.0 || clutter_max <= clutter_min) {
        error = "Invalid clutter range";
        return false;
    }
    if (clutter_max > system.max_range_m) {
        error = "clutter range exceeds radar max_range";
        return false;
    }

    return true;
}

void RadarConfig::print() const {
    std::cout << "=== RadarConfig ===\n";
    system.print();
    std::cout << "\n[waveform]\n"
              << "  waveform_type: " << static_cast<int>(waveform.waveform_type) << "\n"
              << "  polarization: " << static_cast<int>(waveform.polarization) << "\n";
    std::cout << "\n[antenna]\n"
              << "  model_type: " << static_cast<int>(antenna.model_type) << "\n"
              << "  num_elements_az: " << antenna.num_elements_az << "\n"
              << "  peak_gain_db: " << antenna.peak_gain_db << "\n";
    std::cout << "\n[clutter]\n"
              << "  enabled: " << (clutter.enabled ? "true" : "false") << "\n"
              << "  k_shape_nu: " << clutter.k_shape_nu << "\n";
    std::cout << "\n[target]\n"
              << "  enabled: " << (target.enabled ? "true" : "false") << "\n";
}

bool RadarConfig::load_from_json(const std::string& filepath) {
    (void)filepath;
    std::cerr << "[RadarConfig] JSON import is disabled in current stage.\n";
    return false;
}

bool RadarConfig::save_to_json(const std::string& filepath) const {
    (void)filepath;
    std::cerr << "[RadarConfig] JSON export is disabled in current stage.\n";
    return false;
}

// RadarConfigManager implementation

void RadarConfigManager::add_config(const std::string& name, const RadarConfig& config) {
    configs_[name] = config;
    if (active_name_.empty()) {
        set_active(name);
    }
}

std::optional<RadarConfig> RadarConfigManager::get_config(const std::string& name) const {
    const auto it = configs_.find(name);
    if (it == configs_.end()) {
        return std::nullopt;
    }
    return it->second;
}

void RadarConfigManager::set_active(const std::string& name) {
    const auto it = configs_.find(name);
    if (it == configs_.end()) {
        throw std::invalid_argument("RadarConfigManager::set_active failed: unknown config '" + name + "'");
    }
    active_name_ = name;
    active_config_ = it->second;
}

const RadarConfig& RadarConfigManager::get_active() const {
    if (active_name_.empty()) {
        throw std::runtime_error("RadarConfigManager::get_active failed: no active config");
    }
    return active_config_;
}

}  // namespace radar
```

- [ ] **Step 3: Add RadarConfig tests**

在 `test/radar_tests.cpp` 中添加：

```cpp
// === RadarConfig Tests ===

TEST(RadarConfigTest, DefaultConstructionValid) {
    radar::RadarConfig config;
    std::string error;
    EXPECT_TRUE(config.validate(error));
}

TEST(RadarConfigTest, ValidateFullChecksCrossModule) {
    radar::RadarConfig config;
    std::string error;

    // 跨模块检查：fs < bw 应失败
    config.system.fs_hz = 10.0e6;  // 小于默认 bw_hz = 20.0e6
    config.system.compute_derived_params();
    EXPECT_FALSE(config.validate_full(error));
    EXPECT_FALSE(error.empty());

    // 恢复正常
    config.system.fs_hz = 40.0e6;
    config.system.compute_derived_params();
    EXPECT_TRUE(config.validate_full(error));
}

TEST(RadarConfigManagerTest, AddAndGetConfig) {
    radar::RadarConfigManager manager;
    radar::RadarConfig config1;
    config1.system.fc_hz = 5.0e9;

    manager.add_config("config1", config1);
    EXPECT_TRUE(manager.get_config("config1").has_value());
    EXPECT_FALSE(manager.get_config("nonexistent").has_value());

    manager.set_active("config1");
    const auto& active = manager.get_active();
    EXPECT_NEAR(active.system.fc_hz, 5.0e9, 1e-6);
}
```

- [ ] **Step 4: Run tests**

```bash
cd /home/lhlj/Radar && cmake --build build && ./build/test/radar_tests --gtest_filter="RadarConfig*"
```
Expected: All tests PASS

- [ ] **Step 5: Commit RadarConfig**

```bash
git add include/core/radar_config.h src/core/radar_config.cpp test/radar_tests.cpp
git commit -m "feat(params): add RadarConfig composition container and tests"
```

---

## Task 10: Update CMakeLists.txt

**Files:**
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add new source files to CMakeLists.txt**

在 `CMakeLists.txt` 中找到源文件列表，添加新文件：

```cmake
# Core module sources
set(CORE_SOURCES
    src/core/radar_system_params.cpp
    src/core/radar_config.cpp
    src/core/waveform_config.cpp
    src/core/waveform_generator.cpp
    src/core/antenna_set.cpp
    # ... other existing files
)

# Antenna module sources
set(ANTENNA_SOURCES
    src/antenna/antenna_config.cpp
    # ... other existing files
)

# Noise module sources
set(NOISE_SOURCES
    src/noise/noise_config.cpp
    src/noise/noise_engine.cpp
    # ... other existing files
)

# Clutter module sources
set(CLUTTER_SOURCES
    src/clutter/sea_clutter_config.cpp
    src/clutter/clutter_engine.cpp
    src/clutter/sea_clutter_model.cpp
    # ... other existing files
)

# Target module sources
set(TARGET_SOURCES
    src/target/target_config.cpp
    src/target/target_engine.cpp
    src/target/target_echo_synthesizer.cpp
    src/target/target_kinematics.cpp
    src/target/target_manager.cpp
    # ... other existing files
)
```

- [ ] **Step 2: Commit CMakeLists.txt**

```bash
git add CMakeLists.txt
git commit -m "chore: add new config source files to CMakeLists"
```

---

## Task 11: Modify WaveformGenerator

**Files:**
- Modify: `include/core/waveform_generator.h`
- Modify: `src/core/waveform_generator.cpp`

- [ ] **Step 1: Update WaveformGenerator header**

```cpp
// include/core/waveform_generator.h (modified section)
#pragma once
#include "types.h"
#include "radar_system_params.h"
#include "waveform_config.h"
#include <memory>
#include <string>
#include <vector>

namespace radar {

class WaveformGenerator {
public:
    WaveformGenerator() = default;
    explicit WaveformGenerator(const RadarSystemParams& sys,
                               const waveform::WaveformConfig& cfg);

    void set_params(const RadarSystemParams& sys,
                    const waveform::WaveformConfig& cfg);

    // ... rest of methods unchanged

private:
    RadarSystemParams sys_;
    waveform::WaveformConfig cfg_;
    ComplexVec waveform_;
    ComplexVec matched_filter_;
    ComplexVec spectrum_;

    ComplexVec generate_window(int length, WindowType type) const;
    std::vector<Scalar> generate_window_weights(int length, WindowType type) const;
};

}  // namespace radar
```

- [ ] **Step 2: Update WaveformGenerator implementation**

修改构造函数和 set_params：

```cpp
// src/core/waveform_generator.cpp (modified sections)

WaveformGenerator::WaveformGenerator(const RadarSystemParams& sys,
                                      const waveform::WaveformConfig& cfg)
    : sys_(sys), cfg_(cfg) {
    // 根据配置生成波形
}

void WaveformGenerator::set_params(const RadarSystemParams& sys,
                                    const waveform::WaveformConfig& cfg) {
    sys_ = sys;
    cfg_ = cfg;
}

// 其他方法中使用 sys_.fc_hz, sys_.bw_hz 等代替 params_.fc_hz 等
// 使用 cfg_.waveform_type 等代替 params_.waveform_type 等
```

- [ ] **Step 3: Commit WaveformGenerator changes**

```bash
git add include/core/waveform_generator.h src/core/waveform_generator.cpp
git commit -m "refactor(waveform): update WaveformGenerator to use RadarSystemParams + WaveformConfig"
```

---

## Task 12: Modify NoiseEngine

**Files:**
- Modify: `include/noise/noise_engine.h`
- Modify: `src/noise/noise_engine.cpp`

- [ ] **Step 1: Update NoiseEngine header**

```cpp
// include/noise/noise_engine.h (modified section)
#pragma once

#include "core/radar_system_params.h"
#include "noise/noise_config.h"
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>

namespace radar {

class NoiseEngine {
public:
    NoiseEngine();
    explicit NoiseEngine(const RadarSystemParams& sys,
                          const noise::NoiseConfig& cfg);

    bool set_params(const RadarSystemParams& sys,
                    const noise::NoiseConfig& cfg);

    const noise::NoiseConfig& config() const { return cfg_; }
    // ... rest of methods unchanged

private:
    RadarSystemParams sys_;
    noise::NoiseConfig cfg_;
    // ... other members
};

}  // namespace radar
```

- [ ] **Step 2: Update NoiseEngine implementation**

修改构造函数，在 ThermalKTB 模式下使用 `sys_.noise_figure_db`：

```cpp
// src/noise/noise_engine.cpp (modified sections)

NoiseEngine::NoiseEngine(const RadarSystemParams& sys,
                          const noise::NoiseConfig& cfg)
    : sys_(sys), cfg_(cfg) {
    // 初始化 RNG
    rng_.seed(cfg_.seed);
    // 计算噪声功率（使用 sys_.noise_figure_db）
    // ...
}

bool NoiseEngine::set_params(const RadarSystemParams& sys,
                              const noise::NoiseConfig& cfg) {
    std::string error;
    if (!cfg.validate(error)) {
        last_error_ = error;
        return false;
    }
    sys_ = sys;
    cfg_ = cfg;
    rng_.seed(cfg_.seed);
    // 重新计算噪声功率
    return true;
}
```

- [ ] **Step 3: Commit NoiseEngine changes**

```bash
git add include/noise/noise_engine.h src/noise/noise_engine.cpp
git commit -m "refactor(noise): update NoiseEngine to use RadarSystemParams + NoiseConfig"
```

---

## Task 13: Modify ClutterEngine

**Files:**
- Modify: `include/clutter/clutter_engine.h`
- Modify: `src/clutter/clutter_engine.cpp`

- [ ] **Step 1: Update ClutterEngine header**

```cpp
// include/clutter/clutter_engine.h (modified section)
#pragma once

#include "core/radar_system_params.h"
#include "clutter/sea_clutter_config.h"
#include "clutter/sea_clutter_model.h"
#include <string>

namespace radar {

class ClutterEngine {
public:
    ClutterEngine() = default;
    explicit ClutterEngine(const RadarSystemParams& sys,
                           const clutter::SeaClutterConfig& cfg);

    bool set_params(const RadarSystemParams& sys,
                    const clutter::SeaClutterConfig& cfg);

    bool generate_sea_clutter_cpi(const PhasedArrayAntenna& antenna,
                                  const AzEl& beam_pointing,
                                  int beam_index,
                                  const ComplexVec& tx_waveform,
                                  CpiEcho& out_clutter);
    // 注意：不再传入 RadarParams

    const SeaClutterModel& sea_model() const { return sea_model_; }
    const std::string& last_error() const { return last_error_; }

private:
    RadarSystemParams sys_;
    clutter::SeaClutterConfig cfg_;
    SeaClutterModel sea_model_;
    std::string last_error_;
};

}  // namespace radar
```

- [ ] **Step 2: Update ClutterEngine implementation**

```cpp
// src/clutter/clutter_engine.cpp (modified sections)

ClutterEngine::ClutterEngine(const RadarSystemParams& sys,
                              const clutter::SeaClutterConfig& cfg)
    : sys_(sys), cfg_(cfg) {
    sea_model_.set_params(cfg_);
}

bool ClutterEngine::set_params(const RadarSystemParams& sys,
                                const clutter::SeaClutterConfig& cfg) {
    sys_ = sys;
    cfg_ = cfg;
    return sea_model_.set_params(cfg_);
}

bool ClutterEngine::generate_sea_clutter_cpi(const PhasedArrayAntenna& antenna,
                                              const AzEl& beam_pointing,
                                              int beam_index,
                                              const ComplexVec& tx_waveform,
                                              CpiEcho& out_clutter) {
    // 使用 sys_ 中的参数，不再需要传入 RadarParams
    return sea_model_.generate_cpi_echo(sys_, antenna, beam_pointing,
                                         beam_index, tx_waveform, out_clutter);
}
```

- [ ] **Step 3: Commit ClutterEngine changes**

```bash
git add include/clutter/clutter_engine.h src/clutter/clutter_engine.cpp
git commit -m "refactor(clutter): update ClutterEngine to use RadarSystemParams + SeaClutterConfig"
```

---

## Task 14: Modify TargetEngine

**Files:**
- Modify: `include/target/target_engine.h`
- Modify: `src/target/target_engine.cpp`

- [ ] **Step 1: Update TargetEngine header**

```cpp
// include/target/target_engine.h (modified section)
#pragma once

#include "core/radar_system_params.h"
#include "target/target_config.h"
#include "core/antenna_set.h"
#include <cstdint>
#include <string>
#include <vector>

namespace radar {

class TargetEngine {
public:
    TargetEngine() = default;
    explicit TargetEngine(const RadarSystemParams& sys,
                          const target::TargetConfig& cfg);

    bool set_params(const RadarSystemParams& sys,
                    const target::TargetConfig& cfg);

    bool generate(const TargetList& active_targets,
                  const BeamView& beam,
                  const ComplexVec& tx_waveform,
                  CpiEcho& out_echo);
    // 注意：不再传入 RadarParams

    CpiEcho generate_or_throw(const TargetList& active_targets,
                              const BeamView& beam,
                              const ComplexVec& tx_waveform);

    const std::string& last_error() const { return last_error_; }

private:
    RadarSystemParams sys_;
    target::TargetConfig cfg_;
    std::string last_error_;
};

}  // namespace radar
```

- [ ] **Step 2: Update TargetEngine implementation**

```cpp
// src/target/target_engine.cpp (modified sections)

TargetEngine::TargetEngine(const RadarSystemParams& sys,
                            const target::TargetConfig& cfg)
    : sys_(sys), cfg_(cfg) {
}

bool TargetEngine::set_params(const RadarSystemParams& sys,
                              const target::TargetConfig& cfg) {
    sys_ = sys;
    cfg_ = cfg;
    return true;
}

bool TargetEngine::generate(const TargetList& active_targets,
                            const BeamView& beam,
                            const ComplexVec& tx_waveform,
                            CpiEcho& out_echo) {
    // 使用 sys_ 和 cfg_，不再传入 RadarParams
    // ... 实现
}
```

- [ ] **Step 3: Commit TargetEngine changes**

```bash
git add include/target/target_engine.h src/target/target_engine.cpp
git commit -m "refactor(target): update TargetEngine to use RadarSystemParams + TargetConfig"
```

---

## Task 15: Modify AntennaSet

**Files:**
- Modify: `include/core/antenna_set.h` (if exists, otherwise check existing location)
- Modify: `src/core/antenna_set.cpp`

- [ ] **Step 1: Update AntennaSet header**

```cpp
// include/core/antenna_set.h (modified section)
#pragma once

#include "radar_system_params.h"
#include "antenna/antenna_config.h"
#include "phased_array_antenna.h"
#include <string>
#include <vector>

namespace radar {

class AntennaSet {
public:
    AntennaSet() = default;
    explicit AntennaSet(const RadarSystemParams& sys,
                        const antenna::AntennaConfig& cfg);

    void set_params(const RadarSystemParams& sys,
                    const antenna::AntennaConfig& cfg);

    // ... rest of methods unchanged

private:
    RadarSystemParams sys_;
    antenna::AntennaConfig cfg_;
    PhasedArrayAntenna antenna_;
    // ... other members
};

}  // namespace radar
```

- [ ] **Step 2: Update AntennaSet implementation**

```cpp
// src/core/antenna_set.cpp (modified sections)

AntennaSet::AntennaSet(const RadarSystemParams& sys,
                       const antenna::AntennaConfig& cfg)
    : sys_(sys), cfg_(cfg) {
    antenna_.configure(sys_.wavelength_m, cfg_);
}

void AntennaSet::set_params(const RadarSystemParams& sys,
                             const antenna::AntennaConfig& cfg) {
    sys_ = sys;
    cfg_ = cfg;
    antenna_.configure(sys_.wavelength_m, cfg_);
}
```

- [ ] **Step 3: Commit AntennaSet changes**

```bash
git add include/core/antenna_set.h src/core/antenna_set.cpp
git commit -m "refactor(antenna): update AntennaSet to use RadarSystemParams + AntennaConfig"
```

---

## Task 16: Update main.cpp usage

**Files:**
- Modify: `app/main.cpp`

- [ ] **Step 1: Update main.cpp to use RadarConfig**

```cpp
// app/main.cpp (modified section)
#include "core/radar_config.h"
#include "core/waveform_generator.h"
#include "noise/noise_engine.h"
#include "clutter/clutter_engine.h"
#include "target/target_engine.h"
// ... other includes

int main() {
    // 使用 RadarConfig
    radar::RadarConfig config;
    config.compute_derived_params();

    std::string error;
    if (!config.validate(error)) {
        std::cerr << "Invalid config: " << error << "\n";
        return 1;
    }

    // 构造各 Engine
    radar::WaveformGenerator waveform_gen(config.system, config.waveform);
    radar::NoiseEngine noise_engine(config.system, config.noise);
    radar::ClutterEngine clutter_engine(config.system, config.clutter);
    radar::TargetEngine target_engine(config.system, config.target);
    radar::AntennaSet antenna_set(config.system, config.antenna);

    // ... rest of main logic
}
```

- [ ] **Step 2: Commit main.cpp**

```bash
git add app/main.cpp
git commit -m "refactor(main): update to use RadarConfig and new Engine signatures"
```

---

## Task 17: Delete old radar_params files

**Files:**
- Delete: `include/core/radar_params.h`
- Delete: `src/core/radar_params.cpp`

- [ ] **Step 1: Remove old files**

```bash
git rm include/core/radar_params.h src/core/radar_params.cpp
```

- [ ] **Step 2: Commit deletion**

```bash
git commit -m "refactor(params): remove old RadarParams files"
```

---

## Task 18: Final build and test verification

- [ ] **Step 1: Build project**

```bash
cd /home/lhlj/Radar && cmake --build build --clean-first
```
Expected: Build succeeds without errors

- [ ] **Step 2: Run all tests**

```bash
./build/test/radar_tests
```
Expected: All tests PASS

- [ ] **Step 3: Commit final verification**

```bash
git add -A && git status
# 如果有遗漏的文件变更，提交它们
git commit -m "chore: final verification after parameter refactoring"
```

---

## Task 19: Update dependent files (sea_clutter_model, etc.)

**Files:**
- Modify: `include/clutter/sea_clutter_model.h`
- Modify: `src/clutter/sea_clutter_model.cpp`
- Other dependent files that reference RadarParams

- [ ] **Step 1: Update SeaClutterModel to use RadarSystemParams**

检查 `sea_clutter_model.h/cpp`，将 `RadarParams` 参数改为 `RadarSystemParams`：

```cpp
// include/clutter/sea_clutter_model.h
#pragma once

#include "core/radar_system_params.h"
#include "clutter/sea_clutter_config.h"
// ... other includes

namespace radar {

class SeaClutterModel {
public:
    bool set_params(const clutter::SeaClutterConfig& cfg);
    bool generate_cpi_echo(const RadarSystemParams& sys,  // 改为 RadarSystemParams
                           const PhasedArrayAntenna& antenna,
                           const AzEl& beam_pointing,
                           int beam_index,
                           const ComplexVec& tx_waveform,
                           CpiEcho& out_echo);
    // ... rest
};

}  // namespace radar
```

- [ ] **Step 2: Commit SeaClutterModel changes**

```bash
git add include/clutter/sea_clutter_model.h src/clutter/sea_clutter_model.cpp
git commit -m "refactor(clutter): update SeaClutterModel to use RadarSystemParams"
```

---

## Summary

This plan creates:
- 6 new config header files
- 6 new config implementation files
- 1 RadarConfig composition container
- Comprehensive tests for all new components

And modifies:
- 5 Engine classes (WaveformGenerator, NoiseEngine, ClutterEngine, TargetEngine, AntennaSet)
- main.cpp usage
- CMakeLists.txt
- All dependent files

Total: ~25 files touched, ~50 atomic steps.