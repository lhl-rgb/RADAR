# 雷达仿真数据验证工具

## 概述

本工具用于验证雷达仿真结果，通过导出仿真数据并使用 MATLAB 进行回放分析，验证设置的目标参数是否正确体现在回波信号中。

## 文件说明

### C++ 导出模块
- `include/core/data_exporter.h` - 数据导出器头文件
- `src/core/data_exporter.cpp` - 数据导出器实现

### MATLAB 验证脚本
- `matlab/radar_data_validator.m` - 主验证脚本
- `matlab/loadjson.m` - JSON 文件读取函数（兼容旧版 MATLAB）

## 输出文件格式

### 1. config_params.json - 配置参数
JSON 格式，包含：
- `system_params` - 雷达系统参数（频率、PRF、带宽等）
- `target_config` - 目标模块配置
- `initial_targets` - 初始目标列表（位置、速度、RCS 等）

### 2. echo_iq_scan_N.dat - IQ 数据（二进制）
二进制文件格式（小端）：

| 偏移 | 大小 | 字段 | 类型 |
|------|------|------|------|
| 0 | 4 | magic | uint32 (0x4543484F) |
| 4 | 4 | version | uint32 |
| 8 | 4 | scan_index | int32 |
| 12 | 4 | cpi_count | uint32 |
| 16 | 4 | pulses_per_cpi | uint32 |
| 20 | 4 | samples_per_pulse | uint32 |
| 24 | 8 | reserved | uint64 |
| 32 | - | IQ 数据 | float32 (real, imag 交替) |

### 3. target_trajectory.csv - 目标轨迹
CSV 格式，列包括：
- `cpi_index`, `pulse_index`, `slow_time_s`
- `target_id`, `range_m`, `radial_velocity_mps`
- `azimuth_deg`, `elevation_deg`, `rcs_m2`

## 使用方法

### 1. 运行仿真
```bash
cd /path/to/Radar
./out/bin/radar
```
仿真完成后，数据将导出到 `output/` 目录。

### 2. 运行 MATLAB 验证
```matlab
cd matlab
radar_data_validator
```

### 3. 查看结果
脚本将生成一个包含 4 个子图的 figure：
- **左上**: 距离 - 多普勒图（Range-Doppler Map）
- **右上**: 距离向功率分布（Range Profile）
- **左下**: 多普勒频谱（某距离单元）
- **右下**: 距离 - 脉冲幅度图

红色 `+` 标记表示根据设定目标参数计算的期望位置。

## 验证流程

1. **读取配置**: 从 `config_params.json` 加载雷达参数和目标参数
2. **读取 IQ 数据**: 从 `.dat` 文件加载二进制 IQ 数据
3. **信号处理**:
   - 距离向：直接使用（假设已脉压）
   - 多普勒向：FFT + 加窗
4. **目标检测**: 简易门限检测（10 倍噪声基底）
5. **参数对比**: 将检测到的峰值位置与理论计算值对比

## 预期结果

如果仿真正确：
- RD 图上的能量峰值应与红色 `+` 标记位置吻合
- 静止目标：多普勒频率接近 0 Hz
- 运动目标：多普勒频率与径向速度成正比

## 文件格式说明

### DAT 文件 MATLAB 读取示例
```matlab
fid = fopen('echo_iq_scan_0.dat', 'rb');
magic = fread(fid, 1, 'uint32');
version = fread(fid, 1, 'uint32');
scan_idx = fread(fid, 1, 'int32');
cpi_count = fread(fid, 1, 'uint32');
pulses_per_cpi = fread(fid, 1, 'uint32');
samples_per_pulse = fread(fid, 1, 'uint32');

% 读取 IQ 数据
num_samples = cpi_count * pulses_per_cpi * samples_per_pulse;
iq_raw = fread(fid, num_samples * 2, 'float');
fclose(fid);

% 重组为复数
iq = iq_raw(1:2:end) + 1i * iq_raw(2:2:end);
iq_matrix = reshape(iq, [samples_per_pulse, pulses_per_cpi, cpi_count]);
```

## 注意事项

1. **数据文件大小**: IQ 数据为 float32 格式，每圈扫描约 100MB+
2. **MATLAB 版本**: R2016b 及以上版本直接使用 `jsondecode`，旧版本使用内置兼容函数
3. **内存需求**: 处理大数据时建议至少 8GB 内存
