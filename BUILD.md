# Radar 构建与海杂波测试命令

本文档只保留当前常用命令，方便直接复制运行。以下命令默认在仓库根目录执行：

```bash
cd ~/Radar
```

## 1. 构建

新 Ubuntu 工控机首次配置依赖见：

```bash
scripts/setup_ubuntu.sh --help
```

详细说明见 `SETUP_UBUNTU.md`。

### 1.1 配置 CMake

```bash
cmake -S . -B out/build
```

### 1.2 编译全部目标

```bash
cmake --build out/build -j
```

### 1.3 运行单元测试

```bash
./out/bin/radar_tests
ctest --test-dir out/build --output-on-failure
```

### 1.4 常用可执行文件

```text
./out/bin/export_clutter_patch_trace       # 单个 patch 的 IQ/PSD 导出
./out/bin/export_clutter_rotation_ppi      # 一圈 PPI 杂波导出
./out/bin/benchmark_clutter_gpu_rotation   # 一圈生成速度测试
./out/bin/benchmark_clutter_gpu_2d         # 1D / 2D / 2D-fast 对比测试
./out/bin/benchmark_clutter_gpu_stream     # 2D-fast 同步 / 异步双缓冲对比测试
```

## 2. 分布编号与通用参数

分布编号：

```text
0 Rayleigh
1 Weibull
2 LogNormal
3 K
```

本文档里的默认雷达/杂波测试参数：

```text
range_cells              2000
sample_count             20000
az_patch_step_deg        0.5
active_gain_floor_db     -40
fir_length               13
sigma_f_hz               40
doppler_center_hz        0
prf_hz                   1600
rotation_rate_dps        60
seed                     2026
iq_range_idx_1based      1000
```

## 3. 单个 Patch 导出与 MATLAB 验证

单 patch 导出会生成：

```text
<prefix>_trace.txt
<prefix>_psd.txt
```

MATLAB 验证函数：

```matlab
addpath('test')
validate_clutter_patch_trace('<prefix>_trace.txt', '<prefix>_psd.txt')
```

### 3.1 Rayleigh 单 patch

```bash
./out/bin/export_clutter_patch_trace \
  20000 1000 0 13 40 0 1600 2026 out/clutter_patch_rayleigh \
  0
```

```matlab
addpath('test')
validate_clutter_patch_trace('out/clutter_patch_rayleigh_trace.txt', ...
                             'out/clutter_patch_rayleigh_psd.txt')
```

### 3.2 Weibull 单 patch

```bash
./out/bin/export_clutter_patch_trace \
  20000 1000 0 13 40 0 1600 2026 out/clutter_patch_weibull \
  1 1.5 1.2
```

```matlab
addpath('test')
validate_clutter_patch_trace('out/clutter_patch_weibull_trace.txt', ...
                             'out/clutter_patch_weibull_psd.txt')
```

### 3.3 LogNormal 单 patch

```bash
./out/bin/export_clutter_patch_trace \
  20000 1000 0 13 40 0 1600 2026 out/clutter_patch_lognormal \
  2 1.5 1.2 -0.5 0.5
```

```matlab
addpath('test')
validate_clutter_patch_trace('out/clutter_patch_lognormal_trace.txt', ...
                             'out/clutter_patch_lognormal_psd.txt')
```

### 3.4 K 分布单 patch

```bash
./out/bin/export_clutter_patch_trace \
  20000 1000 0 13 40 0 1600 2026 out/clutter_patch_k \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

```matlab
addpath('test')
validate_clutter_patch_trace('out/clutter_patch_k_trace.txt', ...
                             'out/clutter_patch_k_psd.txt')
```

## 4. 一圈杂波 PPI 导出与 MATLAB 显示

一圈导出会生成：

```text
<prefix>_power.bin       # float32, range x pulse
<prefix>_meta.txt        # 元数据
<prefix>_iq.txt          # 指定距离单元的一圈 IQ
```

MATLAB 显示函数：

```matlab
addpath('test')
show_clutter_rotation_ppi('<prefix>', 50, 1000)
```

### 4.1 Rayleigh 一圈 PPI

```bash
./out/bin/export_clutter_rotation_ppi \
  2000 0.5 -40 13 1600 60 out/clutter_rotation_rayleigh 1000 \
  0
```

```matlab
addpath('test')
show_clutter_rotation_ppi('out/clutter_rotation_rayleigh', 50, 1000)
```

### 4.2 Weibull 一圈 PPI

```bash
./out/bin/export_clutter_rotation_ppi \
  2000 0.5 -40 13 1600 60 out/clutter_rotation_weibull 1000 \
  1 1.5 1.2
```

```matlab
addpath('test')
show_clutter_rotation_ppi('out/clutter_rotation_weibull', 50, 1000)
```

### 4.3 LogNormal 一圈 PPI

```bash
./out/bin/export_clutter_rotation_ppi \
  2000 0.5 -40 13 1600 60 out/clutter_rotation_lognormal 1000 \
  2 1.5 1.2 -0.5 0.5
```

```matlab
addpath('test')
show_clutter_rotation_ppi('out/clutter_rotation_lognormal', 50, 1000)
```

### 4.4 K 分布一圈 PPI

```bash
./out/bin/export_clutter_rotation_ppi \
  2000 0.5 -40 13 1600 60 out/clutter_rotation_k 1000 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

```matlab
addpath('test')
show_clutter_rotation_ppi('out/clutter_rotation_k', 50, 1000)
```

## 5. 实时性测试

实时性测试只输出耗时和实时倍率，不导出图像文件。

参数第 2 个是测试圈数 `rotations`。建议：

```text
1 圈：快速功能测试
10 圈：正式性能统计，论文/汇报推荐使用
30 圈：长时间稳定性测试
```

关注输出字段：

```text
avg_us_per_prt
generated_prt_per_sec
realtime_ratio
avg_kernel_sync_us
avg_gpu_call_total_us
```

### 5.0 1D / 2D / 2D-fast 对比测试

各路径的线程映射、优化动机、预期收益和适用条件见
[`docs/gpu_clutter_parallel_paths_comparison.md`](docs/gpu_clutter_parallel_paths_comparison.md)。

该测试同时输出：

```text
1D range-thread path
2D range_patch path
2D fast range_patch path
speedup_2d_vs_1d
speedup_2d_fast_vs_1d
speedup_2d_fast_vs_2d
```

K 分布、1 圈快速测试：

```bash
./out/bin/benchmark_clutter_gpu_2d \
  2000 1 0.5 -40 25 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

K 分布、较大 active patch 压力测试：

```bash
./out/bin/benchmark_clutter_gpu_2d \
  2000 1 0.2 -40 37 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

### 5.0.1 2D-fast 异步双缓冲测试

该测试用于比较：

```text
sync 2D-fast path
stream 2D-fast double-buffer path
speedup_stream_vs_sync
```

说明：K 分布 texture 是有状态 AR(1)，因此 stream 版本保持相邻 PRT 的 kernel 顺序，
主要重叠 active patch 上传、回波下载、CPU 侧构建 active patch 和主机转换开销。

K 分布、1 圈快速测试：

```bash
./out/bin/benchmark_clutter_gpu_stream \
  2000 1 0.5 -40 25 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

K 分布、较大 active patch 压力测试：

```bash
./out/bin/benchmark_clutter_gpu_stream \
  2000 1 0.2 -40 37 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

### 5.1 Rayleigh 实时性

快速测试 1 圈：

```bash
./out/bin/benchmark_clutter_gpu_rotation \
  2000 1 0.5 -40 13 1600 60 \
  0
```

正式统计 10 圈：

```bash
./out/bin/benchmark_clutter_gpu_rotation \
  2000 10 0.5 -40 13 1600 60 \
  0
```

### 5.2 Weibull 实时性

快速测试 1 圈：

```bash
./out/bin/benchmark_clutter_gpu_rotation \
  2000 1 0.5 -40 13 1600 60 \
  1 1.5 1.2
```

正式统计 10 圈：

```bash
./out/bin/benchmark_clutter_gpu_rotation \
  2000 10 0.5 -40 13 1600 60 \
  1 1.5 1.2
```

### 5.3 LogNormal 实时性

快速测试 1 圈：

```bash
./out/bin/benchmark_clutter_gpu_rotation \
  2000 1 0.5 -40 13 1600 60 \
  2 1.5 1.2 -0.5 0.5
```

正式统计 10 圈：

```bash
./out/bin/benchmark_clutter_gpu_rotation \
  2000 10 0.5 -40 13 1600 60 \
  2 1.5 1.2 -0.5 0.5
```

### 5.4 K 分布实时性

快速测试 1 圈：

```bash
./out/bin/benchmark_clutter_gpu_rotation \
  2000 1 0.5 -40 13 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

正式统计 10 圈：

```bash
./out/bin/benchmark_clutter_gpu_rotation \
  2000 10 0.5 -40 13 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

长时间稳定性测试 30 圈：

```bash
./out/bin/benchmark_clutter_gpu_rotation \
  2000 30 0.5 -40 13 1600 60 \
  3 1.5 1.2 -0.5 0.5 1.0 2.0 4096
```

## 6. 参数扫描示例

### 6.1 FIR 长度对实时性的影响

```bash
./out/bin/benchmark_clutter_gpu_rotation 2000 1 0.5 -40 13 1600 60 0
./out/bin/benchmark_clutter_gpu_rotation 2000 1 0.5 -40 25 1600 60 0
./out/bin/benchmark_clutter_gpu_rotation 2000 1 0.5 -40 64 1600 60 0
```

### 6.2 方位 patch 步长对实时性的影响

```bash
./out/bin/benchmark_clutter_gpu_rotation 2000 1 1.0  -40 13 1600 60 0
./out/bin/benchmark_clutter_gpu_rotation 2000 1 0.5  -40 13 1600 60 0
./out/bin/benchmark_clutter_gpu_rotation 2000 1 0.25 -40 13 1600 60 0
```

### 6.3 距离单元数对实时性的影响

```bash
./out/bin/benchmark_clutter_gpu_rotation 1000 1 0.5 -40 13 1600 60 0
./out/bin/benchmark_clutter_gpu_rotation 2000 1 0.5 -40 13 1600 60 0
./out/bin/benchmark_clutter_gpu_rotation 4000 1 0.5 -40 13 1600 60 0
```

## 7. 常见问题

### 7.1 CUDA 运行环境不可用

如果输出类似：

```text
cudaGetDeviceCount: CUDA driver version is insufficient for CUDA runtime version
```

说明当前 CUDA driver/runtime 不匹配，程序会跳过 GPU 测试。需要在 CUDA 驱动正常的机器上运行，例如 Jetson AGX Orin 或驱动版本匹配的 NVIDIA GPU 主机。

### 7.2 benchmark 和 export 的区别

```text
benchmark_clutter_gpu_rotation:
  只测试生成速度，不写文件。

export_clutter_patch_trace:
  导出单个 patch 的慢时间 IQ 和 PSD。

export_clutter_rotation_ppi:
  导出完整一圈 PPI 数据和指定距离单元 IQ。
```
