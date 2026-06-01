# GPU 加速杂波生成方案思考

## 目标

当前项目的杂波生成核心是基于距离-方位二维杂波图的 AR(1) 逐脉冲生成。后续如果要加速，合理方向不是“所有模块都上 GPU”，而是把杂波生成中计算密集、数据并行度高、适合长期流式运行的部分放到 GPU。

推荐方案可以概括为：

> GPU 常驻杂波图 + 波束窗口局部 AR 更新 + AR jump 跳步补偿 + pinned host 异步输出。

## 当前杂波生成模型

当前杂波状态可以理解为二维网格：

```text
state[range_bin, azimuth_cell]
sqrt_power[range_bin, azimuth_cell]
last_update[range_bin, azimuth_cell]
```

每个杂波单元维护一个复高斯 AR(1) 状态：

```text
x_k = ρ x_{k-1} + sqrt(1 - |ρ|²) w_k
```

当前波束指向某个方位时，只对波束附近的方位单元进行更新，然后按天线方向图加权叠加，输出距离向杂波回波：

```text
y_k(r) = Σ_m sqrt(P_c(r,m)) · G_v(θ_m - θ_ant) · x_k(r,m)
```

这类计算具有明显的数据并行特征：不同距离单元、不同方位单元之间大部分计算可以并行。

## AR jump 理论是否成立

AR jump 是成立的。

对 AR(1) 过程：

```text
x_k = ρ x_{k-1} + sqrt(1 - |ρ|²) w_k
```

如果某个杂波单元中间跳过 L 个脉冲没有逐拍更新，可以直接写为：

```text
x_k = ρ^L x_{k-L} + sqrt(1 - |ρ|^(2L)) z
```

其中 z 仍然是复高斯随机变量。

这意味着没有必要对未被波束照射的杂波单元每个脉冲都更新。它们可以保持旧状态，等再次进入波束窗口时，根据距离上次更新的脉冲间隔 L 做一次跳步更新。

AR jump 的核心价值不是让单个单元更新更快，而是避免每个脉冲更新整张 360° 杂波图。

## 按当前项目参数估算脉冲数量

当前默认 PRF 为：

```text
PRF = 1600 Hz
PRI = 1 / 1600 = 0.625 ms
```

常见机械扫描转速下，每圈脉冲数如下：

| 转速 | 角速度 | 一圈时间 | 每圈脉冲数 | 每度脉冲数 | 每个 PRT 方位步进 |
|---:|---:|---:|---:|---:|---:|
| 6 r/min | 36°/s | 10 s | 16000 | 44.44 | 0.0225° |
| 12 r/min | 72°/s | 5 s | 8000 | 22.22 | 0.045° |
| 24 r/min | 144°/s | 2.5 s | 4000 | 11.11 | 0.09° |

即使在 24 r/min 时，一圈也有约 4000 个脉冲；6 r/min 时一圈约 16000 个脉冲。因此杂波生成是典型的长时间连续流式计算。

## 每脉冲活跃计算规模

当前默认方位分辨率：

```text
angular_resolution = 0.25°
azimuth_cells = 360 / 0.25 = 1440
```

当前默认距离范围与采样率：

```text
min_range = 1000 m
max_range = 120000 m
fs = 20 MHz
range_bin_size ≈ 7.5 m
range_bins ≈ 15878
```

如果方位波束宽度为 5°，当前窗口取约 1.5 倍波束宽度，则：

```text
az_window = 1.5 * 5° = 7.5°
half_window ≈ 7.5 / 0.25 + 1 = 31
active_az_cells ≈ 63
```

每个脉冲活跃 cell 数约为：

```text
63 * 15878 ≈ 100 万 cell / pulse
```

在 PRF = 1600 Hz 时：

```text
100 万 * 1600 ≈ 16 亿 cell 更新/秒
```

如果实际波束宽度是 1.5°，则活跃方位单元约为 21 个：

```text
21 * 15878 ≈ 33 万 cell / pulse
33 万 * 1600 ≈ 5.3 亿 cell 更新/秒
```

因此虽然每个脉冲只更新局部方位窗口，但由于距离单元多、PRF 高、运行时间长，整体计算规模仍然适合 GPU 加速。

## 显存占用估算

全图 cell 数约为：

```text
15878 range bins * 1440 az cells ≈ 2286 万 cell
```

如果使用 float complex：

| 数据 | 每 cell 占用 | 总量 |
|---|---:|---:|
| state | 8 B | 约 183 MB |
| sqrt_power | 4 B | 约 91 MB |
| last_update | 4 B | 约 91 MB |
| power，如果运行期保留 | 4 B | 约 91 MB |

估计总显存：

```text
不保留 power: 约 365 MB
保留 power: 约 456 MB
```

这个显存量对现代 GPU 通常是可接受的。运行时可以只保留 sqrt_power，不保留 power。

## cudaHost 锁页内存的合理用途

cudaHost 锁页内存适合用于 CPU/GPU 之间的异步传输缓冲，而不是用于存放 GPU 高频访问的大状态矩阵。

推荐内存布局：

```text
GPU 显存常驻：
  state
  sqrt_power
  last_update
  antenna weight / azimuth table
  out_echo_device

CPU pinned host 内存：
  out_echo_host_pinned
  input/output ring buffer
```

调度流程：

```text
CPU/Qt/调度层
  ↓ 传入少量参数：beam_az, pulse_index
GPU kernel
  ↓ 局部更新当前波束附近杂波单元
GPU out_echo_device
  ↓ 如果后续处理也在 GPU，则继续留在 GPU
  ↓ 如果需要给 CPU/Qt/网络输出，则异步拷贝到 pinned host buffer
CPU pinned host buffer
```

不建议把 state、sqrt_power、last_update 放在 pinned host 内存中让 GPU 频繁跨 PCIe 访问。这样会被 PCIe 带宽和延迟限制，通常比常驻显存慢。

## GPU 加速的关键条件

GPU 方案是否值得，主要取决于以下条件：

1. **状态是否常驻 GPU**  
   如果每个脉冲都 CPU/GPU 来回搬整张图，会非常慢。必须让 state、sqrt_power、last_update 长期驻留显存。

2. **是否连续处理大量脉冲**  
   6/12/24 r/min 对应每圈 16000/8000/4000 脉冲，适合流式 GPU 计算。

3. **后续处理是否也能留在 GPU**  
   如果目标回波、噪声、脉压、FFT、检测等后续链路也逐步 GPU 化，则 out_echo 可以避免每脉冲拷回 CPU，加速收益更明显。

4. **kernel 粒度是否足够大**  
   如果每个脉冲只有很少距离单元或很窄窗口，kernel 启动开销可能吃掉收益；但按当前默认范围和 PRF，计算规模足够大。

## 推荐 GPU 计算流程

推荐把 GPU 杂波生成设计为以下几个阶段：

### 1. 初始化阶段

CPU 负责：

- 读取配置
- 计算系统派生参数
- 计算距离/方位网格大小
- 分配 GPU 显存
- 初始化或上传 sqrt_power
- 初始化 state
- 初始化 last_update

GPU 可负责：

- 批量初始化 state
- 批量计算 sqrt_power
- 预计算方位单元角度
- 预计算部分天线方向图查表

### 2. 每脉冲运行阶段

每个脉冲只传入少量参数：

```text
beam_az_deg
pulse_index
```

GPU kernel 执行：

1. 计算当前波束覆盖的方位窗口
2. 对窗口内 `(range_bin, azimuth_cell)` 并行更新 AR 状态
3. 对 steps > 1 的单元执行 AR jump
4. 计算或查表获得天线方向图权重
5. 计算 `sqrt_power * antenna_weight * state`
6. 对同一距离单元的多个方位贡献进行归约
7. 输出 `out_echo[range_bin]`

### 3. 输出阶段

如果后续链路在 GPU：

```text
out_echo_device -> target/noise/waveform/FFT/detection kernels
```

如果需要给 CPU：

```text
cudaMemcpyAsync(out_echo_host_pinned, out_echo_device)
```

并使用 double buffer 或 ring buffer 避免同步等待。

## 实现上的注意点

### 1. 数据布局需要调整

当前 CPU 版使用：

```text
index = range_bin * num_az_cells + az_cell
```

如果固定 az_cell 遍历 range_bin，内存访问是跨 stride 的，不是连续访问。GPU 版本要注意 memory coalescing。

可以考虑两种布局：

```text
layout A: state[range][az]
  优点：同一 range 下多个 az 连续，方便对方位窗口归约

layout B: state[az][range]
  优点：同一 az 下所有 range 连续，方便一个方位单元批量更新
```

如果 GPU kernel 以 `(az, range)` 二维线程块组织，建议根据归约方式选择布局，避免非合并访存。

### 2. AR jump 参数可按方位单元预计算

同一个方位单元下，多数 range_bin 的 `steps` 是相同的。因此 `ρ^L` 和 `sqrt(1-|ρ|^(2L))` 不应在每个 range cell 里重复计算。

推荐先按方位单元计算：

```text
rho_pow[az]
noise_scale[az]
```

然后所有 range cell 复用。

### 3. 复高斯归一化要统一

如果定义复高斯 `w = (n_re + j n_im) / sqrt(2)`，其中 `n_re,n_im ~ N(0,1)`，则：

```text
E[|w|²] = 1
```

初始化状态和每次 AR 注入噪声应该保持同样归一化。否则杂波功率可能偏差约 3 dB。

### 4. 随机数建议使用 counter/hash 风格

当前项目已有 hash 到复高斯的思路，这对 GPU 友好，因为不需要为每个 cell 保存庞大的 RNG 状态。

推荐随机数输入至少包含：

```text
base_seed
pulse_index
azimuth_cell
range_bin
```

这样可以保证不同 cell、不同脉冲的随机激励可复现且相互区分。

### 5. 不要每脉冲拷回大状态

每脉冲只应传输：

```text
输入：beam_az_deg, pulse_index 等少量参数
输出：out_echo，或者继续留在 GPU
```

禁止每脉冲在 CPU/GPU 之间传输 state、sqrt_power、last_update。

## 可以作为专利点的表述

可以考虑将技术方案表述为：

> 一种基于波束扫描选择更新的雷达杂波并行生成方法。该方法将距离-方位二维杂波状态图常驻 GPU 显存；根据机械扫描雷达当前波束方位确定局部方位窗口；仅对窗口内杂波单元执行 AR 状态更新；对未连续照射的杂波单元根据上次更新时间执行跳步 AR 更新；并在 GPU 端完成方向图加权、杂波功率调制和距离向回波归约输出。

可强调的创新点：

1. **扫描同步的局部杂波更新**  
   每个脉冲只更新当前波束覆盖区域，避免全图逐拍更新。

2. **AR jump 跳步补偿机制**  
   对离开波束窗口的单元不逐拍更新，再次进入窗口时用闭式 AR 跳步公式补偿。

3. **GPU 常驻二维杂波状态图**  
   state、sqrt_power、last_update 长期驻留显存，减少 CPU/GPU 数据搬运。

4. **counter/hash 式复高斯激励生成**  
   避免维护大规模 per-cell RNG 状态，适合 GPU 并行。

5. **面向距离-方位网格的并行归约输出**  
   对当前波束窗口内多个方位单元的贡献进行并行加权叠加，生成距离向杂波回波。

## 结论

对于当前项目的杂波生成，GPU 加速是合理的，但前提不是“能上 GPU 的都上 GPU”，而是：

```text
计算密集部分上 GPU
状态常驻显存
按波束局部更新
用 AR jump 避免全图逐拍更新
用 pinned host 做异步 I/O 缓冲
尽量让后续处理继续留在 GPU
```

在长时间连续运行、PRF 较高、距离单元较多的条件下，该方案有明确加速空间，也具备较清晰的技术方案表达价值。
