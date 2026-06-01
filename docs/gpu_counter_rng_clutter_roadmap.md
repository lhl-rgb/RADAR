# Counter-based GPU 海杂波实现可行性与优化路线

## 当前基线

当前实现已经采用以下核心路径：

```text
每 PRT:
  CPU: 波束方位/俯仰 + active 方位窗口参数
  GPU: 每个 thread 处理一个距离单元
       对 active 方位 patch 做 FIR
       FIR 输入由 Philox counter-based RNG 即时生成
       输出一个距离向 z_clutter[N_r]
```

当前 benchmark 结果，GT 1030，CUDA 架构 6.5：

```text
range_cells_actual:      2000
az_cells:                720
az_patch_step_deg:       0.500
active_gain_floor_db:    -40.000
fir_length:              13
prf_hz:                  1600
pulses_per_rotation:     9600
run_ms:                  6424.179
avg_us_per_prt:          669.185
realtime_ratio:          0.934x
```

1600 Hz PRF 对应单 PRT 周期：

```text
1 / 1600 = 625 us
```

因此当前基线在低端 GT 1030 上距离实时只差约 7%。这说明算法方向成立，后续应优先做低风险 GPU 端优化，而不是推翻结构。

## 可行性判断

该方案可行，原因如下：

1. **PRT 间状态非常小**

   Counter-based RNG 不维护每个 patch 的白噪历史，也不需要 delay line。理论上可复现状态只需要：

   ```text
   seed + pulse_index
   ```

2. **FIR 历史样本由 counter 重建**

   对同一个 `(seed, range_idx, az_idx, pulse_index, source_id)`，Philox 输出确定。FIR 的 `n-k` 历史白噪声可按需即时重算。

3. **每个距离单元天然并行**

   当前线程映射为：

   ```text
   thread -> range cell
   ```

   对 `N_r ~= 2000` 的场景，即使在低端 GPU 上也可以跑到接近实时。

4. **后续分布扩展路径清楚**

   当前 FIR 输出是复高斯杂波，幅度边缘为 Rayleigh。后续 Weibull、K、log-normal 等分布可在 FIR 输出后加 ZMNL：

   ```text
   u = FIR(white)
   u_dist = zmnl(u, distribution_type, extra_rng_if_needed)
   echo += gain * A * u_dist
   ```

   K 分布需要额外纹理随机数流，应使用不同 `source_id` 与白噪声流解耦。

## 当前实现与目标流程差异

目标流程中建议：

```text
A_static[N_r x N_az] 常驻 GPU
active_patch[] 每 PRT 上传
active patch 内含 theta_idx + gain_voltage
FIR h[k] 放 constant memory
一次 Philox 输出复用两个 tap
```

当前实现为了先跑通 GPU baseline，采用：

```text
range_amplitude[N_r] 常驻 GPU
range_elevation_deg[N_r] 常驻 GPU
active 方位窗口连续，用 center + offset 描述
天线 gain 在 kernel 内直接计算
FIR h[k] 在 device global memory
每个 tap 调一次 Philox
```

这些差异主要影响性能和扩展性，不影响当前 Rayleigh/复高斯杂波的基本正确性。

## 已确定要吸收的优化方向

### 1. active patch list + 天线方向图 LUT

当前 kernel 内每个距离单元、每个 active 方位都会调用一次 `expf` 计算天线方位/俯仰增益。

优化方向：

```text
CPU 或 GPU 侧按当前 beam 生成 active_patch[]
active_patch[i] = { az_idx, az_gain_voltage }
kernel 内直接读取 gain，不再重复算方位 expf
```

如果俯仰维仍与 range 有关，可以继续保留 `range_elevation_deg[N_r]`，或预计算 range 俯仰增益表。

预期收益：对 GT 1030 这类低端卡很可能明显，因为当前 `expf` 开销不小。

### 2. FIR 系数放 constant memory

FIR 系数是全 kernel 只读、小数组，适合放 CUDA constant memory：

```text
__constant__ cuFloatComplex c_fir[L_MAX]
```

预期收益：减少 global memory 读取开销，改动范围小。

### 3. Philox 一次输出复用两个 tap

当前每个 tap 调一次 Philox，只用 `rnd.x/rnd.y`。

Philox4x32_10 一次输出 4 个 `uint32`，可转成两个复高斯样本：

```text
(rnd.x, rnd.y) -> w0
(rnd.z, rnd.w) -> w1
```

然后：

```text
tap k     使用 w0
tap k + 1 使用 w1
```

注意需要明确 counter 语义，避免 `n-k` 与 `n-k-1` 的时间轴含义混乱。可以把 counter 的低位用于 pair index，pair 内第 0/1 个样本来自同一次 Philox。

预期收益：RNG 调用次数约减半。

### 4. active list 放 shared memory

当 active patch list 显式上传后，每个 block 可把 active list 搬到 shared memory：

```text
extern __shared__ ActivePatch s_active[];
```

每个 block 内所有 range thread 共享同一组 active patch。

预期收益：减少重复 global 读取。active 数量很小时收益有限，但结构更接近最终实现。

### 5. 输出留在 GPU / 异步拷贝

当前 benchmark 每个 PRT 都同步并拷贝回 host，真实系统如果后续脉压、MTD、CFAR 也在 GPU，应让 `z_clutter` 留在 device。

优化方向：

```text
generate clutter -> add target/noise -> pulse compression -> MTD/CFAR
```

只在显示、记录或网络输出时异步拷贝。

预期收益：减少同步与 PCIe 拷贝开销；当前 benchmark 的 `669 us` 包含同步/回拷成本。

### 6. 二维 A_static 图

当前先按“全图均匀海面杂波”实现，因此只保存：

```text
range_amplitude[N_r]
```

后续如果要接入不同方位/地物/海况 patch，应扩展为：

```text
A_static[N_r x N_az]
```

其中保存电压幅度：

```text
A_static = sqrt(P_static)
```

这样 kernel 内只需：

```text
scale = beam_gain_voltage * A_static[r, az]
```

### 7. ZMNL / 多分布扩展

当前分布：

```text
complex Gaussian -> Rayleigh amplitude
```

后续分布接口建议放在 FIR 后：

```text
u = FIR(white)
u = apply_distribution(u, distribution_type, params, seed, r, az, n)
```

其中：

- Weibull：可由 Rayleigh 幅度做幂变换
- Log-normal：可由额外 Gaussian 生成纹理
- K 分布：需要 Gamma 纹理，使用独立 `source_id`

### 8. 二维并行与归约

如果上述低风险优化后仍无法实时，再考虑改线程映射：

```text
thread/block -> (range, active_az)
```

先生成每个 `(r, active_az)` 的贡献，再对同一 range 做 reduce。

这是较大结构调整，当前不作为第一阶段优化。

## 随机数正确性检查点

当前 Philox 设计应满足：

```text
same(seed, r, az, n, source_id) -> bitwise same output
different source_id -> independent stream
different az/r/n -> different counter
```

需要补充测试：

1. 固定 `(seed, r, az, n, source)`，CPU/GPU 重复调用输出一致。
2. 同 seed 不同 pulse，输出不同。
3. 白复高斯统计：

   ```text
   E[real] ~= 0
   E[imag] ~= 0
   E[real^2 + imag^2] ~= 1
   ```

4. FIR 后单位功率保持：

   ```text
   sum |h[k]|^2 = 1
   ```

## 当前优先级

建议后续实现顺序：

1. 修正 benchmark checksum 使用科学计数法，避免小功率显示成 `0.000`。
2. active patch list + 天线方向图 gain 预计算，减少 kernel 内 `expf`。
3. FIR constant memory。
4. Philox 一次生成两个复高斯 tap。
5. 输出留 GPU / async copy。
6. A_static 二维幅度图。
7. ZMNL 多分布。
8. 二维并行与 reduce。

当前阶段不要一次性引入所有优化。每做一项，都用 `benchmark_clutter_gpu_rotation` 记录 `avg_us_per_prt` 和 `realtime_ratio`，避免优化方向不可量化。
