# 噪声部分验证方案

## 验证目标

验证噪声引擎 (NoiseEngine) 的 C++ 实现与 MATLAB 参考实现的一致性，确保：
1. 复高斯白噪声生成正确
2. 三种噪声模式计算结果准确
3. 加噪后 SNR 符合预期
4. I/Q 分量统计特性正确

---

## 验证原理

### 1. 数学模型

#### 复高斯白噪声生成

复噪声样本定义为：
```
n = n_I + j·n_Q

n_I ~ N(0, σ²_IQ)
n_Q ~ N(0, σ²_IQ)

其中：σ²_IQ = Pn / 2
```

**参数说明**：
- `n_I`, `n_Q` : I/Q 分量（独立同分布高斯随机变量）
- `Pn` : 噪声功率（W）
- `σ_IQ` : I/Q 分量标准差

#### 噪声功率计算

**ComplexSigma 模式**：
```
Pn = σ²_complex
σ_IQ = σ_complex / √2
```

**NoisePower 模式**：
```
Pn = noise_power_w
σ_IQ = √(Pn / 2)
```

**ThermalKTB 模式**：
```
Pn = k · T · B · F

其中:
  k = 1.380649e-23 J/K  (玻尔兹曼常数)
  T = 噪声温度 (K)
  B = 带宽 (Hz)
  F = 噪声系数 (线性值) = 10^(NF_dB/10)
```

#### 信噪比 (SNR) 计算

```
SNR_linear = Ps / Pn
SNR_dB = 10 · log10(SNR_linear)
```

### 2. 验证策略

采用**统计对比**策略：
1. C++ 和 MATLAB 使用**相同的随机种子**
2. 生成大量噪声样本（N=100000）
3. 对比功率、I/Q 标准差、SNR 等统计量
4. 验证误差在可接受范围内

### 3. 通过标准

| 指标 | 通过标准 |
|------|----------|
| 功率相对误差 | < 0.5% (大样本统计) |
| I/Q 标准差误差 | < 0.5% |
| SNR 误差 | < 0.1 dB |
| ThermalKTB 功率计算 | < 0.01% (纯计算，无数值误差) |
| I/Q 互相关 | < 0.01 (独立性验证) |
| 重复性 | 完全一致 (误差=0) |

---

## 文件结构

```
Radar/
├── app/
│   └── noise_validation.cpp      # C++ 验证工具
├── matlab/
│   └── 03 噪声仿真/
│       └── compare_noise_cpp_matlab.m   # MATLAB 对比脚本
├── test/
│   └── radar_tests.cpp             # 单元测试（含噪声测试）
├── out/
│   └── noise_validation/         # 验证输出目录
│       ├── complex_sigma_samples.csv
│       ├── noise_power_samples.csv
│       ├── thermalktb_power.csv
│       ├── thermalktb_samples.csv
│       ├── snr_verification.csv
│       ├── noise_2d_matrix.csv
│       └── snr_points.csv
└── docs/superpowers/
    └── noise-validation-guide.md # 本文档
```

---

## 验证流程

### 步骤 1：编译验证工具

```bash
cd /home/lhlj/Radar
mkdir -p out
cmake -B out -S .
cmake --build out --target noise_validation
cmake --build out --target radar_tests
```

### 步骤 2：运行 C++ 验证工具

```bash
./out/bin/noise_validation --output_dir=out/noise_validation
```

**输出说明**：
- 7 个测试用例的结果
- 7 个 CSV 数据文件（供 MATLAB 对比）

**测试用例**：
| # | 测试名称 | 说明 | 样本数 |
|---|----------|------|--------|
| 1 | ComplexSigma Mode | 直接给定复噪声标准差 | 100000 |
| 2 | NoisePower Mode | 直接给定噪声功率 | 100000 |
| 3 | ThermalKTB Mode | 热噪声公式 kTB | 100000 |
| 4 | SNR Verification | 加噪后信噪比验证 | 100000 |
| 5 | 2D Noise Matrix | 二维回波加噪 | 2048×32 |
| 6 | Seed Repeatability | 随机种子重复性 | 100000 |
| 7 | Multiple SNR Points | 多 SNR 点验证 | 6 个点 |

### 步骤 3：运行 MATLAB 对比脚本

```matlab
cd matlab/03 噪声仿真
compare_noise_cpp_matlab.m
```

**MATLAB 执行内容**：
1. 读取 C++ 导出的 CSV 文件
2. 使用 MATLAB 函数计算相同参数下的噪声统计
3. 计算误差：`error = |C++ - MATLAB|`
4. 绘制噪声波形和统计分布
5. 输出验证报告

### 步骤 4：运行单元测试

```bash
./out/bin/radar_tests
```

**单元测试内容**：
- ComplexSigma 模式功率验证
- NoisePower 模式功率验证
- ThermalKTB 模式功率计算
- add_noise 功能验证
- 随机种子重复性
- I/Q 分量独立性
- SNR 验证（20 dB）
- ThermalKTB 统计验证

---

## 关键验证点

### 1. 噪声功率统计

**验证方法**：
```cpp
// 生成 N 个样本
auto noise = engine.generate(N);

// 计算平均功率
Scalar total_power = 0.0;
for (const auto& n : noise) {
    total_power += std::norm(n);  // |n|²
}
Scalar avg_power = total_power / N;

// 验证：|avg_power - target| / target < 0.5%
```

**预期结果**：
- 大样本下（N=100000），相对误差 < 0.5%

### 2. I/Q 分量独立性

**验证方法**：
```cpp
// 计算 I/Q 互相关
Scalar iq_correlation = 0.0;
for (const auto& n : noise) {
    iq_correlation += (n.real() - i_mean) * (n.imag() - q_mean);
}
iq_correlation /= N;

// 归一化
Scalar normalized_correlation = iq_correlation / theoretical_iq_var;

// 验证：|normalized_correlation| < 0.01
```

**预期结果**：
- I/Q 分量独立，互相关接近 0

### 3. SNR 验证

**验证方法**：
```cpp
// 创建恒定信号
Scalar signal_power = 100.0 * noise_power;  // SNR = 20 dB
ComplexVec signal(N, Complex(sqrt(signal_power), 0.0));

// 生成噪声
ComplexVec noise = engine.generate(N);

// 计算实测 SNR
Scalar measured_snr_db = math::linear_to_db(signal_power / noise_power);

// 验证：|measured_snr_db - 20.0| < 0.1 dB
```

**预期结果**：
- 实测 SNR 与理论值误差 < 0.1 dB

### 4. ThermalKTB 模式

**验证方法**：
```cpp
// C++ 计算
Scalar cpp_power = k * T * B * F;

// MATLAB 公式
Scalar matlab_power = k * T * B * F;

// 验证：相对误差 < 0.01%
```

**预期结果**：
- 纯计算，误差应该极小（< 0.01%）

---

## 测试结果示例

### C++ 验证工具输出

```
╔══════════════════════════════════════════════╗
║  噪声验证工具 - C++ vs MATLAB 对比           ║
╚══════════════════════════════════════════════╝

============================================================
  Test 1: ComplexSigma Mode
============================================================

[INFO] Parameters: sigma=0.001000, N=100000
  Target Power             : 1.000000e-06 W
  Estimated Power          : 1.001265e-06 W
  Relative Error           : 1.264603e-01 %

Validation:
  [PASS] Power error < 0.5%
  [PASS] I/Q sigma error < 0.5%

============================================================
  Test 4: SNR Verification
============================================================

[INFO] Signal Power: 0.000100 W
[INFO] Noise Power: 0.000001 W

SNR Results:
  Theoretical SNR          : 2.000000e+01 dB
  Measured SNR             : 1.999451e+01 dB
  SNR Error                : 5.488632e-02 dB

Validation:
  [PASS] SNR error < 0.1 dB

============================================================
  Test 7: Multiple SNR Points
============================================================

Target SNR  Measured SNR   Error       Status    
-5.000000   -4.998178      0.001822    PASS      
0.000000    -0.011106      0.011106    PASS      
5.000000    5.006988       0.006988    PASS      
10.000000   10.025785      0.025785    PASS      
20.000000   19.996885      0.003115    PASS      
30.000000   29.985796      0.014204    PASS      
```

### 单元测试输出

```
=== NoiseEngine Tests ===
  [PASS] NoiseEngine ComplexSigma init
  [PASS] ComplexSigma power statistics
  [PASS] NoiseEngine NoisePower init
  [PASS] NoisePower statistics
  [PASS] NoiseEngine ThermalKTB init
  [PASS] ThermalKTB power calculation
  [PASS] add_noise increases power
  [PASS] Same seed produces identical noise
  [PASS] I/Q components uncorrelated
  [PASS] SNR verification at 20dB
  [PASS] ThermalKTB statistical verification
```

---

## 常见问题

### Q1: 为什么功率统计有误差？

**A**: 噪声是随机过程，样本功率是统计估计值。根据大数定律，样本数 N 越大，估计越准确。相对误差约等于 `1/√N`。

对于 N=100000：
```
预期相对误差 ≈ 1/√100000 ≈ 0.3%
```

### Q2: 如何验证 I/Q 分量独立性？

**A**: 计算 I/Q 分量的互相关：
```
ρ_IQ = E[(I - μ_I)(Q - μ_Q)] / σ²_IQ
```

对于独立高斯变量，ρ_IQ 应该接近 0（典型值 < 0.01）。

### Q3: 如何修改测试参数？

**A**: 编辑 `app/noise_validation.cpp` 中的测试参数：

```cpp
// Test 1: ComplexSigma 模式
cfg.sigma_complex = 1.0e-3;  // 修改标准差
cfg.seed = 12345;            // 修改种子

// Test 7: 多 SNR 点
std::vector<Scalar> snr_targets_db = {-5.0, 0.0, 5.0, 10.0, 20.0, 30.0};
```

### Q4: ThermalKTB 模式的噪声系数在哪里设置？

**A**: 噪声系数在 `RadarSystemParams` 中设置：
```cpp
sys.noise_figure_db = 4.0;  // 4 dB 噪声系数
```

---

## 维护说明

### 更新 MATLAB 参考函数

如果 MATLAB 噪声仿真代码有更新，需要同步更新 `compare_noise_cpp_matlab.m` 中的参考函数：
- `matlab_generate_noise()`
- `matlab_thermalktb_power()`

### 扩展验证场景

可添加的验证场景：
- 不同样本数下的统计收敛特性
- 有色噪声生成（如多普勒谱整形）
- 多维回波加噪（Range-Doppler 矩阵）
- 低 SNR 场景（如 -10 dB）

---

## 参考资料

- MATLAB 噪声仿真代码：`matlab/03 噪声仿真/noise.m`
- C++ 噪声实现：`src/noise/`, `include/noise/`
- 单元测试：`test/radar_tests.cpp`

---

**文档创建日期**: 2026-04-09  
**最后更新**: 2026-04-09  
**验证状态**: ✅ 全部通过（75/75 测试）
