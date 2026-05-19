# 海杂波验证指南

## 1. 概述

本文档描述海杂波模块的 C++ 实现与 MATLAB 仿真的对比验证方法和流程。

### 1.1 验证目标

- 验证 Morchin 海杂波σ⁰模型的计算正确性
- 验证 K 分布杂波序列的统计特性
- 验证高斯多普勒谱的正确性
- 验证杂波功率计算
- 验证随机种子的可重复性

### 1.2 验证工具

- **C++ 验证工具**: `app/clutter_validation.cpp`
- **MATLAB 对比脚本**: `matlab/05 杂波仿真/compare_clutter_cpp_matlab.m`

## 2. 数学模型

### 2.1 Morchin 海杂波模型

海杂波的归一化雷达散射截面积（σ⁰）使用论文式 Morchin 模型计算：

$$
\sigma_0=
\frac{4\times10^{-7}\cdot 10^{0.6(ss+1)}\cdot \sigma_0^c\cdot \sin\varphi}{\lambda}
\;+\;
\cot^2 \beta \cdot \exp\left(-\frac{\tan^2(\pi/2-\varphi)}{\tan^2\beta}\right)
$$

其中：

$$
\beta=\frac{2.44(ss+1)^{1.08}}{57.29},\qquad
h_e=0.025+0.046ss^{1.72},\qquad
\varphi_c=\arcsin\left(\frac{\lambda}{4\pi h_e}\right)
$$

$$
\sigma_0^c=
\begin{cases}
(\varphi/\varphi_c)^{1.9}, & \varphi<\varphi_c \\
1, & \varphi\ge \varphi_c
\end{cases}
$$

其中：
- $\varphi$ 为掠射角（弧度）
- $\lambda$ 为雷达波长
- $ss$ 为海情等级
- $\beta$ 为经验常数
- $h_e$ 为海面粗糙度高度（m）
- $\varphi_c$ 为临界角
- $\sigma_0^c$ 为论文中的分段理论参数

### 2.2 K 分布模型

海杂波的幅度统计特性使用 K 分布描述，通过 SIRP（Spherically Invariant Random Process）方法生成：

$$x(t) = \sqrt{\tau(t)} \cdot g(t)$$

其中：
- $g(t)$ 是复高斯白噪声
- $\tau(t)$ 是 Gamma 分布的纹理分量，$\tau \sim \Gamma(\nu, 1/\nu)$
- $\nu$ 是 K 分布形状参数，控制杂波的"尖峰"程度

### 2.3 高斯多普勒谱

杂波的多普勒谱特性使用高斯模型：

$$S(f) = \exp\left(-\frac{1}{2}\left(\frac{f - f_d}{\sigma_d}\right)^2\right)$$

其中：
- $f_d$ 是多普勒中心频率
- $\sigma_d$ 是多普勒展宽

### 2.4 雷达方程（杂波功率计算）

杂波单元的平均接收功率：

$$P_r = \frac{P_t G^2 \lambda^2 \sigma^0 A_{\text{cell}}}{(4\pi)^3 R^4 L_{\text{sys}}}$$

其中：
- $P_t$ 是发射功率
- $G$ 是天线增益
- $\lambda$ 是波长
- $\sigma^0$ 是归一化 RCS
- $A_{\text{cell}}$ 是杂波单元面积
- $R$ 是斜距
- $L_{\text{sys}}$ 是系统损耗

## 3. 验证步骤

### 3.1 编译 C++ 验证工具

```bash
cd /home/lhlj/Radar
cmake --build out --target clutter_validation
```

### 3.2 运行 C++ 验证工具

```bash
./out/bin/clutter_validation --output_dir=out/clutter_validation
```

输出文件：
1. `morchin_sigma0.csv` - Morchin σ⁰随掠射角变化
2. `k_distribution_samples.csv` - K 分布杂波样本
3. `doppler_spectrum.csv` - 高斯多普勒谱
4. 杂波功率计算结果（日志输出）

### 3.3 运行 MATLAB 对比脚本

在 MATLAB 环境中执行：

```matlab
cd matlab/05 杂波仿真
compare_clutter_cpp_matlab.m
```

MATLAB 脚本将：
1. 读取 C++ 导出的 CSV 数据
2. 使用 MATLAB 计算相同参数下的理论值
3. 对比并计算误差
4. 生成对比图表和误差分析

## 4. 验证内容

### Test 1: Morchin σ⁰ 模型验证

**验证内容**：对比不同掠射角下的 σ⁰ 计算值

**参数**：
- 载频：10 GHz
- 掠射角范围：0.1° - 30°

**通过标准**：σ⁰ 误差 < 0.1 dB

### Test 2: K 分布统计特性验证

**验证内容**：验证 K 分布杂波序列的功率归一化和统计特性

**参数**：
- 形状参数 ν = 0.8
- 样本数 N = 100000

**通过标准**：
- 功率归一化误差 < 1%
- 幅度统计特性与 MATLAB 一致

### Test 3: 高斯多普勒谱验证

**验证内容**：验证多普勒谱形状和参数

**参数**：
- 多普勒中心 $f_d$ = 100 Hz
- 多普勒展宽 $\sigma_d$ = 20 Hz
- PRF = 1000 Hz

**通过标准**：
- 多普勒中心在 Nyquist 范围内
- 谱形与理论高斯谱一致

### Test 4: 杂波功率计算验证

**验证内容**：验证完整杂波生成链路的功率计算

**参数**：
- 雷达：fc=10GHz, fs=40MHz, PRF=1kHz, Pt=1kW
- 杂波：距离范围 [1km, 50km]
- 天线：ULA N=16, G_peak=30dB

**通过标准**：CPI 生成成功，功率量级正确

### Test 5: 随机种子重复性验证

**验证内容**：验证相同种子产生相同的杂波序列

**参数**：
- 种子 = 42
- 样本数 N = 100000

**通过标准**：相同种子产生的序列完全相同

## 5. 故障排除

### 问题 1: K 分布形状参数估计误差大

**原因**：当 ν<1 时，矩估计方法的方差较大，这是统计学上的固有限制。

**解决方案**：使用 MATLAB 进行更精确的最大似然估计验证。

### 问题 2: 多普勒谱验证失败

**原因**：FFT 点数不足或频率轴定义不一致。

**解决方案**：检查 C++ 和 MATLAB 中的频率轴定义是否一致。

### 问题 3: 杂波功率量级异常

**原因**：雷达方程参数单位错误或归一化因子遗漏。

**解决方案**：逐一检查每个参数的单位和量纲。

## 6. 输出文件说明

| 文件名 | 内容 | 用途 |
|--------|------|------|
| morchin_sigma0.csv | σ⁰ vs 掠射角 | 验证论文式 Morchin 模型 |
| k_distribution_samples.csv | K 分布样本 | 验证统计特性 |
| doppler_spectrum.csv | 多普勒谱数据 | 验证频谱特性 |
| sigma0_comparison.csv | σ⁰对比结果 | MATLAB 生成 |
| k_statistics_comparison.csv | K 分布统计对比 | MATLAB 生成 |
| doppler_comparison.csv | 多普勒谱对比 | MATLAB 生成 |

## 7. 参考文献

1. Morchin, W.C., "Airborne Early Warning Radar", 1998.
2. Ward, K.D., "Compound Representation of High Resolution Sea Clutter", Electronics Letters, 1981.
3. Conte, E. and Longo, M., "Characterisation of Radar Clutter as a Spherically Invariant Random Process", IEE Proceedings F, 1987.

## 8. 更新记录

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-04-10 | 1.0 | 初始版本 |
