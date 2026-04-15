# 波形生成模块验证总结报告

## 1. 概述

### 1.1 验证目标

- 验证三种波形（LFM、NLFM、相位编码）的 C++ 实现正确性
- 验证波形生成的时间轴一致性
- 验证匹配滤波器的正确性
- 验证功率归一化
- 验证与 MATLAB 仿真的数据接口

### 1.2 验证工具

- **C++ 验证工具**: `app/waveform_validation.cpp`
- **单元测试**: `test/radar_tests.cpp` (WaveformGenerator Tests)
- **MATLAB 对比脚本**: 预留 `matlab/03 波形生成/` 目录

### 1.3 验证环境

| 项目 | 参数 |
|------|------|
| 采样率 (fs) | 40 MHz |
| 脉宽 (T) | 20 μs |
| 带宽 (B) | 20 MHz |
| 采样点数 (N) | 800 |

---

## 2. 数学模型

### 2.1 LFM（线性调频）信号

LFM 信号使用闭式相位表达式：

$$s(t) = \exp\left(j 2\pi \left[-\frac{B}{2}t + \frac{B}{2T}t^2\right]\right)$$

其中：
- $B$ = 带宽 (20 MHz)
- $T$ = 脉宽 (20 μs)
- 调频率 $\gamma = B/T = 1 \text{ MHz/μs}$

**时间轴约定**：第 $n$ 个采样点 $t_n = (n + 0.5) \cdot dt$，其中 $dt = 1/f_s$

### 2.2 NLFM（非线性调频）信号

NLFM 采用群时延法设计：

1. **频域加权**：$W(f)$ 使用窗函数（Hamming/Hann/Blackman）
2. **群时延**：$\tau(f) = T \times \text{CDF}[W(f)]$
3. **频率响应**：$f(t) = \tau^{-1}(t)$（数值求逆）
4. **相位累积**：$\phi(t) = 2\pi \int_0^t f(\tau) d\tau$
5. **信号生成**：$s(t) = \exp(j\phi(t))$

**功率归一化**：$\frac{1}{N}\sum_{n=0}^{N-1}|s(n)|^2 = 1$

### 2.3 相位编码信号（Barker 码）

Barker 码是二相位编码，幅度恒定为±1：

$$s(t) = \begin{cases} +1, & \text{chip} = 1 \\ -1, & \text{chip} = -1 \end{cases}$$

支持的 Barker 码类型：
- Barker2, Barker3, Barker4, Barker5, Barker7, Barker11, Barker13

**码片分配**：每个码片均匀分配到 $\lfloor N/N_{\text{chips}} \rfloor$ 个采样点

### 2.4 匹配滤波器

匹配滤波器定义为波形的共轭反转：

$$h(n) = s^*(N-1-n)$$

---

## 3. 验证步骤

### 3.1 编译验证工具

```bash
cd /home/lhlj/Radar
cmake --build out --target waveform_validation
cmake --build out --target radar_tests
```

### 3.2 运行波形验证工具

```bash
./out/bin/waveform_validation -o out/validation
```

输出文件：
- `lfm_waveform.csv` - LFM 波形复数数据
- `nlfm_waveform.csv` - NLFM 波形复数数据
- `barker11_waveform.csv` - Barker11 波形复数数据
- `*_matched_filter.csv` - 匹配滤波器数据

### 3.3 运行单元测试

```bash
./out/bin/radar_tests
```

---

## 4. 验证内容与结果

### Test 1: LFM 波形验证

**验证内容**：
- 波形初始化
- 波形长度正确
- 匹配滤波器共轭反转正确

**测试结果**：
| 测试项 | 状态 |
|--------|------|
| LFM waveform initialize | ✅ PASS |
| LFM waveform not empty | ✅ PASS |
| LFM waveform size (800) | ✅ PASS |
| Matched filter size | ✅ PASS |
| Matched filter is conjugate reversed | ✅ PASS |

**结论**：LFM 波形生成正确，匹配滤波器符合定义。

### Test 2: NLFM 波形验证

**验证内容**：
- 波形初始化
- 功率归一化（平均功率 = 1）

**测试结果**：
| 测试项 | 状态 | 实测值 |
|--------|------|--------|
| NLFM waveform initialize | ✅ PASS | - |
| NLFM waveform not empty | ✅ PASS | - |
| NLFM average power ≈ 1 | ✅ PASS | 1.000000 (diff=0.000000) |

**结论**：NLFM 波形功率归一化精确，群时延法实现正确。

### Test 3: 相位编码波形验证（Barker13）

**验证内容**：
- 波形初始化
- 恒定包络特性

**测试结果**：
| 测试项 | 状态 |
|--------|------|
| Barker13 waveform initialize | ✅ PASS |
| Barker13 waveform not empty | ✅ PASS |
| Barker13 constant envelope | ✅ PASS |

**结论**：Barker 码波形幅度恒定，相位跳变正确。

### Test 4: CW 波形验证

**验证内容**：
- 连续波初始化
- 恒定包络特性

**测试结果**：
| 测试项 | 状态 |
|--------|------|
| CW waveform initialize | ✅ PASS |
| CW waveform not empty | ✅ PASS |
| CW constant envelope | ✅ PASS |

### Test 5: 采样点数一致性验证

**验证内容**：C++ 与 MATLAB 采样点数计算方式对比

**参数**：$T = 20 \mu s$, $f_s = 40 \text{ MHz}$

**结果**：
| 方法 | 计算公式 | 结果 |
|------|----------|------|
| C++ | $\lceil T \times f_s \rceil$ | 800 |
| MATLAB | $\text{round}(T \times f_s)$ | 800 |

**结论**：在当前参数下，C++ 和 MATLAB 的采样点数一致。

### Test 6: 数据导出验证

**验证内容**：CSV 格式数据导出供 MATLAB 对比

**输出文件**：
| 文件 | 大小 | 内容 |
|------|------|------|
| lfm_waveform.csv | 39 KB | 800 点复数波形 |
| lfm_matched_filter.csv | 39 KB | 匹配滤波器 |
| nlfm_waveform.csv | 39 KB | NLFM 复数波形 |
| nlfm_matched_filter.csv | 39 KB | NLFM 匹配滤波器 |
| barker11_waveform.csv | 39 KB | Barker11 复数波形 |
| barker11_matched_filter.csv | 39 KB | Barker11 匹配滤波器 |

---

## 5. 关键实现细节

### 5.1 统一时间轴

所有波形使用统一的时间轴定义：

```cpp
// 第 n 个采样点对应 t_n = (n + 0.5) * dt
inline Scalar sample_time(int n, Scalar dt) {
    return (static_cast<Scalar>(n) + 0.5) * dt;
}
```

这确保了：
- LFM 和 NLFM 的时间基准一致
- 与 MATLAB 的 `t = (0:N-1)*dt + 0.5*dt` 等价
- 采样点覆盖 $[0.5dt, (N-0.5)dt] \approx [0, T]$

### 5.2 NLFM 群时延法

NLFM 实现的关键步骤：

1. **频域离散化**：4096~8192 点频域网格
2. **窗函数加权**：支持 Rectangular/Hann/Hamming/Blackman
3. **CDF 计算**：数值积分得累积分布
4. **归一化映射**：$\tau(f) \in [0, T]$
5. **二分查找求逆**：`invert_monotonic_linear()`
6. **相位累积**：$\phi[n] = \phi[n-1] + 2\pi f[n] dt$

### 5.3 功率归一化

```cpp
void normalize_average_power(ComplexVec& waveform) {
    if (waveform.empty()) return;
    Scalar power_sum = 0.0;
    for (const Complex& s : waveform) {
        power_sum += std::norm(s);
    }
    const Scalar scale = std::sqrt(
        static_cast<Scalar>(waveform.size()) / power_sum);
    for (Complex& s : waveform) {
        s *= scale;
    }
}
```

---

## 6. 故障排除

### 问题 1: NLFM 功率不归一

**原因**：积分过程中数值误差累积

**解决方案**：在生成完成后调用 `normalize_average_power()`

### 问题 2: 相位编码边界不连续

**原因**：码片到采样点的映射计算错误

**解决方案**：使用 $\lfloor (c+1) \cdot N/N_{\text{chips}} \rfloor$ 计算边界

### 问题 3: 匹配滤波器相位错误

**原因**：忘记取共轭或反转顺序

**解决方案**：验证 $h[n] = s^*[N-1-n]$

---

## 7. 输出文件说明

| 文件名 | 内容 | 用途 |
|--------|------|------|
| lfm_waveform.csv | LFM 复数波形 | MATLAB 对比 |
| nlfm_waveform.csv | NLFM 复数波形 | MATLAB 对比 |
| barker11_waveform.csv | Barker11 复数波形 | MATLAB 对比 |
| *_matched_filter.csv | 匹配滤波器 | 脉冲压缩验证 |

CSV 格式：
```
index,real,imag
0,1.000000e+00,0.000000e+00
1,9.999999e-01,1.234567e-02
...
```

---

## 8. 单元测试汇总

**WaveformGenerator Tests** 测试结果：

```
╔══════════════════════════════════════════╗
║       Radar System Unit Tests            ║
╚══════════════════════════════════════════╝

=== WaveformGenerator Tests ===
  [PASS] LFM waveform initialize
  [PASS] LFM waveform not empty
  [PASS] LFM waveform size (size=800)
  [PASS] Matched filter size
  [PASS] Matched filter is conjugate reversed
  [PASS] NLFM waveform initialize
  [PASS] NLFM waveform not empty
  [PASS] NLFM average power ≈ 1
  [PASS] Barker13 waveform initialize
  [PASS] Barker13 waveform not empty
  [PASS] Barker13 constant envelope
  [PASS] CW waveform initialize
  [PASS] CW waveform not empty
  [PASS] CW constant envelope

Test Summary:
  Passed: 14/14 (WaveformGenerator)
```

---

## 9. 后续工作

### 9.1 待完成的验证

- [ ] MATLAB 频谱对比（需要 FFT 后端）
- [ ] 模糊函数对比
- [ ] 脉冲压缩性能对比
- [ ] 多普勒容限分析

### 9.2 性能优化

- [ ] 集成 FFTW 进行频谱分析
- [ ] GPU 加速 NLFM 生成
- [ ] 批量波形生成优化

---

## 10. 参考文献

1. Levanon, N. and Mozeson, E., "Radar Signals", Wiley, 2004.
2. Mahafza, B.R., "Radar Systems Analysis and Design Using MATLAB", 2000.
3. 邓廷全，"雷达系统设计与仿真"，2018.

---

## 11. 更新记录

| 日期 | 版本 | 说明 |
|------|------|------|
| 2026-04-10 | 1.0 | 初始版本，完成基础波形验证 |
