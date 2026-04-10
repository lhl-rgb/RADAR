# 天线部分验证方案

## 验证目标

验证相控阵天线模块 (ULA/UPA) 的 C++ 实现与 MATLAB 参考实现的一致性，确保：
1. 阵因子计算公式正确
2. 增益计算结果准确
3. 波束扫描功能正常

---

## 验证原理

### 1. 数学模型

#### ULA（均匀线阵）阵因子

```
ψ = 2π(d/λ)(sinθ - sinθ₀)

AF = Σ w[n] · exp(j·n·ψ), n=0,1,...,N-1

P_norm = |AF|²

G = G_peak · P_norm
```

**参数说明**：
- `θ` : 目标方位角
- `θ₀` : 波束指向角
- `d/λ` : 阵元间距（波长单位）
- `N` : 阵元数
- `w[n]` : 阵元加权系数

#### UPA（均匀平面阵）阵因子

```
u = cos(el)·sin(az)
v = sin(el)

ψx = 2π(dx/λ)(u - u₀)
ψy = 2π(dy/λ)(v - v₀)

AF = ΣΣ wx[m]·wy[n]·exp(j·(m·ψx + n·ψy))

P_norm = |AF|²
```

### 2. 验证策略

采用**端到端对比**策略：
1. C++ 和 MATLAB 使用**相同的输入参数**
2. 分别计算增益和归一化功率
3. 对比输出结果的数值差异
4. 验证误差在可接受范围内

### 3. 通过标准

| 指标 | 通过标准 |
|------|----------|
| 主瓣增益误差 | < 0.01 dB |
| 归一化功率误差 | < 1e-4 |
| 波束指向误差 | < 0.01° |

> **注**：方向图零深区域（增益 < -30dB）的 dB 误差会因数值精度被放大，该区域仅评估功率误差。

---

## 文件结构

```
Radar/
├── app/
│   └── antenna_validation.cpp      # C++ 验证工具
├── matlab/
│   └── 02 天线仿真/
│       └── compare_antenna_cpp_matlab.m   # MATLAB 对比脚本
├── test/
│   └── radar_tests.cpp             # 单元测试（含天线测试）
├── out/
│   └── antenna_validation/         # 验证输出目录
│       ├── ula_single_beam.csv
│       ├── ula_scan.csv
│       ├── upa_single_beam.csv
│       ├── upa_scan.csv
│       ├── ula_pattern.csv
│       ├── upa_pattern_2d.csv
│       └── beam_scanner.csv
└── docs/superpowers/
    └── antenna-validation-guide.md # 本文档
```

---

## 验证流程

### 步骤 1：编译验证工具

```bash
cd /home/lhlj/Radar
mkdir -p out
cmake -B out -S .
cmake --build out --target antenna_validation
cmake --build out --target radar_tests
```

### 步骤 2：运行 C++ 验证工具

```bash
./out/bin/antenna_validation --output_dir=out/antenna_validation
```

**输出说明**：
- 7 个测试用例的结果
- 6 个 CSV 数据文件（供 MATLAB 对比）

**测试用例**：
| # | 测试名称 | 说明 |
|---|----------|------|
| 1 | ULA Single Beam | ULA 单波位增益 |
| 2 | ULA Multi-Beam Scan | ULA 多波位扫描 |
| 3 | UPA Single Beam | UPA 单波位增益 |
| 4 | UPA 2D Scan | UPA 二维扫描 |
| 5 | ULA Pattern | ULA 一维方向图 |
| 6 | UPA 2D Pattern | UPA 二维方向图 |
| 7 | BeamScanner | 波束扫描控制器 |

### 步骤 3：运行 MATLAB 对比脚本

```matlab
cd matlab/02 天线仿真
compare_antenna_cpp_matlab.m
```

**MATLAB 执行内容**：
1. 读取 C++ 导出的 CSV 文件
2. 使用 MATLAB 函数计算相同参数下的增益
3. 计算误差：`error = |C++ - MATLAB|`
4. 绘制方向图对比
5. 输出验证报告

### 步骤 4：运行单元测试

```bash
./out/bin/radar_tests
```

**单元测试内容**：
- ULA 增益范围验证
- ULA 均匀加权 vs Hamming 加权
- UPA 主瓣增益验证
- UPA 方向余弦计算
- BeamScanner 基本功能
- BeamScanner 循环扫描

---

## 关键验证点

### 1. 阵因子计算

**ULA 验证点**：
- 相位差公式：`ψ = 2π(d/λ)(sinθ - sinθ₀)`
- 加权归一化：`w = w / sum(w)`
- 功率计算：`P = |AF|²`

**UPA 验证点**：
- 方向余弦：`u = cos(el)·sin(az)`, `v = sin(el)`
- 二维相位增量：`ψx`, `ψy`
- 可分离加权：`wx[m]·wy[n]`

### 2. 边界条件

| 场景 | 预期行为 |
|------|----------|
| 目标 = 波束指向 | `P_norm = 1.0`, `G = G_peak` |
| 目标偏离主瓣 | `P_norm < 1.0`, `G < G_peak` |
| 离轴角度大 | 增益显著下降（旁瓣/零深） |

### 3. 加权类型

- **Uniform**：所有阵元等权重
- **Hamming**：`w[n] = 0.54 - 0.46·cos(2πn/(N-1))`

加权后需要归一化，确保主瓣峰值增益不变。

---

## 测试结果示例

### C++ 单元测试

```
=== Antenna Tests ===
  [PASS] ULA gain in range (gain=16.772431 dB)
  [PASS] ULA normalized power valid (power=0.047560)
  [PASS] ULA uniform peak gain (expected=30.000000, actual=30.000000)
  [PASS] ULA hamming peak gain (expected=30.000000, actual=30.000000)
  [PASS] UPA peak gain at boresight (expected=35.000000, actual=35.000000)
  [PASS] UPA normalized power at boresight (expected=1.000000, actual=1.000000)
  ...
  Total: 15/15 PASS
```

### MATLAB 对比

```
Test 1: ULA Single Beam Gain Comparison
  Gain (dB): C++=16.772431, MATLAB=16.772431, Error=3.55e-15 dB
  [PASS] Gain error: 3.55e-15 < 1.00e-02 dB

Test 3: UPA Single Beam Gain Comparison
  Gain (dB): C++=35.000000, MATLAB=35.000000, Error=0.000000 dB
  [PASS] Gain error: 0.000000 < 1.00e-02 dB

Test 6: UPA 2D Pattern Comparison
  Max gain error (mainlobe > -30dB): 3.65e-12 dB
  [PASS] Max mainlobe gain error < 1.00e-02 dB
```

---

## 常见问题

### Q1: 为什么零深区域的 dB 误差很大？

**A**: 在方向图零深区域，功率值接近 0（如 1e-10），微小的功率差异（1e-15）经过 dB 转换后会被放大：

```
P1 = 1.0e-10  →  -100 dB
P2 = 1.0e-10 + 1.0e-15  →  -99.9957 dB
Error = 0.0043 dB（看似很大，但实际功率差异极小）
```

**解决方案**：只评估主瓣区域（增益 > -30dB）的 dB 误差，零深区域评估功率误差。

### Q2: 如何修改测试参数？

**A**: 编辑 `app/antenna_validation.cpp` 中的测试参数：

```cpp
// Test 1: ULA 单波位
cfg.num_elements_az = 16;     // 阵元数
cfg.spacing_az_lambda = 0.5;  // 阵元间距
cfg.peak_gain_db = 30.0;      // 峰值增益
```

然后重新编译运行。

### Q3: 如何添加新的测试用例？

**A**: 
1. 在 `antenna_validation.cpp` 中添加新的 `test_xxx()` 函数
2. 在 `main()` 中调用新测试
3. 在 `compare_antenna_cpp_matlab.m` 中添加对应的 MATLAB 验证
4. 在 `radar_tests.cpp` 中添加单元测试

---

## 维护说明

### 更新 MATLAB 参考函数

如果 MATLAB 天线仿真代码有更新，需要同步更新 `compare_antenna_cpp_matlab.m` 中的参考函数：
- `matlab_ula_gain_to_target()`
- `matlab_upa_gain_to_target()`

### 扩展验证场景

可添加的验证场景：
- 不同加权类型对比（Hann, Blackman）
- 栅瓣特性验证（d/λ > 0.5 时）
- 宽角扫描特性（scan > 60°）
- 极化特性（如支持）

---

## 参考资料

- MATLAB 天线仿真代码：`matlab/02 天线仿真/`
- C++ 天线实现：`src/antenna/`, `include/antenna/`
- 单元测试：`test/radar_tests.cpp`

---

**文档创建日期**: 2026-04-09  
**最后更新**: 2026-04-09  
**验证状态**: ✅ 全部通过
