# 相控阵雷达测试记录

## 1. 测试元信息
- 测试时间: 2026-03-25 21:30:04 CST
- 平台环境: Linux 6.6.87.2-microsoft-standard-WSL2
- 工程路径: `/home/lhlj/Radar`

## 2. 本轮变更摘要（NoiseEngine）
| 项目 | 处理结果 | 关联位置 |
|---|---|---|
| 噪声模式定义 | 新增 `NoiseLevelMode`：`ComplexSigma/NoisePower/ThermalKTB` | `include/core/radar_params.h` |
| 生成主接口 | 定稿为 `sample()` 与 `generate(n)` | `include/noise/noise_engine.h`, `src/noise/noise_engine.cpp` |
| 叠加主接口 | 新增 `add_noise(ComplexVec&)` 与 `add_noise(CpiEcho&)` | `include/noise/noise_engine.h`, `src/noise/noise_engine.cpp` |
| 参数失败回滚 | `set_params` 改为 `bool` 返回并提供 `last_error` | `include/noise/noise_engine.h`, `src/noise/noise_engine.cpp` |
| 重复计算优化 | 改为设置参数时缓存 `noise_power/sigma/sigma_iq`，采样阶段只读缓存 | `src/noise/noise_engine.cpp` |
| 旧接口清理 | 删除 `sample_real/sample_complex/generate_*_noise/add_noise_inplace`，仅保留主接口 | `include/noise/noise_engine.h`, `src/noise/noise_engine.cpp` |
| 头文件修复 | 移除错误自包含与 `../include/...` 风格路径 | `include/noise/noise_engine.h`, `src/noise/noise_engine.cpp` |

## 3. 执行命令
```bash
cmake -S . -B out/build
cmake --build out/build -j
./out/build/test/radar_tests
ctest --test-dir out/build --output-on-failure
```

## 4. 模块测试结果
| 用例 | 预期 | 实际 | 结论 |
|---|---|---|---|
| RadarParams.DefaultDerivedAndValidate | 默认参数派生值有效，`validate()==true` | 通过 | PASS |
| RadarParams.InvalidPRIRejected | `pulse_width > PRI` 被判非法 | 通过 | PASS |
| RadarParams.InvalidFsBwRejected | `fs < bw` 被判非法 | 通过 | PASS |
| WaveformGenerator.LFMAndMatchedFilter | LFM 长度正确，匹配滤波共轭反转正确 | 通过 | PASS |
| WaveformGenerator.PhaseCoded | 相位编码波形生成正常（Barker） | 通过 | PASS |
| WaveformGenerator.NLFMAndSpectrumStub | NLFM 生成正常，频谱占位接口抛异常 | 通过 | PASS |
| PhasedArrayAntenna.ULAResponse | ULA 主瓣中心响应高于离轴 | 通过 | PASS |
| PhasedArrayAntenna.UPAResponse | UPA 主瓣中心响应高于离轴 | 通过 | PASS |
| AntennaScanModel.CsvAndLoopAndEmptyProtection | CSV 读取、循环推进、空表保护行为正确 | 通过 | PASS |
| NoiseEngine.ModeComplexSigma | `sigma` 模式功率统计接近 `sigma^2` | 通过 | PASS |
| NoiseEngine.ModeNoisePower | 功率模式下 `noise_sigma=sqrt(power)` 且统计一致 | 通过 | PASS |
| NoiseEngine.ModeThermalKTB | 热噪声模式与 `kTBF` 计算一致 | 通过 | PASS |
| NoiseEngine.InvalidParamsRollback | 非法参数失败且旧配置保持不变 | 通过 | PASS |
| NoiseEngine.SeedRepeatability | 同 seed 生成序列一致，`reseed` 可复现 | 通过 | PASS |
| NoiseEngine.AddNoiseThreeLayers | 向量/脉冲语义/CPI 三层加噪均生效 | 通过 | PASS |

## 5. 集成测试结果
| 场景 | 预期 | 实际 | 结论 |
|---|---|---|---|
| 1DScanGainTrendWithWaveform | 波位扫过目标方向时增益峰值出现在对准位置 | 通过 | PASS |
| 2DScanGainTrendAndFailurePath | 2D 对准增益提升；坏 CSV 路径可诊断失败 | 通过 | PASS |
| WaveformPlusNoisePowerTrend | 波形链路加噪后功率上升，CPI 加噪联动正确 | 通过 | PASS |

## 6. 关键控制台输出摘要
```text
=== MODULE TESTS ===
[MODULE] RadarParams.DefaultDerivedAndValidate ... PASS
[MODULE] RadarParams.InvalidPRIRejected ... PASS
[MODULE] RadarParams.InvalidFsBwRejected ... PASS
[MODULE] WaveformGenerator.LFMAndMatchedFilter ... PASS
[MODULE] WaveformGenerator.PhaseCoded ... PASS
[MODULE] WaveformGenerator.NLFMAndSpectrumStub ... PASS
[MODULE] PhasedArrayAntenna.ULAResponse ... PASS
[MODULE] PhasedArrayAntenna.UPAResponse ... PASS
[MODULE] AntennaScanModel.CsvAndLoopAndEmptyProtection ... PASS
[MODULE] NoiseEngine.ModeComplexSigma ... PASS
[MODULE] NoiseEngine.ModeNoisePower ... PASS
[MODULE] NoiseEngine.ModeThermalKTB ... PASS
[MODULE] NoiseEngine.InvalidParamsRollback ... PASS
[MODULE] NoiseEngine.SeedRepeatability ... PASS
[MODULE] NoiseEngine.AddNoiseThreeLayers ... PASS
=== INTEGRATION TESTS ===
[INTEGRATION] 1DScanGainTrendWithWaveform ... PASS
[INTEGRATION] 2DScanGainTrendAndFailurePath ... PASS
[INTEGRATION] WaveformPlusNoisePowerTrend ... PASS
[SUMMARY] total=18 pass=18 fail=0
```

## 7. 未解决问题与后续建议
- 当前无失败用例。
- 统计型测试阈值当前按经验值设置（8%）；后续可按样本数给出更严格置信区间判据。
- 当前噪声仍为白噪声模型；若后续需要，可扩展有色噪声/脉间相关噪声模块。
