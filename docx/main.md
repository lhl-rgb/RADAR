## 主循环架构

```cpp
MainLoop(radar):
    // 初始化
    TargetManager target_manager;
    target_manager.set_targets(radar.target.targets);  // 初始状态

    Scalar current_time = 0.0;
    const Scalar cpi_duration = radar.pulses_per_cpi / radar.prf_hz;

    Precompute waveform / antenna tables

    for each scan:
        for each CPI:
            beam = scan_model.update(current_time)  // 每个CPI一个波位

            // 更新目标状态到当前时刻
            target_manager.update_to_time(current_time)

            // 获取当前目标（暂不做波位筛选）
            TargetList active_targets = target_manager.get_current_targets()

            // 生成回波
            y_target  = target_engine.generate(active_targets, beam, radar, tx_waveform)
            y_clutter = clutter_engine.generate(scene, beam, radar)
            y_noise   = noise_engine.generate(radar)
            y_rx      = y_target + y_clutter + y_noise
            writer.write_iq(y_rx)

            // 时间推进
            current_time += cpi_duration
```

## 模块职责

### TargetManager（目标状态管理器）

负责：
- 维护目标初始状态（时刻0）
- 根据运动模型计算指定时刻的位置/速度
- 使用绝对时间方案：从初始状态计算

不负责：
- 波位筛选（让 TargetKinematics 的 `skip_out_of_beam_targets` 处理）

### TargetEngine（目标回波引擎）

负责：
- 接收已准备好的 TargetList
- 编排 TargetKinematics 和 TargetEchoSynthesizer
- 生成一个 CPI 的目标回波

不负责：
- 目标状态更新
- 目标筛选
- 轨迹推进

## 目标回波生成分层

### 层1：Target Kinematics
负责生成：
- 位置 position_m
- 速度 velocity_mps
- 加速度 acceleration_mps2
- RCS sigma_m2

### 层2：Target Echo Synthesis
负责生成：
- 延时 delay_s = 2R/c
- 相位 φ = -4πR(t)/λ
- 幅度（雷达方程）
- 回波矩阵（写入距离bin）