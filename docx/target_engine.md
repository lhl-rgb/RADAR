  # Radar C++ TargetEngine 架构设计

  ## Summary

  面向 现有工程，TargetEngine 应按与 ClutterEngine /
  NoiseEngine 一致的模块风格设计：
  公开职责只做“当前 CPI 活动目标集合 -> 目标回波 CpiEcho 生成”，不负责轨迹推进；输入目标状态已由外部场景/调度层更新到
  当前 CPI。
  第一版只做 多目标点目标，支持 雷达本地直角坐标、直线/圆周/匀加速 运动、平均 RCS(m²) 和 Swerling I~IV，公开入口保持你
  给的三参形式：target_engine.generate(active_targets, beam, radar)。
  输出对外统一为 CpiEcho，与海杂波/噪声模块同型，便于后续做 target + clutter + noise 叠加。

  ## Key Changes

  ### 1. 模块分层

  - TargetEngine 作为对外门面，职责是参数校验、流程编排、统一错误出口、生成一个 CPI 的目标回波。
  - 将具体计算拆到 target 子域内部组件，避免把所有物理逻辑堆进 TargetEngine：
      - TargetModel：单目标状态与配置描述
      - TargetKinematics：按输入状态生成本 CPI 每脉冲几何真值
      - TargetRcsModel：Swerling 0/I/II/III/IV 起伏
      - TargetEchoSynthesizer：距离延迟、相位、多普勒、幅度和波束增益合成
  - TargetEngine 不依赖场景管理器，不持有全局轨迹，不承担“目标筛选是否 active”的职责；active_targets 已经过滤完成。

  ### 2. Public API 设计

  - 公开主接口保持：
      - bool generate(const TargetList& active_targets, const BeamView& beam, const RadarParams& radar, CpiEcho&
        out_echo);
  - 若必须严格贴合表达式 y_target = target_engine.generate(active_targets, beam, radar)，建议额外提供便捷重载：
      - CpiEcho generate_or_throw(const TargetList& active_targets, const BeamView& beam, const RadarParams&
        radar);
  - TargetEngine 还应提供：
      - bool set_params(const TargetEngineParams&)
      - const TargetEngineParams& params() const
      - const std::string& last_error() const
  - 对外不直接暴露“逐脉冲真值矩阵”作为主返回值；若需要验证，可提供独立调试接口或可选诊断输出，不污染主生成接口。

  ### 3. 关键类型与接口

  - 新增 TargetEngineParams
      - bool enable_beam_gain = true
      - bool enable_two_way_propagation_loss = true
      - bool enable_phase = true
      - bool enable_swerling = true
      - uint64_t seed
      - Scalar beam_gate_threshold_db
      - bool skip_out_of_beam_targets
  - 新增 TargetState
      - uint64_t id
      - Vec3 position_m
      - Vec3 velocity_mps
      - Vec3 acceleration_mps2
      - MotionModel motion_model
      - Scalar rcs_mean_m2
      - SwerlingType swerling
      - bool enabled
  - 新增 TargetList
      - 本质为 std::vector<TargetState> 或轻量别名；第一版不引入复杂容器
  - 新增 BeamView
      - 包含 AzEl pointing
      - 包含 int beam_index
      - 可选包含 const PhasedArrayAntenna* antenna
  - 新增 TargetSnapshot
      - Scalar time_s
      - Scalar range_m
      - Scalar radial_velocity_mps
      - Scalar delay_s
      - Scalar doppler_hz
      - Scalar gain_linear
      - Scalar sigma_m2
  - 新增 TargetTrajectory 
      - 一个目标在当前 CPI 的逐脉冲真值序列
  - 类型关系
      - TargetEngine::generate(...) 只消费 TargetList / BeamView / RadarParams
      - 内部为每个目标生成 TargetTrajectory 
      - 内部把所有目标累加到 CpiEcho

  ### 4. 数据流与算法口径

  - 输入假设
      - active_targets 中每个目标状态均以雷达本地直角坐标给出
      - beam 表示当前 CPI 使用的波位与 beam index
      - RadarParams 已完成 compute_derived_params() 且通过 validate()
  - 单目标 CPI 生成流程
      - 读取当前目标初始状态
      - 在 CPI 慢时间上按脉冲时刻生成 position/velocity
      - 计算 R = ||p||
      - 计算 Vr = dot(p, v) / ||p||
      - 计算 tau = 2R/c
      - 计算 fd = 2Vr/lambda
      - 采样 Swerling RCS
      - 结合波束增益和传播损耗得到复幅度
      - 将每个目标逐脉冲写入 out_echo.pulses[pulse_idx][sample_idx]
  - 多目标合成
      - 目标回波按复数线性叠加
      - 第一版不做目标间遮挡、多径、相互耦合
  - 坐标口径
      - 第一版内部一律用雷达本地直角坐标
      - 不在 TargetEngine 内做经纬度到 ENU/本地坐标转换
  - 波束口径
      - 若 beam.antenna 可用且启用 beam gain，则通过现有天线增益接口计算目标方向增益
      - 若无天线对象，则默认单位增益，不让接口失效
  - 起伏口径
      - Swerling I/III：CPI 内慢起伏，同一 CPI 全脉冲共享一个瞬时 RCS
      - Swerling II/IV：CPI 内快起伏，每脉冲独立采样
      - rcs_mean_m2 始终为场景统一输入单位

  ## Implementation Changes

  ### 1. 文件与模块布局

  - include/target/target_engine.h
      - 对外公开 TargetEngine、TargetEngineParams、主接口
  - include/target/target_types.h
      - 放 TargetState、BeamView、TargetSnapshot、TargetTrajectory  等类型
  - include/target/target_kinematics.h
      - 放逐脉冲运动学计算接口
  - include/target/target_rcs_model.h
      - 放 Swerling 采样接口
  - include/target/target_echo_synthesizer.h
      - 放单目标/多目标回波写入接口
  - src/target/*.cpp
      - 对应实现；target_engine.cpp 只做编排，不承载大段物理公式

  ### 2. 与现有工程的衔接

  - 与 core/types.h
      - 继续复用 Scalar / Vec3 / Complex / PulseEcho / CpiEcho
  - 与 core/radar_params.h
      - 第一版不把 target 参数塞进 RadarParams
      - RadarParams 继续只描述雷达体制/天线/场景公共参数
  - 与 core/antenna_set.h
      - 通过 PhasedArrayAntenna::gain(...) 或 is_target_in_beam(...) 复用现有波束逻辑
  - 与 clutter/noise
      - TargetEngine 单独输出 CpiEcho
      - 上层汇总器负责 target + clutter + noise
      - 不在 TargetEngine 内直接加噪或加杂波

  ### 3. 错误处理与可复现性

  - 遵循现有模块风格：bool + last_error()
  - 对非法输入直接返回 false
      - 空 active_targets 不是错误，返回合法的全零 CpiEcho
      - samples_per_pulse <= 0、pulses_per_cpi <= 0、rcs_mean_m2 <= 0、非有限位置/速度 为错误
  - 随机性
      - Swerling 随机数由 TargetEngineParams.seed 控制
      - 若需要可复现，目标级播种建议混入 target_id + beam_index + cpi_index
      - 若当前接口没有 cpi_index，则先只用 target_id + beam_index + engine_seed，并在设计说明里保留未来扩展点

  ### 4. 性能与扩展边界

  - 第一版用 CPU 串行实现，保证正确性和接口稳定
  - 内层组织按“目标外层、脉冲中层、样本写入内层”即可；不提前引入 SIMD/GPU
  - 从设计上预留未来扩展：
      - TargetState 可扩为“目标由多个散射点组成”
      - TargetEchoSynthesizer 可扩为 PointTargetSynthesizer / ExtendedTargetSynthesizer
  - 当前不纳入：
      - 多径
      - 海面镜像
      - 船体散射中心组
      - 微动
      - 极化差异建模
      - RCS 频率/姿态依赖表

  ## Test Plan

  - 模块单测
      - 空目标列表返回全零 CpiEcho
      - 单静止目标：主峰在理论距离 bin，慢时间无明显多普勒旋转
      - 单匀速目标：主峰距离/多普勒与理论值一致
      - 匀加速/圆周运动：逐脉冲 R/Vr 与解析值一致
      - Swerling1/3 同一 CPI 内不变，跨调用变化
  - Beam 相关单测
      - 目标在主瓣内时增益高于离轴目标
      - 启用 skip_out_of_beam_targets 时，波束外目标可被裁剪
  - 集成单测
      - TargetEngine 输出 CpiEcho 维度与 RadarParams 一致
      - 多目标叠加结果的能量高于单目标
      - target + clutter + noise 可在上层无类型转换直接叠加
  - 验证用例
      - 对照现有 MATLAB 验证基线，至少复现：
          - 单目标静止
          - 单目标匀速
          - 三目标分离
          - 近距离近邻但多普勒分离目标

  ## Assumptions

  - active_targets 由外部场景层提前筛选并更新到当前 CPI 起始时刻，TargetEngine 不做生命周期管理。
  - 第一版 TargetEngine 只做点目标，不直接实现船只多散射点模型。
  - 主公开接口保持三参入口语义；工程上通过 BeamView 吸收 beam_index 和可选天线对象。
  - 主返回值固定为 CpiEcho；若需要真值/调试信息，作为附加诊断接口，不改变主链路返回类型。
  - 第一版采用雷达本地直角坐标作为唯一目标输入坐标系，不在模块内部处理经纬度转换。




  目标回波生成逻辑梳理
整体架构

┌─────────────────────────────────────────────────────────────────┐
│                        TargetEngine                              │
│  (编排层：参数校验、流程调度、错误处理)                            │
└─────────────────────────────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          ▼                   ▼                   ▼
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ TargetManager   │  │TargetKinematics │  │TargetEcho       │
│ (状态管理)      │  │(层1：运动学)    │  │Synthesizer      │
│                 │  │                 │  │(层2：回波合成)  │
└─────────────────┘  └─────────────────┘  └─────────────────┘
层1：TargetKinematics（运动学计算）
输入：TargetState（CPI起始时刻的目标状态）

输出：TargetTrajectory （逐脉冲真值序列）


for (pulse = 0; pulse < pulses_per_cpi; pulse++) {
    // 1. 时间
    time_s = pulse * PRI

    // 2. 位置/速度/加速度（根据运动模型）
    position_m = p0 + v0*t + 0.5*a*t²
    velocity_mps = v0 + a*t
    acceleration_mps2 = a

    // 3. 几何量
    range_m = ||position||
    radial_velocity = (position · velocity) / range
    radial_acceleration = (position · acceleration) / range

    // 4. 波束增益
    az, el = position_to_az_el(position)
    gain_linear = antenna.gain(az, el, beam_az, beam_el)

    // 5. RCS（Swerling起伏）
    sigma_m2 = draw_rcs(swerling_type, rcs_mean)
}
关键点：

静止：位置不变
匀速：位置线性变化
匀加速：位置二次变化，速度线性变化
Swerling 1/3（慢起伏）：CPI内RCS不变
Swerling 2/4（快起伏）：每脉冲独立采样
层2：TargetEchoSynthesizer（回波合成）
输入：TargetTrajectory  + tx_waveform

输出：写入 CpiEcho


for (pulse = 0; pulse < pulses_per_cpi; pulse++) {
    // 1. 延时计算
    delay_s = 2 * range_m / c
    fast_time_s = delay_s - tau_ref  // 相对于min_range
    delay_idx = round(fast_time_s * fs)

    // 2. 幅度计算（雷达方程）
    if (enable_two_way_propagation_loss) {
        // 完整雷达方程
        amp = sqrt(Pt * G² * λ² * σ / ((4π)³ * R⁴ * L))
    } else {
        // 简化
        amp = sqrt(G * σ)
    }

    // 3. 相位计算（双程传播相位）
    phase = -4π * R(t) / λ

    // 4. 写入距离bin（发射波形卷积）
    for (n = 0; n < waveform_length; n++) {
        sample_idx = delay_idx + n
        out_echo[pulse][sample_idx] += amp * exp(j*phase) * tx_waveform[n]
    }
}
关键公式：

物理量	公式
延时	τ = 2R/c
多普勒	fd = 2vr/λ
相位	φ = -4πR/λ（双程）
幅度	√(Pt G² λ² σ / (4π)³ R⁴ L)
数据流

TargetState (CPI起始状态)
     │
     ▼
┌─────────────────────────────────────────────────────────────┐
│ TargetKinematics::generate_trajectory()                         │
│                                                             │
│  pulse 0: pos, vel, range, radial_v, gain, rcs             │
│  pulse 1: pos, vel, range, radial_v, gain, rcs             │
│  ...                                                        │
│  pulse N-1: pos, vel, range, radial_v, gain, rcs           │
└─────────────────────────────────────────────────────────────┘
     │
     ▼
TargetTrajectory  (逐脉冲真值)
     │
     ▼
┌─────────────────────────────────────────────────────────────┐
│ TargetEchoSynthesizer::generate_target_echo()            │
│                                                             │
│  pulse 0: delay → delay_idx, amp, phase → 写入回波          │
│  pulse 1: delay → delay_idx, amp, phase → 写入回波          │
│  ...                                                        │
│  pulse N-1: delay → delay_idx, amp, phase → 写入回波        │
└─────────────────────────────────────────────────────────────┘
     │
     ▼
CpiEcho (脉冲×距离采样复矩阵)
多目标叠加

// TargetEngine::generate()
for (target : active_targets) {
    truth = TargetKinematics::generate_trajectory(target)
    TargetEchoSynthesizer::generate_target_echo(truth, out_echo)
}
每个目标的回波线性叠加到 out_echo，相位包含多普勒信息。