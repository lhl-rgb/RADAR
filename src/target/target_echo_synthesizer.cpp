/**
 * @file target_echo_synthesizer.cpp
 * @brief 目标真值到 CPI 回波写入与叠加实现
 * @details
 * 层2：Target Echo Synthesis 负责生成：
 * - 延时（delay_s = 2R/c）
 * - 相位（传播相位 + 多普勒相位旋转）
 * - 幅度（雷达方程）
 * - 回波矩阵（写入距离bin）
 */

#include "target/target_echo_synthesizer.h"
#include "core/math_utils.h"

#include <cmath>

namespace radar {

bool TargetEchoSynthesizer::generate_target_echo(const TargetTrajectory & trajectory,
                                                   const RadarParams& radar,
                                                   const ComplexVec& tx_waveform,
                                                   CpiEcho& out_echo,
                                                   std::string& error) {
    // 清空错误信息
    error.clear();

    // ==================== 输入参数校验 ====================
    if (radar.pulses_per_cpi <= 0 || radar.samples_per_pulse <= 0) {
        error = "RadarParams invalid: pulses_per_cpi and samples_per_pulse must be positive.";
        return false;
    }
    if (tx_waveform.empty()) {
        error = "TargetEchoSynthesizer invalid input: tx_waveform must not be empty.";
        return false;
    }
    // 校验目标轨迹快照数与雷达CPI脉冲数一致
    if (trajectory.snapshots.size() != static_cast<std::size_t>(radar.pulses_per_cpi)) {
        error = "TargetTrajectory size mismatch with radar.pulses_per_cpi.";
        return false;
    }
    // 校验输出回波脉冲数与雷达CPI脉冲数一致
    if (out_echo.pulses.size() != static_cast<std::size_t>(radar.pulses_per_cpi)) {
        error = "Output CpiEcho pulse count mismatch with radar.pulses_per_cpi.";
        return false;
    }

    // ==================== 雷达参数预处理 ====================
    // 参考延时：对应最小作用距离的双程传播时间
    const Scalar tau_ref_s = 2.0 * radar.min_range_m / C;
    // 采样率、波长（确保正值，避免除零）
    const Scalar fs_hz = math::clamp_positive_eps(radar.fs_hz);
    const Scalar lambda_m = math::clamp_positive_eps(radar.wavelength_m);
    const TargetParams& target_params = radar.target_params;

    // ==================== 逐脉冲回波合成 ====================
    for (int pulse = 0; pulse < radar.pulses_per_cpi; ++pulse) {
        // 获取当前脉冲对应的目标状态快照
        const TargetSnapshot& sample = trajectory.snapshots[static_cast<std::size_t>(pulse)];
        
        // 若目标增益或RCS无效，跳过该脉冲（目标不可见）
        if (sample.gain_linear <= 0.0 || sample.rcs_m2 <= 0.0) {
            continue;
        }

        // ---------- 时延计算（快时间维采样索引） ----------
        // 双程传播延时：t = 2R/c
        const Scalar delay_s = (2.0 * sample.range_m) / C;
        // 相对参考点的快时间（用于距离门对齐）
        const Scalar fast_time_s = delay_s - tau_ref_s;
        // 转换为离散采样索引（四舍五入取整）
        const int delay_idx = static_cast<int>(std::llround(fast_time_s * fs_hz));

        // ---------- 幅度计算（雷达方程） ----------
        const Scalar gain_linear = math::clamp_nonnegative(sample.gain_linear);
        const Scalar rcs_m2 = math::clamp_positive_eps(sample.rcs_m2);
        const Scalar current_range_m = sample.position_m.norm();

        // 基础幅度：sqrt(G·σ)，G为天线增益，σ为RCS
        Scalar amp = std::sqrt(gain_linear * rcs_m2);
        
        // 可选：启用双程传播损耗（完整的雷达方程）
        if (target_params.enable_two_way_propagation_loss) {
            // 雷达方程分子：P_t · G² · λ² · σ
            const Scalar numerator =
                radar.peak_power_w * gain_linear * gain_linear * lambda_m * lambda_m * rcs_m2;
            // 雷达方程分母：(4π)³ · R⁴ · L_sys
            const Scalar denominator =
                std::pow(4.0 * PI, 3.0) * std::pow(current_range_m, 4.0) *
                math::clamp_positive_eps(radar.system_loss_linear);
            // 接收信号幅度 ∝ sqrt(接收功率)
            amp = std::sqrt(math::safe_div(numerator, denominator));
        }

        // ---------- 相位计算（多普勒/距离相位） ----------
        Scalar phase = 0.0;
        if (target_params.enable_phase) {
            // 双程传播相位：φ = -4πR/λ
            // 负号表示回波相位滞后，包含距离和多普勒信息
            phase = -4.0 * PI * current_range_m / lambda_m;
        }

        // 合成复系数：幅度 × 相移
        const Complex phasor(std::cos(phase), std::sin(phase));
        const Complex coeff = amp * phasor;

        // ---------- 波形卷积：发射波形搬移到目标时延处 ----------
        PulseEcho& pulse_echo = out_echo.pulses[static_cast<std::size_t>(pulse)];
        for (std::size_t n = 0; n < tx_waveform.size(); ++n) {
            // 计算该采样点在距离门中的实际位置
            const int sample_idx = delay_idx + static_cast<int>(n);
            // 边界检查：确保不越出脉冲采样范围
            if (sample_idx < 0 || sample_idx >= radar.samples_per_pulse) {
                continue;
            }
            // 叠加目标回波：复系数 × 发射波形采样（线性叠加支持多目标）
            pulse_echo[static_cast<std::size_t>(sample_idx)] += coeff * tx_waveform[n];
        }
    }

    return true;
}

}  // namespace radar
