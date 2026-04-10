/**
 * @file waveform_generator.cpp
 * @brief 波形生成器实现 —— 所有波形统一时间轴 t = (n+0.5)*dt
 */

#include "waveform/waveform_generator.h"
#include "core/math_utils.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace radar::waveform {

namespace {

// ============================================================
//  辅助：Barker 码
// ============================================================
std::vector<int> barker_code(PhaseCodeType code_type) {
    switch (code_type) {
    case PhaseCodeType::Barker2:  return {1, -1};
    case PhaseCodeType::Barker3:  return {1, 1, -1};
    case PhaseCodeType::Barker4:  return {1, 1, -1, 1};
    case PhaseCodeType::Barker5:  return {1, 1, 1, -1, 1};
    case PhaseCodeType::Barker7:  return {1, 1, 1, -1, -1, 1, -1};
    case PhaseCodeType::Barker11: return {1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1};
    case PhaseCodeType::Barker13: return {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
    default:
        throw std::invalid_argument("Unsupported Barker code type.");
    }
}

// ============================================================
//  辅助：发射采样点数
// ============================================================
int samples_per_tx(const RadarSystemParams& sys) {
    if (sys.fs_hz <= 0.0 || sys.pulse_width_s <= 0.0) {
        return std::max(1, sys.samples_per_pulse);
    }
    return sys.samples_per_tx;
}

// ============================================================
//  辅助：功率归一化
// ============================================================
void normalize_average_power(ComplexVec& waveform) {
    if (waveform.empty()) return;

    Scalar power_sum = 0.0;
    for (const Complex& s : waveform) {
        power_sum += std::norm(s);
    }
    if (power_sum <= EPSILON) return;

    const Scalar scale = std::sqrt(
        static_cast<Scalar>(waveform.size()) / power_sum);
    for (Complex& s : waveform) {
        s *= scale;
    }
}

// ============================================================
//  辅助：单调函数线性插值求逆
//  给定单调递增的 x_monotonic[] 和对应的 y[]，
//  求 x_query 对应的 y 值（即 interp1 反函数）
// ============================================================
Scalar invert_monotonic_linear(
    const ScalarVector& x_monotonic,
    const ScalarVector& y,
    Scalar x_query)
{
    if (x_monotonic.empty() || y.empty() || 
        x_monotonic.size() != y.size()) {
        return 0.0;
    }
    if (x_query <= x_monotonic.front()) return y.front();
    if (x_query >= x_monotonic.back())  return y.back();

    auto it = std::upper_bound(
        x_monotonic.begin(), x_monotonic.end(), x_query);
    const auto idx_hi = static_cast<std::size_t>(
        std::distance(x_monotonic.begin(), it));
    const auto idx_lo = idx_hi - 1;

    const Scalar x0 = x_monotonic[idx_lo];
    const Scalar x1 = x_monotonic[idx_hi];
    const Scalar y0 = y[idx_lo];
    const Scalar y1 = y[idx_hi];

    if (std::abs(x1 - x0) <= EPSILON) return y0;

    const Scalar alpha = (x_query - x0) / (x1 - x0);
    return y0 + alpha * (y1 - y0);
}

// ============================================================
//  核心：统一时间轴计算
//  约定：第 n 个采样点对应 t_n = (n + 0.5) * dt
//  其中 dt = 1/fs, n = 0, 1, ..., N-1
//  这样 N 个采样点覆盖 [0.5*dt, (N-0.5)*dt] ≈ [0, T]
// ============================================================
inline Scalar sample_time(int n, Scalar dt) {
    return (static_cast<Scalar>(n) + 0.5) * dt;
}

}  // anonymous namespace

// ================================================================
//  初始化入口
// ================================================================
bool WaveformGenerator::initialize() {
    if (initialized_) return true;

    const int num_samples = samples_per_tx(sys_);

    switch (cfg_.waveform_type) {
    case WaveformType::LFM:
        waveform_ = generate_lfm(num_samples);
        break;
    case WaveformType::NLFM:
        waveform_ = generate_nlfm(num_samples);
        break;
    case WaveformType::PHASE_CODED:
        waveform_ = generate_phase_coded(num_samples, cfg_.phase_code_type);
        break;
    case WaveformType::CW:
        waveform_.assign(
            static_cast<std::size_t>(num_samples), Complex(1.0, 0.0));
        break;
    default:
        waveform_.assign(
            static_cast<std::size_t>(num_samples), Complex(0.0, 0.0));
        break;
    }

    matched_filter_ = generate_matched_filter();
    spectrum_.clear();
    initialized_ = true;
    return true;
}

// ================================================================
//  LFM 生成 —— 闭式相位
//  φ(t) = 2π[-B/2 · t + (B/2T) · t²]
//  时间轴：t_n = (n+0.5)*dt
// ================================================================
ComplexVec WaveformGenerator::generate_lfm(int num_samples) const {
    if (num_samples <= 0) return {};

    const Scalar fs = std::max(sys_.fs_hz, 1.0);
    const Scalar dt = 1.0 / fs;
    const Scalar T  = sys_.pulse_width_s;
    const Scalar B  = math::clamp_nonnegative(sys_.bw_hz);

    ComplexVec waveform(static_cast<std::size_t>(num_samples));

    if (B <= EPSILON) {
        std::fill(waveform.begin(), waveform.end(), Complex(1.0, 0.0));
        return waveform;
    }

    const Scalar chirp_rate = B / T;  // Hz/s

    for (int n = 0; n < num_samples; ++n) {
        const Scalar t = sample_time(n, dt);  // ★ 统一时间轴
        const Scalar phase = 2.0 * PI * (-0.5 * B * t + 0.5 * chirp_rate * t * t);
        waveform[static_cast<std::size_t>(n)] = std::polar(1.0, phase);
    }

    return waveform;
}

// ================================================================
//  NLFM 生成 —— 群时延法
//  ① 选窗 W(f)
//  ② 群时延 τ(f) = T × CDF[W(f)]
//  ③ 数值求逆 τ(f) → f(t)
//  ④ 累积积分 φ(t) = 2π∫f(t)dt
//  ⑤ s(t) = exp(jφ)
//  时间轴：t_n = (n+0.5)*dt  ★ 与 LFM 统一
// ================================================================
ComplexVec WaveformGenerator::generate_nlfm(int num_samples) const {
    if (num_samples <= 0) return {};

    const Scalar fs = std::max(sys_.fs_hz, 1.0);
    const Scalar dt = 1.0 / fs;
    const Scalar T  = sys_.pulse_width_s;
    const Scalar B  = math::clamp_nonnegative(sys_.bw_hz);

    if (B <= EPSILON) {
        ComplexVec waveform(
            static_cast<std::size_t>(num_samples), Complex(1.0, 0.0));
        normalize_average_power(waveform);
        return waveform;
    }

    // ---------- 第①步：频域离散 + 选窗 ----------
    const int    Nf_int = std::max(4096, 8 * num_samples);
    const auto   Nf     = static_cast<std::size_t>(Nf_int);

    ScalarVector freq_axis(Nf);
    ScalarVector weight(Nf);

    const auto window_weights = generate_window_weights(
        Nf_int, cfg_.nlfm_window_type);

    for (std::size_t k = 0; k < Nf; ++k) {
        const Scalar alpha = static_cast<Scalar>(k) /
                             static_cast<Scalar>(Nf_int - 1);
        freq_axis[k] = -0.5 * B + alpha * B;
        weight[k]    = math::clamp_nonnegative(window_weights[k]);
    }

    // ---------- 第②步：群时延 τ(f) = T × CDF[W(f)] ----------
    ScalarVector group_delay(Nf, 0.0);
    for (std::size_t k = 1; k < Nf; ++k) {
        const Scalar df    = freq_axis[k] - freq_axis[k - 1];
        const Scalar avg_w = 0.5 * (weight[k] + weight[k - 1]);
        group_delay[k]     = group_delay[k - 1] + avg_w * df;
    }

    // 归一化：显式归零起点，映射到 [0, T]
    const Scalar gd_start = group_delay.front();
    const Scalar gd_total = group_delay.back() - gd_start;

    if (gd_total <= EPSILON) {
        ComplexVec waveform(
            static_cast<std::size_t>(num_samples), Complex(1.0, 0.0));
        normalize_average_power(waveform);
        return waveform;
    }

    for (Scalar& tg : group_delay) {
        tg = T * (tg - gd_start) / gd_total;
    }

    // ---------- 第③步：数值求逆 τ(f) → f(t) ----------
    const auto N_out = static_cast<std::size_t>(num_samples);
    ScalarVector inst_freq(N_out, 0.0);

    for (int n = 0; n < num_samples; ++n) {
        const Scalar t = sample_time(n, dt);  // ★ 统一时间轴
        const Scalar t_clamped = std::clamp(t, 0.0, T);
        inst_freq[static_cast<std::size_t>(n)] =
            invert_monotonic_linear(group_delay, freq_axis, t_clamped);
    }

    // ---------- 第④⑤步：积分得相位 → 生成信号 ----------
    ComplexVec waveform(N_out);
    Scalar phase = 0.0;

    for (std::size_t n = 0; n < N_out; ++n) {
        phase += 2.0 * PI * inst_freq[n] * dt;  // dt = 采样间隔
        waveform[n] = std::polar(1.0, phase);
    }

    normalize_average_power(waveform);
    return waveform;
}

// ================================================================
//  相位编码信号
// ================================================================
ComplexVec WaveformGenerator::generate_phase_coded(
    int num_samples, PhaseCodeType code_type) const
{
    if (num_samples <= 0) return {};

    const auto chips = barker_code(code_type);
    const int  chips_count = static_cast<int>(chips.size());
    if (chips_count <= 0) return {};

    ComplexVec waveform(
        static_cast<std::size_t>(num_samples), Complex(0.0, 0.0));

    for (int c = 0; c < chips_count; ++c) {
        const int start = static_cast<int>(std::floor(
            static_cast<Scalar>(c) * num_samples / chips_count));
        const int stop  = static_cast<int>(std::floor(
            static_cast<Scalar>(c + 1) * num_samples / chips_count));

        const Complex val = (chips[c] > 0)
            ? Complex(1.0, 0.0) : Complex(-1.0, 0.0);

        for (int n = start; n < stop; ++n) {
            waveform[static_cast<std::size_t>(n)] = val;
        }
    }

    return waveform;
}

// ================================================================
//  匹配滤波器 = conj(flip(waveform))
// ================================================================
ComplexVec WaveformGenerator::generate_matched_filter() const {
    if (waveform_.empty()) return {};

    ComplexVec mf(waveform_.rbegin(), waveform_.rend());
    for (auto& s : mf) {
        s = std::conj(s);
    }
    return mf;
}

// ================================================================
//  频谱计算（预留 FFT 后端）
// ================================================================
ComplexVec WaveformGenerator::compute_spectrum(
    const ComplexVec& waveform) const
{
    (void)waveform;
    throw std::logic_error(
        "WaveformGenerator::compute_spectrum is reserved "
        "for GPU/FFTW backend.");
}

// ================================================================
//  窗函数（复数包装）
// ================================================================
ComplexVec WaveformGenerator::generate_window(
    int length, WindowType type) const
{
    const auto weights = generate_window_weights(length, type);
    ComplexVec window(weights.size());
    for (std::size_t i = 0; i < weights.size(); ++i) {
        window[i] = Complex(weights[i], 0.0);
    }
    return window;
}

// ================================================================
//  窗函数权重
// ================================================================
ScalarVector WaveformGenerator::generate_window_weights(
    int length, WindowType type) const
{
    if (length <= 0) return {};

    ScalarVector window(static_cast<std::size_t>(length), 1.0);
    if (length == 1) return window;

    for (int n = 0; n < length; ++n) {
        const Scalar x = static_cast<Scalar>(n) /
                         static_cast<Scalar>(length - 1);
        switch (type) {
        case WindowType::Hann:
            window[static_cast<std::size_t>(n)] =
                0.5 * (1.0 - std::cos(2.0 * PI * x));
            break;
        case WindowType::Hamming:
            window[static_cast<std::size_t>(n)] =
                0.54 - 0.46 * std::cos(2.0 * PI * x);
            break;
        case WindowType::Blackman:
            window[static_cast<std::size_t>(n)] =
                0.42 - 0.5 * std::cos(2.0 * PI * x)
                     + 0.08 * std::cos(4.0 * PI * x);
            break;
        default:  // Rectangular
            window[static_cast<std::size_t>(n)] = 1.0;
            break;
        }
    }
    return window;
}

}  // namespace radar