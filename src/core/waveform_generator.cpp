#include "core/waveform_generator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace radar {

namespace {

std::vector<int> barker_code(PhaseCodeType code_type) {
    switch (code_type) {
    case PhaseCodeType::Barker2:
        return {1, -1};
    case PhaseCodeType::Barker3:
        return {1, 1, -1};
    case PhaseCodeType::Barker4:
        return {1, 1, -1, 1};
    case PhaseCodeType::Barker5:
        return {1, 1, 1, -1, 1};
    case PhaseCodeType::Barker7:
        return {1, 1, 1, -1, -1, 1, -1};
    case PhaseCodeType::Barker11:
        return {1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1};
    case PhaseCodeType::Barker13:
        return {1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1};
    default:
        throw std::invalid_argument("Unsupported Barker code type.");
    }
}

//tp时间内的采样点数
int default_tx_samples(const RadarParams& params) {
    if (params.fs_hz <= 0.0 || params.pulse_width_s <= 0.0) {
        return std::max(1, params.samples_per_pulse);
    }
    return std::max(1, static_cast<int>(std::ceil(params.pulse_width_s * params.fs_hz)));
}
//归一化功率
void normalize_average_power(ComplexVec& waveform) {
    if (waveform.empty()) {
        return;
    }
    Scalar power_sum = 0.0;
    for (const Complex sample : waveform) {
        power_sum += std::norm(sample);
    }
    if (power_sum <= EPSILON) {
        return;
    }
    const Scalar scale = std::sqrt(static_cast<Scalar>(waveform.size()) / power_sum);
    for (Complex& sample : waveform) {
        sample *= scale;
    }
}
Scalar invert_monotonic_linear(
    const std::vector<Scalar>& x_monotonic,
    const std::vector<Scalar>& y,
    Scalar x_query){

    if (x_monotonic.empty() || y.empty() || x_monotonic.size() != y.size()) {
        return 0.0;
    }
    if (x_query <= x_monotonic.front()) {
        return y.front();
    }
    if (x_query >= x_monotonic.back()) {
        return y.back();
    }
    auto it = std::upper_bound(x_monotonic.begin(), x_monotonic.end(), x_query);
    const std::size_t idx_hi =
        static_cast<std::size_t>(std::distance(x_monotonic.begin(), it));
    const std::size_t idx_lo = idx_hi - 1;
    const Scalar x0 = x_monotonic[idx_lo];
    const Scalar x1 = x_monotonic[idx_hi];
    const Scalar y0 = y[idx_lo];
    const Scalar y1 = y[idx_hi];
    if (std::abs(x1 - x0) <= EPSILON) {
        return y0;
    }
    const Scalar alpha = (x_query - x0) / (x1 - x0);
    return y0 + alpha * (y1 - y0);
}
}  // namespace

WaveformGenerator::WaveformGenerator(const RadarParams& params) {
    set_params(params);
}

void WaveformGenerator::set_params(const RadarParams& params) {
    params_ = params;
    const int num_samples = default_tx_samples(params_);

    switch (params_.waveform_type) {
    case WaveformType::LFM:
        waveform_ = generate_lfm(num_samples);
        break;

    case WaveformType::NLFM:
        waveform_ = generate_nlfm(num_samples);
        break;

    case WaveformType::PHASE_CODED:
        waveform_ = generate_phase_coded(num_samples, params_.phase_code_type);
        break;

    case WaveformType::CW:
        waveform_.assign(static_cast<std::size_t>(num_samples), Complex(1.0, 0.0));
        break;

    default:
        waveform_.assign(static_cast<std::size_t>(num_samples), Complex(0.0, 0.0));
        break;
    }

    matched_filter_ = generate_matched_filter();
    spectrum_.clear();
}

ComplexVec WaveformGenerator::generate_lfm(int num_samples) const {
    if (num_samples <= 0) {
        return {};
    }

    ComplexVec waveform(static_cast<std::size_t>(num_samples), Complex(1.0, 0.0));

    const Scalar fs = std::max(params_.fs_hz, 1.0);
    const Scalar dt = 1.0 / fs;
    const Scalar T  = std::max(params_.pulse_width_s, num_samples * dt);
    const Scalar B  = std::max(params_.bw_hz, 0.0);

    if (B <= EPSILON) {
        return waveform;
    }

    for (int n = 0; n < num_samples; ++n) {
        const Scalar t = (static_cast<Scalar>(n) + 0.5) * dt;
        const Scalar phase =
            2.0 * PI * (-0.5 * B * t + 0.5 * (B / T) * t * t);
        waveform[static_cast<std::size_t>(n)] = std::polar(1.0, phase);
    }

    return waveform;
}

ComplexVec WaveformGenerator::generate_nlfm(int num_samples) const
{
    if (num_samples <= 0) {
        return {};
    }

    const Scalar fs = std::max(params_.fs_hz, 1.0);
    const Scalar dt = 1.0 / fs;
    const Scalar T  = std::max(params_.pulse_width_s, num_samples * dt);
    const Scalar B  = std::max(params_.bw_hz, 0.0);

    if (B <= EPSILON) {
        // 无带宽时，退化成常数信号
        ComplexVec waveform(static_cast<std::size_t>(num_samples), Complex(1.0, 0.0));
        normalize_average_power(waveform);
        return waveform;
    }

    // -------- 1) 频域离散 ----------
    // 频域采样点数建议比时域采样更密一些，保证反插值平滑
    const int num_freq_samples = std::max(4096, 8 * num_samples);

    std::vector<Scalar> freq_axis(static_cast<std::size_t>(num_freq_samples), 0.0);
    std::vector<Scalar> weight(static_cast<std::size_t>(num_freq_samples), 1.0);

    const auto window_weights = generate_window_weights(num_freq_samples, params_.nlfm_window_type);
    // 若你想换窗，可以把 "hamming" 改成 params_.nlfm_window_type 之类

    for (int k = 0; k < num_freq_samples; ++k) {
        const Scalar alpha = static_cast<Scalar>(k) /
                             static_cast<Scalar>(num_freq_samples - 1);
        freq_axis[static_cast<std::size_t>(k)] = -0.5 * B + alpha * B;

        // 这里建议用窗函数的平方作为“能量分布”权重，更常见一些
        const Scalar w = std::max(window_weights[static_cast<std::size_t>(k)], 0.0);
        weight[static_cast<std::size_t>(k)] = w * w;
    }

    // -------- 2) 构造群时延 T(f) ----------
    // 用累积梯形积分实现：
    // tau_g(f) = K * ∫ W(f) df
    std::vector<Scalar> group_delay(static_cast<std::size_t>(num_freq_samples), 0.0);

    for (int k = 1; k < num_freq_samples; ++k) {
        const Scalar df = freq_axis[static_cast<std::size_t>(k)] -
                          freq_axis[static_cast<std::size_t>(k - 1)];
        const Scalar avg_w =
            0.5 * (weight[static_cast<std::size_t>(k)] +
                   weight[static_cast<std::size_t>(k - 1)]);
        group_delay[static_cast<std::size_t>(k)] =
            group_delay[static_cast<std::size_t>(k - 1)] + avg_w * df;
    }

    const Scalar integral_total = group_delay.back();
    if (integral_total <= EPSILON) {
        ComplexVec waveform(static_cast<std::size_t>(num_samples), Complex(1.0, 0.0));
        normalize_average_power(waveform);
        return waveform;
    }

    // 归一化到 [0, T]
    for (Scalar& tg : group_delay) {
        tg = T * tg / integral_total;
    }

    // -------- 3) 由 t = T(f) 反求 f(t) ----------
    std::vector<Scalar> inst_freq(static_cast<std::size_t>(num_samples), 0.0);
    for (int n = 0; n < num_samples; ++n) {
        const Scalar t = (static_cast<Scalar>(n) + 0.5) * dt;
        const Scalar t_clamped = std::clamp(t, Scalar(0.0), T);
        inst_freq[static_cast<std::size_t>(n)] =
            invert_monotonic_linear(group_delay, freq_axis, t_clamped);
    }

    // -------- 4) 积分得到相位 ----------
    ComplexVec waveform(static_cast<std::size_t>(num_samples));
    Scalar phase = 0.0;

    for (int n = 0; n < num_samples; ++n) {
        phase += 2.0 * PI * inst_freq[static_cast<std::size_t>(n)] * dt;
        waveform[static_cast<std::size_t>(n)] = std::polar(1.0, phase);
    }

    // -------- 5) 标准 NLFM：不再做时域幅度加窗 ----------
    // 保持恒包络，只做平均功率归一化
    normalize_average_power(waveform);
    return waveform;
}

ComplexVec WaveformGenerator::generate_phase_coded(int num_samples,PhaseCodeType code_type) const{
    if (num_samples <= 0) {
        return {};
    }

    const std::vector<int> chips = barker_code(code_type);
    const int chips_count = static_cast<int>(chips.size());
    if (chips_count <= 0) {
        return {};
    }

    ComplexVec waveform(static_cast<std::size_t>(num_samples), Complex(0.0, 0.0));

    for (int chip_idx = 0; chip_idx < chips_count; ++chip_idx) {
        const int start = static_cast<int>(
            std::floor(static_cast<Scalar>(chip_idx) * num_samples / chips_count));
        const int stop = static_cast<int>(
            std::floor(static_cast<Scalar>(chip_idx + 1) * num_samples / chips_count));

        const Complex chip_value = (chips[chip_idx] > 0)
            ? Complex(1.0, 0.0)
            : Complex(-1.0, 0.0);

        for (int n = start; n < stop; ++n) {
            waveform[static_cast<std::size_t>(n)] = chip_value;
        }
    }

    return waveform;
}



ComplexVec WaveformGenerator::generate_matched_filter() const {
    if (waveform_.empty()) {
        return {};
    }

    ComplexVec matched_filter(waveform_.rbegin(), waveform_.rend());
    for (auto& sample : matched_filter) {
        sample = std::conj(sample);
    }
    return matched_filter;
}

ComplexVec WaveformGenerator::compute_spectrum(const ComplexVec& waveform) const {
    (void)waveform;
    throw std::logic_error(
        "WaveformGenerator::compute_spectrum is reserved for GPU/FFTW backend.");
}


Scalar WaveformGenerator::time_bandwidth_product() const {
    return params_.pulse_width_s * params_.bw_hz;
}


ComplexVec WaveformGenerator::generate_window(
    int length,
    WindowType type) const
{
    const auto weights = generate_window_weights(length, type);
    ComplexVec window(weights.size(), Complex(1.0, 0.0));
    for (std::size_t i = 0; i < weights.size(); ++i) {
        window[i] = Complex(weights[i], 0.0);
    }
    return window;
}

std::vector<Scalar> WaveformGenerator::generate_window_weights(
    int length,
    WindowType type) const
{
    if (length <= 0) {
        return {};
    }

    std::vector<Scalar> window(static_cast<std::size_t>(length), 1.0);
    if (length == 1) {
        return window;
    }

    for (int n = 0; n < length; ++n) {
        const Scalar x = static_cast<Scalar>(n) / static_cast<Scalar>(length - 1);

        if (type == WindowType::Hann) {
            window[static_cast<std::size_t>(n)] =
                0.5 * (1.0 - std::cos(2.0 * PI * x));
        } else if (type == WindowType::Hamming) {
            window[static_cast<std::size_t>(n)] =
                0.54 - 0.46 * std::cos(2.0 * PI * x);
        } else if (type == WindowType::Blackman) {
            window[static_cast<std::size_t>(n)] =
                0.42 - 0.5 * std::cos(2.0 * PI * x)
                     + 0.08 * std::cos(4.0 * PI * x);
        } else {
            window[static_cast<std::size_t>(n)] = 1.0;
        }
    }

    return window;
}

}  // namespace radar
