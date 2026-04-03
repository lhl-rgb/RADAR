/**
 * @file sea_clutter_model.cpp
 * @brief 海杂波双层建模生成器实现
 */

#include "clutter/sea_clutter_model.h"
#include "core/math_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <fftw3.h>

namespace radar::clutter {

namespace {

constexpr uint64_t kMixConst1 = 0x9e3779b97f4a7c15ULL;
constexpr uint64_t kMixConst2 = 0xbf58476d1ce4e5b9ULL;
constexpr uint64_t kMixConst3 = 0x94d049bb133111ebULL;

class GaussianDopplerFftSynthesizer {
public:
    static bool inverse_dft(const ComplexVec& spectrum, ComplexVec& out_sequence) {
        const std::size_t n = spectrum.size();
        if (n == 0) {
            out_sequence.clear();
            return true;
        }

        static thread_local ThreadLocalFft context;
        return context.inverse_dft(spectrum, out_sequence);
    }

private:
    struct ThreadLocalFft {
        std::size_t n = 0;
        fftw_complex* input = nullptr;
        fftw_complex* output = nullptr;
        fftw_plan plan = nullptr;

        ~ThreadLocalFft() {
            cleanup();
        }

        bool ensure_plan(std::size_t size) {
            if (size == 0) {
                return false;
            }
            if (size == n && plan != nullptr) {
                return true;
            }

            cleanup();

            input = fftw_alloc_complex(static_cast<int>(size));
            output = fftw_alloc_complex(static_cast<int>(size));
            if (!input || !output) {
                cleanup();
                return false;
            }

            plan = fftw_plan_dft_1d(static_cast<int>(size), input, output,
                                     FFTW_BACKWARD, FFTW_MEASURE);
            if (plan == nullptr) {
                cleanup();
                return false;
            }

            n = size;
            return true;
        }

        void cleanup() {
            if (plan != nullptr) {
                fftw_destroy_plan(plan);
                plan = nullptr;
            }
            if (input != nullptr) {
                fftw_free(input);
                input = nullptr;
            }
            if (output != nullptr) {
                fftw_free(output);
                output = nullptr;
            }
            n = 0;
        }

        bool inverse_dft(const ComplexVec& spectrum, ComplexVec& out_sequence) {
            const std::size_t size = spectrum.size();
            if (!ensure_plan(size)) {
                return false;
            }

            for (std::size_t i = 0; i < size; ++i) {
                input[i][0] = spectrum[i].real();
                input[i][1] = spectrum[i].imag();
            }

            fftw_execute(plan);

            out_sequence.assign(size, Complex(0.0, 0.0));
            const Scalar inv_n = 1.0 / static_cast<Scalar>(size);
            for (std::size_t i = 0; i < size; ++i) {
                out_sequence[i] = Complex(output[i][0], output[i][1]) * inv_n;
            }
            return true;
        }
    };
};

}  // namespace

SeaClutterModel::SeaClutterModel(const clutter::SeaClutterConfig& params) {
    set_params(params);
}

bool SeaClutterModel::set_params(const clutter::SeaClutterConfig& params) {
    std::string error;
    if (!validate_params(params, error)) {
        last_error_ = error;
        return false;
    }

    params_ = params;
    // 参数变化时清除缓存
    pool_cache_.valid = false;
    last_error_.clear();
    return true;
}

bool SeaClutterModel::initialize_pool(const RadarSystemParams& system, int num_beams) {
    if (params_.sequence_mode != SeaClutterSequenceMode::SequencePoolMode) {
        // 非池模式不需要初始化
        return true;
    }

    if (system.pulses_per_cpi <= 0) {
        last_error_ = "initialize_pool failed: pulses_per_cpi must be positive.";
        return false;
    }
    if (!math::is_finite_positive(system.prf_hz)) {
        last_error_ = "initialize_pool failed: prf_hz must be positive.";
        return false;
    }

    // 计算池大小：考虑所有CPI的总脉冲数
    const std::size_t pool_length = static_cast<std::size_t>(params_.pool_length_factor) *
                                    static_cast<std::size_t>(system.pulses_per_cpi);

    SPDLOG_INFO("[SeaClutter] Initializing sequence pool: length={}, pulses_per_cpi={}",
                pool_length, system.pulses_per_cpi);

    // 生成杂波池
    pool_cache_.sequence = generate_sequence_pool(pool_length,
                                                   system.prf_hz,
                                                   params_.doppler_center_hz,
                                                   params_.doppler_sigma_hz,
                                                   params_.k_shape_nu);

    if (pool_cache_.sequence.empty()) {
        last_error_ = "initialize_pool failed: sequence generation failed.";
        pool_cache_.valid = false;
        return false;
    }

    // 记录生成参数
    pool_cache_.prf_hz = system.prf_hz;
    pool_cache_.doppler_center_hz = params_.doppler_center_hz;
    pool_cache_.doppler_sigma_hz = params_.doppler_sigma_hz;
    pool_cache_.k_shape_nu = params_.k_shape_nu;
    pool_cache_.pulses_per_cpi = system.pulses_per_cpi;
    pool_cache_.valid = true;

    SPDLOG_INFO("[SeaClutter] Sequence pool initialized successfully, size={}",
                pool_cache_.sequence.size());
    return true;
}

bool SeaClutterModel::pool_needs_update(const RadarSystemParams& system) const {
    if (!pool_cache_.valid) {
        return true;
    }
    // 检查关键参数是否变化
    return pool_cache_.prf_hz != system.prf_hz ||
           pool_cache_.doppler_center_hz != params_.doppler_center_hz ||
           pool_cache_.doppler_sigma_hz != params_.doppler_sigma_hz ||
           pool_cache_.k_shape_nu != params_.k_shape_nu;
}

ComplexVec SeaClutterModel::extract_from_pool(int beam_index, int range_index, int az_index,
                                               std::size_t length) const {
    if (!pool_cache_.valid || pool_cache_.sequence.empty()) {
        return {};
    }

    // 根据 (beam, range, az) 生成唯一种子，确定截取起点
    const uint64_t seed = cell_seed(params_.seed, beam_index, range_index, az_index);
    const std::size_t start = static_cast<std::size_t>(seed % static_cast<uint64_t>(pool_cache_.sequence.size()));

    // 截取序列（序列本身已包含多普勒特性，无需额外相位旋转）
    ComplexVec sequence(length, Complex(0.0, 0.0));
    for (std::size_t p = 0; p < length; ++p) {
        const std::size_t idx = (start + p) % pool_cache_.sequence.size();
        sequence[p] = pool_cache_.sequence[idx];
    }

    // 归一化
    normalize(sequence);
    return sequence;
}

bool SeaClutterModel::generate_cpi(const RadarSystemParams& system,
                                   const antenna::AntennaModel& antenna,
                                   const BeamPoint& beam_pointing,
                                   int beam_index,
                                   const ComplexVec& tx_waveform,
                                   CpiEcho& out_clutter) {
    last_error_.clear();

    if (system.pulses_per_cpi <= 0 || system.samples_per_pulse <= 0) {
        last_error_ =
            "SeaClutterModel::generate_cpi failed: pulses_per_cpi and samples_per_pulse must be "
            "positive.";
        return false;
    }
    if (tx_waveform.empty()) {
        last_error_ = "SeaClutterModel::generate_cpi failed: tx_waveform must not be empty.";
        return false;
    }
    if (!math::is_finite_positive(system.fs_hz)) {
        last_error_ = "SeaClutterModel::generate_cpi failed: radar fs_hz must be positive.";
        return false;
    }
    if (!math::is_finite_positive(system.prf_hz)) {
        last_error_ = "SeaClutterModel::generate_cpi failed: radar prf_hz must be positive.";
        return false;
    }
    std::string nyquist_error;
    if (!validate_doppler_center_nyquist(params_.doppler_center_hz, system.prf_hz,
                                         nyquist_error)) {
        last_error_ = "SeaClutterModel::generate_cpi failed: " + nyquist_error;
        return false;
    }

    std::string param_error;
    if (!validate_params(params_, param_error)) {
        last_error_ = "SeaClutterModel::generate_cpi failed: " + param_error;
        return false;
    }

    Scalar range_min_m = 0.0;
    Scalar range_max_m = 0.0;
    if (!resolve_ground_range(system, range_min_m, range_max_m, param_error)) {
        last_error_ = "SeaClutterModel::generate_cpi failed: " + param_error;
        return false;
    }

    out_clutter.beam_index = beam_index;
    out_clutter.azimuth_deg = beam_pointing.azimuth_deg;
    out_clutter.elevation_deg = beam_pointing.elevation_deg;
    out_clutter.pulses.assign(
        static_cast<std::size_t>(system.pulses_per_cpi),
        PulseEcho(static_cast<std::size_t>(system.samples_per_pulse), Complex(0.0, 0.0)));

    if (!params_.enabled) {
        return true;
    }

    // 根据网格模式选择生成方法
    if (params_.grid_mode == SeaClutterGridMode::SampleGrid) {
        return generate_cpi_sample_grid(system, antenna, beam_pointing, beam_index,
                                        tx_waveform, out_clutter);
    }

    // === PhysicalGrid 模式 ===
    const Scalar antenna_height_m = math::clamp_nonnegative(system.antenna_height_m);
    const Scalar slant_ref_m =
        std::sqrt(antenna_height_m * antenna_height_m + range_min_m * range_min_m);
    const Scalar tau_ref_s = 2.0 * slant_ref_m / C;

    const std::vector<CellInfo> cells =
        build_cells(system, antenna, beam_pointing, range_min_m, range_max_m, tau_ref_s);
    if (cells.empty()) {
        return true;
    }

    // 检查并更新杂波池（如果需要）
    if (params_.sequence_mode == SeaClutterSequenceMode::SequencePoolMode) {
        if (pool_needs_update(system)) {
            if (!initialize_pool(system)) {
                return false;
            }
        }
    }

    for (const CellInfo& cell : cells) {
        ComplexVec cell_sequence;

        if (params_.sequence_mode == SeaClutterSequenceMode::InTimeMode) {
            cell_sequence =
                generate_sequence(static_cast<std::size_t>(system.pulses_per_cpi),
                                         beam_index, cell.range_index, cell.az_index,
                                         system.prf_hz, params_.doppler_center_hz,
                                         params_.doppler_sigma_hz, params_.k_shape_nu);
        } else {
            // 从预生成的杂波池中截取
            cell_sequence = extract_from_pool(beam_index, cell.range_index, cell.az_index,
                                               static_cast<std::size_t>(system.pulses_per_cpi));
        }

        if (cell_sequence.empty()) {
            last_error_ = "SeaClutterModel::generate_cpi failed: cell sequence generation failed.";
            return false;
        }

        const Scalar amplitude = std::sqrt(math::clamp_nonnegative(cell.receive_power_w));
        for (std::size_t p = 0; p < cell_sequence.size(); ++p) {
            const Complex coeff = amplitude * cell_sequence[p];
            auto& pulse = out_clutter.pulses[p];

            for (std::size_t m = 0; m < tx_waveform.size(); ++m) {
                const int sample_index = cell.start_sample_index + static_cast<int>(m);
                if (sample_index < 0 ||
                    sample_index >= static_cast<int>(system.samples_per_pulse)) {
                    continue;
                }
                pulse[static_cast<std::size_t>(sample_index)] += coeff * tx_waveform[m];
            }
        }
    }

    return true;
}

bool SeaClutterModel::generate_cpi_sample_grid(const RadarSystemParams& system,
                                                 const AntennaModel& antenna,
                                                 const BeamPoint& beam_pointing,
                                                 int beam_index,
                                                 const ComplexVec& tx_waveform,
                                                 CpiEcho& out_clutter) {
    // 简化模式：直接按采样点划分，不考虑方位维
    const std::size_t num_samples = static_cast<std::size_t>(system.samples_per_pulse);
    const std::size_t num_pulses = static_cast<std::size_t>(system.pulses_per_cpi);

    // 获取距离范围
    Scalar range_min_m = 0.0;
    Scalar range_max_m = 0.0;
    std::string error;
    if (!resolve_ground_range(system, range_min_m, range_max_m, error)) {
        last_error_ = "generate_cpi_sample_grid failed: " + error;
        return false;
    }

    // 确保杂波池已初始化
    if (params_.sequence_mode == SeaClutterSequenceMode::SequencePoolMode) {
        if (pool_needs_update(system)) {
            if (!initialize_pool(system)) {
                return false;
            }
        }
    }

    const Scalar antenna_height_m = math::clamp_nonnegative(system.antenna_height_m);
    const Scalar slant_ref_m =
        std::sqrt(antenna_height_m * antenna_height_m + range_min_m * range_min_m);
    const Scalar tau_ref_s = 2.0 * slant_ref_m / C;
    const Scalar range_bin_m = system.range_bin_size_m;

    // 预计算常量
    const Scalar tx_power_w = math::clamp_positive_eps(system.peak_power_w);
    const Scalar lambda_m = math::clamp_positive_eps(system.wavelength_m);
    const Scalar system_loss_linear = math::clamp_positive_eps(system.system_loss_linear);
    const Scalar four_pi_cubed = std::pow(4.0 * PI, 3.0);

    // 生成每个采样点的幅度调制序列 (1 × num_samples)
    std::vector<Scalar> amplitude(num_samples, 0.0);

    for (std::size_t s = 0; s < num_samples; ++s) {
        // 计算该采样点对应的距离
        const Scalar tau_s = tau_ref_s + static_cast<Scalar>(s) / system.fs_hz;
        const Scalar slant_range_m = tau_s * C * 0.5;

        // 只处理有效范围内的点
        if (slant_range_m < range_min_m || slant_range_m > range_max_m) {
            continue;
        }

        // 计算地距和掠射角
        const Scalar ground_range_m = std::sqrt(slant_range_m * slant_range_m -
                                                 antenna_height_m * antenna_height_m);
        const Scalar grazing_rad = std::atan2(antenna_height_m,
                                              math::clamp_positive_eps(ground_range_m));

        // 计算天线增益（固定方位，使用波束指向）
        const Scalar elevation_deg = -math::rad_to_deg(grazing_rad);
        const Scalar gain_linear = antenna.gain(beam_pointing.azimuth_deg, elevation_deg,
                                                 beam_pointing.azimuth_deg, beam_pointing.elevation_deg);

        // 计算散射面积（假设方位宽度为波束宽度）
        const Scalar d_az_rad = math::deg_to_rad(params_.beam_az_width_deg);
        const Scalar cell_area_m2 = math::clamp_nonnegative(ground_range_m) *
                                    math::clamp_positive_eps(range_bin_m) * d_az_rad;

        // 计算σ⁰
        const Scalar sigma0_linear = morchin_sigma0_linear(grazing_rad, system.fc_hz);
        const Scalar sigma_cell = sigma0_linear * cell_area_m2;

        // 雷达方程计算接收功率
        const Scalar numerator = tx_power_w * gain_linear * gain_linear *
                                 lambda_m * lambda_m * sigma_cell;
        const Scalar denominator = four_pi_cubed *
                                   std::pow(math::clamp_positive_eps(slant_range_m), 4.0) *
                                   system_loss_linear;
        const Scalar receive_power_w = math::clamp_nonnegative(
            math::safe_div(numerator, denominator));

        amplitude[s] = std::sqrt(receive_power_w);
    }

    // 为每个脉冲生成随机序列并乘以幅度
    std::mt19937_64 rng(params_.seed ^ static_cast<uint64_t>(beam_index));

    for (std::size_t p = 0; p < num_pulses; ++p) {
        auto& pulse = out_clutter.pulses[p];

        // 生成该脉冲的随机序列
        ComplexVec random_seq(num_samples, Complex(0.0, 0.0));

        if (params_.sequence_mode == SeaClutterSequenceMode::SequencePoolMode) {
            // 从池中截取
            const uint64_t pulse_seed = mix_u64(static_cast<uint64_t>(p) ^ params_.seed ^ kMixConst1);
            const std::size_t start = static_cast<std::size_t>(pulse_seed % pool_cache_.sequence.size());

            for (std::size_t s = 0; s < num_samples; ++s) {
                const std::size_t idx = (start + s) % pool_cache_.sequence.size();
                random_seq[s] = pool_cache_.sequence[idx];
            }
        } else {
            // 实时生成
            random_seq = generate_corr_sequence(num_samples, system.prf_hz,
                                                params_.doppler_center_hz,
                                                params_.doppler_sigma_hz, rng);
            // 应用K分布
            random_seq = apply_k_sirp(random_seq, params_.k_shape_nu, rng);
        }

        // 归一化随机序列
        normalize(random_seq);

        // 幅度调制 + 波形卷积
        for (std::size_t s = 0; s < num_samples; ++s) {
            if (amplitude[s] <= 0.0) continue;

            const Complex coeff = amplitude[s] * random_seq[s];

            // 与发射波形卷积
            for (std::size_t m = 0; m < tx_waveform.size(); ++m) {
                const int sample_index = static_cast<int>(s) + static_cast<int>(m);
                if (sample_index < 0 || sample_index >= static_cast<int>(num_samples)) {
                    continue;
                }
                pulse[static_cast<std::size_t>(sample_index)] += coeff * tx_waveform[m];
            }
        }
    }

    return true;
}

bool SeaClutterModel::validate_params(const clutter::SeaClutterConfig& params, std::string& error) {
    error.clear();
    if (params.ground_range_min_m < 0.0 && params.ground_range_min_m != -1.0) {
        error = "ground_range_min_m must be -1 or >= 0.";
        return false;
    }
    if (params.ground_range_max_m < 0.0 && params.ground_range_max_m != -1.0) {
        error = "ground_range_max_m must be -1 or >= 0.";
        return false;
    }
    if (params.ground_range_min_m >= 0.0 && params.ground_range_max_m >= 0.0 &&
        params.ground_range_max_m <= params.ground_range_min_m) {
        error = "ground_range_max_m must be greater than ground_range_min_m.";
        return false;
    }

    if (!math::is_finite_positive(params.range_step_m)) {
        error = "range_step_m must be positive and finite.";
        return false;
    }
    if (!math::is_finite_positive(params.beam_az_width_deg)) {
        error = "beam_az_width_deg must be positive and finite.";
        return false;
    }
    if (!math::is_finite_positive(params.az_step_deg)) {
        error = "az_step_deg must be positive and finite.";
        return false;
    }
    if (!math::is_finite_positive(params.k_shape_nu)) {
        error = "k_shape_nu must be positive and finite.";
        return false;
    }
    if (!math::is_finite_positive(params.doppler_sigma_hz)) {
        error = "doppler_sigma_hz must be positive and finite.";
        return false;
    }
    if (!math::is_finite(params.doppler_center_hz)) {
        error = "doppler_center_hz must be finite.";
        return false;
    }
    if (params.pool_length_factor <= 0) {
        error = "pool_length_factor must be positive.";
        return false;
    }

    if (!std::isfinite(params.morchin_a0_db) || !std::isfinite(params.morchin_a_g) ||
        !std::isfinite(params.morchin_a_f) || !std::isfinite(params.morchin_a_s) ||
        !std::isfinite(params.morchin_sea_state) ||
        !math::is_finite_positive(params.morchin_sin_psi_floor)) {
        error = "Morchin params are invalid.";
        return false;
    }

    return true;
}

bool SeaClutterModel::validate_doppler_center_nyquist(Scalar doppler_center_hz,
                                                       Scalar prf_hz,
                                                       std::string& error) {
    error.clear();
    if (!math::is_finite_positive(prf_hz) || !math::is_finite(doppler_center_hz)) {
        error = "doppler center/prf must be finite with positive prf.";
        return false;
    }

    const Scalar nyquist_hz = 0.5 * prf_hz;
    if (doppler_center_hz < -nyquist_hz || doppler_center_hz >= nyquist_hz) {
        error = "doppler_center_hz must be in [-prf_hz/2, prf_hz/2).";
        return false;
    }
    return true;
}

uint64_t SeaClutterModel::mix_u64(uint64_t value) {
    value += kMixConst1;
    value = (value ^ (value >> 30U)) * kMixConst2;
    value = (value ^ (value >> 27U)) * kMixConst3;
    return value ^ (value >> 31U);
}

uint64_t SeaClutterModel::cell_seed(uint64_t base_seed,
                                    int beam_index,
                                    int range_index,
                                    int az_index) {
    uint64_t seed = mix_u64(base_seed);
    seed ^= mix_u64(static_cast<uint64_t>(static_cast<uint32_t>(beam_index)) + kMixConst1);
    seed ^= mix_u64(static_cast<uint64_t>(static_cast<uint32_t>(range_index)) + kMixConst2);
    seed ^= mix_u64(static_cast<uint64_t>(static_cast<uint32_t>(az_index)) + kMixConst3);
    return mix_u64(seed);
}

bool SeaClutterModel::resolve_ground_range(const RadarSystemParams& system,
                                           Scalar& out_ground_range_min_m,
                                           Scalar& out_ground_range_max_m,
                                           std::string& error) const {
    out_ground_range_min_m =
        (params_.ground_range_min_m < 0.0) ? system.min_range_m : params_.ground_range_min_m;
    out_ground_range_max_m =
        (params_.ground_range_max_m < 0.0) ? system.max_range_m : params_.ground_range_max_m;
    if (out_ground_range_min_m < 0.0 ||
        out_ground_range_max_m <= out_ground_range_min_m) {
        error = "resolved ground range bounds are invalid.";
        return false;
    }
    return true;
}

Scalar SeaClutterModel::morchin_sigma0_linear(Scalar grazing_rad, Scalar fc_hz) const {
    const Scalar sin_psi = std::max(std::sin(math::clamp_nonnegative(grazing_rad)),
                                    params_.morchin_sin_psi_floor);
    const Scalar fc_ghz = math::clamp_positive_eps(fc_hz * 1.0e-9);
    const Scalar sigma0_db = params_.morchin_a0_db +
                             params_.morchin_a_g * std::log10(sin_psi) +
                             params_.morchin_a_f * std::log10(fc_ghz) +
                             params_.morchin_a_s * params_.morchin_sea_state;
    return math::clamp_positive_eps(math::db_to_linear(sigma0_db));
}

Complex SeaClutterModel::sample_complex_gaussian(std::mt19937_64& rng) const {
    std::normal_distribution<Scalar> dist(0.0, 1.0);
    const Scalar inv_sqrt2 = std::sqrt(0.5);
    return Complex(inv_sqrt2 * dist(rng), inv_sqrt2 * dist(rng));
}

ComplexVec SeaClutterModel::generate_corr_sequence(std::size_t length,
                                                            Scalar prf_hz,
                                                            Scalar doppler_center_hz,
                                                            Scalar doppler_sigma_hz,
                                                            std::mt19937_64& rng) const {
    if (length == 0) {
        return {};
    }

    ComplexVec spectrum(length, Complex(0.0, 0.0));
    const Scalar prf = math::clamp_positive_eps(prf_hz);
    const Scalar sigma = math::clamp_positive_eps(doppler_sigma_hz);

    for (std::size_t k = 0; k < length; ++k) {
        const int index = (k <= length / 2)
                                        ? static_cast<int>(k)
                                        : static_cast<int>(k) - static_cast<int>(length);
        const Scalar freq_hz = index * prf / static_cast<Scalar>(length);
        const Scalar z_score = (freq_hz - doppler_center_hz) / sigma;
        const Scalar psd_weight = std::exp(-0.5 * z_score * z_score);
        spectrum[k] = std::sqrt(math::clamp_nonnegative(psd_weight)) *
                      sample_complex_gaussian(rng);
    }

    ComplexVec sequence;
    if (!GaussianDopplerFftSynthesizer::inverse_dft(spectrum, sequence)) {
        return {};
    }

    normalize(sequence);
    return sequence;
}

ComplexVec SeaClutterModel::apply_k_sirp(const ComplexVec& base_sequence,
                                         Scalar k_shape_nu,
                                         std::mt19937_64& rng) const {
    if (base_sequence.empty()) {
        return {};
    }

    ComplexVec output(base_sequence.size(), Complex(0.0, 0.0));
    std::gamma_distribution<Scalar> gamma_dist(k_shape_nu, 1.0 / k_shape_nu);

    for (std::size_t i = 0; i < base_sequence.size(); ++i) {
        const Scalar tau = math::clamp_positive_eps(gamma_dist(rng));
        output[i] = std::sqrt(tau) * base_sequence[i];
    }

    normalize(output);
    return output;
}

void SeaClutterModel::normalize(ComplexVec& sequence) {
    if (sequence.empty()) {
        return;
    }

    Scalar power = 0.0;
    for (const Complex& x : sequence) {
        power += std::norm(x);
    }
    power /= static_cast<Scalar>(sequence.size());
    if (power <= EPSILON) {
        return;
    }

    const Scalar scale = 1.0 / std::sqrt(power);
    for (Complex& x : sequence) {
        x *= scale;
    }
}

ComplexVec SeaClutterModel::generate_sequence(std::size_t length,
                                                     int beam_index,
                                                     int range_index,
                                                     int az_index,
                                                     Scalar prf_hz,
                                                     Scalar doppler_center_hz,
                                                     Scalar doppler_sigma_hz,
                                                     Scalar k_shape_nu) const {
    std::mt19937_64 rng(cell_seed(params_.seed, beam_index, range_index, az_index));
    const ComplexVec gaussian =
        generate_corr_sequence(length, prf_hz, doppler_center_hz, doppler_sigma_hz, rng);
    return apply_k_sirp(gaussian, k_shape_nu, rng);
}

ComplexVec SeaClutterModel::generate_sequence_pool(std::size_t pool_length,
                                                Scalar prf_hz,
                                                Scalar doppler_center_hz,
                                                Scalar doppler_sigma_hz,
                                                Scalar k_shape_nu) const {
    // 使用配置种子生成全局杂波池
    const uint64_t seed = mix_u64(params_.seed ^ kMixConst1);
    std::mt19937_64 rng(seed);
    const ComplexVec gaussian = generate_corr_sequence(
        pool_length, prf_hz, doppler_center_hz, doppler_sigma_hz, rng);
    return apply_k_sirp(gaussian, k_shape_nu, rng);
}

std::vector<SeaClutterModel::CellInfo> SeaClutterModel::build_cells(
    const RadarSystemParams& system,
    const antenna::AntennaModel& antenna,
    const BeamPoint& beam_pointing,
    Scalar ground_range_min_m,
    Scalar ground_range_max_m,
    Scalar tau_ref_s) const {
    const Scalar antenna_height_m = math::clamp_nonnegative(system.antenna_height_m);
    const Scalar d_az_rad = math::deg_to_rad(params_.az_step_deg);
    const int az_count = std::max(
        1, static_cast<int>(std::llround(params_.beam_az_width_deg / params_.az_step_deg)));
    const Scalar az_start_deg =
        beam_pointing.azimuth_deg - 0.5 * params_.beam_az_width_deg + 0.5 * params_.az_step_deg;

    const Scalar tx_power_w = math::clamp_positive_eps(system.peak_power_w);
    const Scalar lambda_m = math::clamp_positive_eps(system.wavelength_m);
    const Scalar system_loss_linear = math::clamp_positive_eps(system.system_loss_linear);
    const Scalar four_pi_cubed = std::pow(4.0 * PI, 3.0);

    std::vector<CellInfo> cells;
    int range_index = 0;
    for (Scalar rg_m = ground_range_min_m + 0.5 * params_.range_step_m; rg_m < ground_range_max_m;
         rg_m += params_.range_step_m, ++range_index) {
        const Scalar grazing_rad =
            std::atan2(antenna_height_m, math::clamp_positive_eps(rg_m));
        const Scalar slant_range_m = std::sqrt(antenna_height_m * antenna_height_m + rg_m * rg_m);
        const Scalar cell_area_m2 =
            math::clamp_nonnegative(rg_m) * params_.range_step_m * d_az_rad;
        const Scalar sigma0_linear = morchin_sigma0_linear(grazing_rad, system.fc_hz);
        const Scalar sigma_cell = sigma0_linear * cell_area_m2;

        for (int az_index = 0; az_index < az_count; ++az_index) {
            const Scalar azimuth_deg =
                math::wrap_azimuth_deg(az_start_deg + static_cast<Scalar>(az_index) * params_.az_step_deg);
            const Scalar elevation_deg = -math::rad_to_deg(grazing_rad);
            const Scalar gain_linear =
                antenna.gain(azimuth_deg, elevation_deg, beam_pointing.azimuth_deg, beam_pointing.elevation_deg);

            const Scalar numerator = tx_power_w * gain_linear * gain_linear * lambda_m * lambda_m * sigma_cell;
            const Scalar denominator =
                four_pi_cubed * std::pow(math::clamp_positive_eps(slant_range_m), 4.0) *
                system_loss_linear;
            const Scalar receive_power_w =
                math::clamp_nonnegative(math::safe_div(numerator, denominator));

            const Scalar tau_s = 2.0 * slant_range_m / C;
            const int start_sample_index = static_cast<int>(
                std::llround((tau_s - tau_ref_s) * system.fs_hz));

            CellInfo cell{};
            cell.range_index = range_index;
            cell.az_index = az_index;
            cell.ground_range_m = rg_m;
            cell.azimuth_deg = azimuth_deg;
            cell.elevation_deg = elevation_deg;
            cell.slant_range_m = slant_range_m;
            cell.receive_power_w = receive_power_w;
            cell.start_sample_index = start_sample_index;
            cells.push_back(cell);
        }
    }

    return cells;
}

}  // namespace radar