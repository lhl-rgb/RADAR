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

namespace radar {

namespace {

constexpr uint64_t kMixConst1 = 0x9e3779b97f4a7c15ULL;
constexpr uint64_t kMixConst2 = 0xbf58476d1ce4e5b9ULL;
constexpr uint64_t kMixConst3 = 0x94d049bb133111ebULL;

/**
 * @brief 频域塑形后的逆变换器 (线程本地缓存 FFTW 计划)
 * @details
 * 这里保持极小职责：只负责 inverse DFT（IFFT），但减少InTimeMode每个单元重复建销计划。
 */
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

SeaClutterModel::SeaClutterModel(const SeaClutterParams& params) {
    set_params(params);
}

bool SeaClutterModel::set_params(const SeaClutterParams& params) {
    std::string error;
    if (!validate_params(params, error)) {
        last_error_ = error;
        return false;
    }

    params_ = params;
    last_error_.clear();
    return true;
}

bool SeaClutterModel::generate_cpi(const RadarParams& radar_params,
                                   const PhasedArrayAntenna& antenna,
                                   const AzEl& beam_pointing,
                                   int beam_index,
                                   const ComplexVec& tx_waveform,
                                   CpiEcho& out_clutter) {
    last_error_.clear();

    if (radar_params.pulses_per_cpi <= 0 || radar_params.samples_per_pulse <= 0) {
        last_error_ =
            "SeaClutterModel::generate_cpi failed: pulses_per_cpi and samples_per_pulse must be "
            "positive.";
        return false;
    }
    if (tx_waveform.empty()) {
        last_error_ = "SeaClutterModel::generate_cpi failed: tx_waveform must not be empty.";
        return false;
    }
    if (!math::is_finite_positive(radar_params.fs_hz)) {
        last_error_ = "SeaClutterModel::generate_cpi failed: radar fs_hz must be positive.";
        return false;
    }
    if (!math::is_finite_positive(radar_params.prf_hz)) {
        last_error_ = "SeaClutterModel::generate_cpi failed: radar prf_hz must be positive.";
        return false;
    }
    std::string nyquist_error;
    if (!validate_doppler_center_nyquist(params_.doppler_center_hz, radar_params.prf_hz,
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
    if (!resolve_ground_range(radar_params, range_min_m, range_max_m, param_error)) {
        last_error_ = "SeaClutterModel::generate_cpi failed: " + param_error;
        return false;
    }

    out_clutter.beam_index = beam_index;
    out_clutter.azimuth_deg = beam_pointing.azimuth;
    out_clutter.elevation_deg = beam_pointing.elevation;
    out_clutter.pulses.assign(
        static_cast<std::size_t>(radar_params.pulses_per_cpi),
        PulseEcho(static_cast<std::size_t>(radar_params.samples_per_pulse), Complex(0.0, 0.0)));

    if (!params_.enabled) {
        return true;
    }

    const Scalar antenna_height_m = math::clamp_nonnegative(radar_params.antenna_height_m);
    const Scalar slant_ref_m =
        std::sqrt(antenna_height_m * antenna_height_m + range_min_m * range_min_m);
    const Scalar tau_ref_s = 2.0 * slant_ref_m / C;

    const std::vector<CellInfo> cells =
        build_cells(radar_params, antenna, beam_pointing, range_min_m, range_max_m, tau_ref_s);
    if (cells.empty()) {
        return true;
    }

    ComplexVec sequence_pool;
    if (params_.sequence_mode == SeaClutterSequenceMode::SequencePoolMode) {
        // pool 模式：先生成一个长序列，单元只做不同起点截取，降低重复生成成本。
        const std::size_t pool_length = std::max<std::size_t>(
            static_cast<std::size_t>(radar_params.pulses_per_cpi),
            static_cast<std::size_t>(params_.pool_length_factor) *
                static_cast<std::size_t>(radar_params.pulses_per_cpi));
        sequence_pool = generate_sequence_pool(pool_length, beam_index, radar_params.prf_hz,
                                            params_.doppler_center_hz, params_.doppler_sigma_hz,
                                            params_.k_shape_nu);
        if (sequence_pool.empty()) {
            last_error_ = "SeaClutterModel::generate_cpi failed: sequence pool build failed.";
            return false;
        }
    }

    for (const CellInfo& cell : cells) {
        ComplexVec cell_sequence;

        if (params_.sequence_mode == SeaClutterSequenceMode::InTimeMode) {
            // 每个单元独立种子，保证并行和回归场景下严格可复现。
            cell_sequence =
                generate_sequence(static_cast<std::size_t>(radar_params.pulses_per_cpi),
                                         beam_index, cell.range_index, cell.az_index,
                                         radar_params.prf_hz, params_.doppler_center_hz,
                                         params_.doppler_sigma_hz, params_.k_shape_nu);
        } else {
            // pool 模式下仍用单元 seed 推导 start 和初相，避免不同单元完全同相。
            const uint64_t seed =
                cell_seed(params_.seed, beam_index, cell.range_index, cell.az_index);
            const std::size_t start =
                static_cast<std::size_t>(seed % static_cast<uint64_t>(sequence_pool.size()));
            const Scalar phase = 2.0 * PI *
                                 (static_cast<Scalar>(mix_u64(seed ^ kMixConst1) &
                                                      std::numeric_limits<uint32_t>::max()) /
                                  static_cast<Scalar>(std::numeric_limits<uint32_t>::max()));
            const Complex phase_rotator = std::polar(1.0, phase);

            cell_sequence.assign(static_cast<std::size_t>(radar_params.pulses_per_cpi),
                                 Complex(0.0, 0.0));
            for (std::size_t p = 0; p < cell_sequence.size(); ++p) {
                const std::size_t idx = (start + p) % sequence_pool.size();
                cell_sequence[p] = sequence_pool[idx] * phase_rotator;
            }
            normalize(cell_sequence);
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
                    sample_index >= static_cast<int>(radar_params.samples_per_pulse)) {
                    continue;
                }
                pulse[static_cast<std::size_t>(sample_index)] += coeff * tx_waveform[m];
            }
        }
    }

    return true;
}

bool SeaClutterModel::validate_params(const SeaClutterParams& params, std::string& error) {
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

    if (!std::isfinite(params.morchin.a0_db) || !std::isfinite(params.morchin.a_g) ||
        !std::isfinite(params.morchin.a_f) || !std::isfinite(params.morchin.a_s) ||
        !std::isfinite(params.morchin.sea_state) ||
        !math::is_finite_positive(params.morchin.sin_psi_floor)) {
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
    // SplitMix64 风格混洗：把相关索引展开为高熵位分布。
    value += kMixConst1;
    value = (value ^ (value >> 30U)) * kMixConst2;
    value = (value ^ (value >> 27U)) * kMixConst3;
    return value ^ (value >> 31U);
}

uint64_t SeaClutterModel::cell_seed(uint64_t base_seed,
                                    int beam_index,
                                    int range_index,
                                    int az_index) {
    // 说明：
    // 1) 保留 base_seed 作为外部可控随机入口；
    // 2) 叠加 beam/range/az 确保单元间解耦；
    // 3) 返回值稳定，可直接用于复现实验与回归测试。
    uint64_t seed = mix_u64(base_seed);
    seed ^= mix_u64(static_cast<uint64_t>(static_cast<uint32_t>(beam_index)) + kMixConst1);
    seed ^= mix_u64(static_cast<uint64_t>(static_cast<uint32_t>(range_index)) + kMixConst2);
    seed ^= mix_u64(static_cast<uint64_t>(static_cast<uint32_t>(az_index)) + kMixConst3);
    return mix_u64(seed);
}
// 计算地距上下限，支持 -1 跟随 RadarParams。
bool SeaClutterModel::resolve_ground_range(const RadarParams& radar_params,
                                           Scalar& out_ground_range_min_m,
                                           Scalar& out_ground_range_max_m,
                                           std::string& error) const {
    out_ground_range_min_m =
        (params_.ground_range_min_m < 0.0) ? radar_params.min_range_m : params_.ground_range_min_m;
    out_ground_range_max_m =
        (params_.ground_range_max_m < 0.0) ? radar_params.max_range_m : params_.ground_range_max_m;
    if (out_ground_range_min_m < 0.0 ||
        out_ground_range_max_m <= out_ground_range_min_m) {
        error = "resolved ground range bounds are invalid.";
        return false;
    }
    return true;
}

Scalar SeaClutterModel::morchin_sigma0_linear(Scalar grazing_rad, Scalar fc_hz) const {
    const Scalar sin_psi = std::max(std::sin(math::clamp_nonnegative(grazing_rad)),
                                    params_.morchin.sin_psi_floor);
    const Scalar fc_ghz = math::clamp_positive_eps(fc_hz * 1.0e-9);
    const Scalar sigma0_db = params_.morchin.a0_db +
                             params_.morchin.a_g * std::log10(sin_psi) +
                             params_.morchin.a_f * std::log10(fc_ghz) +
                             params_.morchin.a_s * params_.morchin.sea_state;
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
        // 频点映射到 [-PRF/2, PRF/2) 频轴，按高斯 PSD 赋权。
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
    // deterministic 模式的核心：单元索引 -> 固定 RNG 种子。
    std::mt19937_64 rng(cell_seed(params_.seed, beam_index, range_index, az_index));
    const ComplexVec gaussian =
        generate_corr_sequence(length, prf_hz, doppler_center_hz, doppler_sigma_hz, rng);
    return apply_k_sirp(gaussian, k_shape_nu, rng);
}

ComplexVec SeaClutterModel::generate_sequence_pool(std::size_t pool_length,
                                                int beam_index,
                                                Scalar prf_hz,
                                                Scalar doppler_center_hz,
                                                Scalar doppler_sigma_hz,
                                                Scalar k_shape_nu) const {
    // pool 只按 beam 级别播种，单元差异通过截取起点与相位体现。
    const uint64_t seed = mix_u64(params_.seed ^ static_cast<uint64_t>(beam_index) ^ kMixConst1);
    std::mt19937_64 rng(seed);
    const ComplexVec gaussian = generate_corr_sequence(
        pool_length, prf_hz, doppler_center_hz, doppler_sigma_hz, rng);
    return apply_k_sirp(gaussian, k_shape_nu, rng);
}

std::vector<SeaClutterModel::CellInfo> SeaClutterModel::build_cells(
    const RadarParams& radar_params,
    const PhasedArrayAntenna& antenna,
    const AzEl& beam_pointing,
    Scalar ground_range_min_m,
    Scalar ground_range_max_m,
    Scalar tau_ref_s) const {
    const Scalar antenna_height_m = math::clamp_nonnegative(radar_params.antenna_height_m);
    const Scalar d_az_rad = math::deg_to_rad(params_.az_step_deg);//方位步进对应的弧度
    const int az_count = std::max(
        1, static_cast<int>(std::llround(params_.beam_az_width_deg / params_.az_step_deg)));//方位向单元数量
    const Scalar az_start_deg =
        beam_pointing.azimuth - 0.5 * params_.beam_az_width_deg + 0.5 * params_.az_step_deg;

    const Scalar tx_power_w = math::clamp_positive_eps(radar_params.peak_power_w);
    const Scalar lambda_m = math::clamp_positive_eps(radar_params.wavelength_m);
    const Scalar system_loss_linear = math::clamp_positive_eps(radar_params.system_loss_linear);
    const Scalar four_pi_cubed = std::pow(4.0 * PI, 3.0);

    std::vector<CellInfo> cells;
    int range_index = 0;
    for (Scalar rg_m = ground_range_min_m + 0.5 * params_.range_step_m; rg_m < ground_range_max_m;
         rg_m += params_.range_step_m, ++range_index) {
        // 地距网格：单元中心点采样。
        const Scalar grazing_rad =
            std::atan2(antenna_height_m, math::clamp_positive_eps(rg_m));
        const Scalar slant_range_m = std::sqrt(antenna_height_m * antenna_height_m + rg_m * rg_m);
        const Scalar cell_area_m2 =
            math::clamp_nonnegative(rg_m) * params_.range_step_m * d_az_rad;
        const Scalar sigma0_linear = morchin_sigma0_linear(grazing_rad, radar_params.fc_hz);
        const Scalar sigma_cell = sigma0_linear * cell_area_m2;

        for (int az_index = 0; az_index < az_count; ++az_index) {
            const Scalar azimuth_deg =
                math::wrap_azimuth_deg(az_start_deg + static_cast<Scalar>(az_index) * params_.az_step_deg);
            const Scalar elevation_deg = -math::rad_to_deg(grazing_rad);
            const Scalar gain_linear =
                antenna.gain(azimuth_deg, elevation_deg, beam_pointing.azimuth, beam_pointing.elevation);

            const Scalar numerator = tx_power_w * gain_linear * gain_linear * lambda_m * lambda_m * sigma_cell;
            const Scalar denominator =
                four_pi_cubed * std::pow(math::clamp_positive_eps(slant_range_m), 4.0) *
                system_loss_linear;
            const Scalar receive_power_w =
                math::clamp_nonnegative(math::safe_div(numerator, denominator));

            // 当前实现使用整数采样映射（nearest）；分数时延不在本次版本范围。
            const Scalar tau_s = 2.0 * slant_range_m / C;
            const int start_sample_index = static_cast<int>(
                std::llround((tau_s - tau_ref_s) * radar_params.fs_hz));//tau_ref_s 对应参考距离的时延，单元时延相对于参考的偏移转换为采样点偏移

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

 
