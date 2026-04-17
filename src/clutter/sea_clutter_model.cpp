/**
 * @file sea_clutter_model.cpp
 * @brief 海杂波双层建模生成器实现
 */

#include "clutter/sea_clutter_model.h"
#include "core/tools/math_utils.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include <fftw3.h>

namespace radar::clutter {

namespace {

constexpr uint64_t kMixConst1 = 0x9e3779b97f4a7c15ULL;
constexpr uint64_t kMixConst2 = 0xbf58476d1ce4e5b9ULL;
constexpr uint64_t kMixConst3 = 0x94d049bb133111ebULL;

constexpr Scalar kMorchinBaseCoeff = 4.0e-7;
constexpr Scalar kMorchinSeaStateRate = 0.6;
constexpr Scalar kMorchinBetaCoeff = 2.44 / 57.29;
constexpr Scalar kMorchinBetaPower = 1.08;
constexpr Scalar kMorchinRoughnessBase = 0.025;
constexpr Scalar kMorchinRoughnessCoeff = 0.046;
constexpr Scalar kMorchinRoughnessPower = 1.72;
constexpr Scalar kMorchinTransitionPower = 1.9;

Scalar compute_morchin_sigma0_linear(Scalar grazing_rad,
                                     Scalar wavelength_m,
                                     Scalar sea_state) {
    const Scalar phi = math::clamp_positive_eps(grazing_rad);
    const Scalar lambda_m = math::clamp_positive_eps(wavelength_m);
    const Scalar ss = math::clamp_nonnegative(sea_state);
    const Scalar ss_plus_one = ss + 1.0;

    const Scalar beta = kMorchinBetaCoeff * std::pow(ss_plus_one, kMorchinBetaPower);
    const Scalar tan_beta = std::max(std::tan(beta), EPSILON);
    const Scalar tan_beta_sq = tan_beta * tan_beta;
    const Scalar cot_beta_sq = 1.0 / tan_beta_sq;

    const Scalar h_e =
        kMorchinRoughnessBase + kMorchinRoughnessCoeff * std::pow(ss, kMorchinRoughnessPower);
    const Scalar phi_c_arg =
        std::clamp(lambda_m / std::max(EPSILON, 4.0 * PI * h_e), 0.0, 1.0);
    const Scalar phi_c = std::max(std::asin(phi_c_arg), EPSILON);

    const Scalar sigma0_c =
        (phi < phi_c) ? std::pow(phi / phi_c, kMorchinTransitionPower) : 1.0;

    const Scalar sin_phi = std::max(std::sin(phi), EPSILON);
    const Scalar cot_phi = std::cos(phi) / sin_phi;
    const Scalar diffuse_term =
        (kMorchinBaseCoeff *
         std::pow(10.0, kMorchinSeaStateRate * ss_plus_one) *
         sigma0_c * sin_phi) / lambda_m;
    const Scalar specular_term =
        cot_beta_sq * std::exp(-(cot_phi * cot_phi) / tan_beta_sq);

    return math::clamp_positive_eps(diffuse_term + specular_term);
}

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

    // 检查并更新杂波池
    if (params_.sequence_mode == SeaClutterSequenceMode::SequencePoolMode) {
        if (pool_needs_update(system)) {
            if (!initialize_pool(system)) {
                return false;
            }
        }
    }

    std::vector<DelayBinContribution> contributions;
    bool build_ok = false;
    if (params_.grid_mode == SeaClutterGridMode::RangeSampleGrid) {
        build_ok = build_range_sample_grid_contributions(
            system, antenna, beam_pointing, beam_index, contributions);
    } else {
        build_ok = build_physical_grid_contributions(
            system, antenna, beam_pointing, beam_index, range_min_m, range_max_m, contributions);
    }
    if (!build_ok) {
        return false;
    }

    synthesize_from_delay_contributions(
        contributions, tx_waveform, system.samples_per_pulse, out_clutter);

    return true;
}

bool SeaClutterModel::build_range_sample_grid_contributions(
    const RadarSystemParams& system,
    const AntennaModel& antenna,
    const BeamPoint& beam_pointing,
    int beam_index,
    std::vector<DelayBinContribution>& out_contributions) {
    out_contributions.clear();

    // 直接按距离采样单元划分，不再引入方位维
    const std::size_t num_samples = static_cast<std::size_t>(system.samples_per_pulse);
    const std::size_t num_pulses = static_cast<std::size_t>(system.pulses_per_cpi);

    Scalar range_min_m = -1.0;
    Scalar range_max_m = -1.0;
    std::string error;
    if (!resolve_ground_range(system, range_min_m, range_max_m, error)) {
        last_error_ = "build_range_sample_grid_contributions failed: " + error;
        return false;
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

    // 每个距离采样单元一条长度 Np 的慢时间序列，先算其平均功率幅度
    std::vector<Scalar> amplitude(num_samples, 0.0);

    for (std::size_t s = 0; s < num_samples; ++s) {
        const Scalar tau_s = tau_ref_s + static_cast<Scalar>(s) / system.fs_hz;
        const Scalar slant_range_m = tau_s * C * 0.5;
        if (slant_range_m < range_min_m || slant_range_m > range_max_m) {
            continue;
        }

        const Scalar ground_range_m = std::sqrt(slant_range_m * slant_range_m -
                                                 antenna_height_m * antenna_height_m);
        const Scalar grazing_rad = std::atan2(antenna_height_m,
                                              math::clamp_positive_eps(ground_range_m));
        const Scalar elevation_deg = -math::rad_to_deg(grazing_rad);
        const Scalar gain_linear = antenna.gain(beam_pointing.azimuth_deg, elevation_deg,
                                                 beam_pointing.azimuth_deg, beam_pointing.elevation_deg);
        const Scalar d_az_rad = math::deg_to_rad(antenna.beamwidth_3db_az_deg());
        const Scalar cell_area_m2 = math::clamp_nonnegative(ground_range_m) *
                                    math::clamp_positive_eps(range_bin_m) * d_az_rad;
        const Scalar sigma0_linear = morchin_sigma0_linear(grazing_rad, system.fc_hz);
        const Scalar sigma_cell = sigma0_linear * cell_area_m2;
        const Scalar numerator = tx_power_w * gain_linear * gain_linear *
                                 lambda_m * lambda_m * sigma_cell;
        const Scalar denominator = four_pi_cubed *
                                   std::pow(math::clamp_positive_eps(slant_range_m), 4.0) *
                                   system_loss_linear;
        const Scalar receive_power_w = math::clamp_nonnegative(
            math::safe_div(numerator, denominator));

        amplitude[s] = std::sqrt(receive_power_w);
    }

    out_contributions.reserve(num_samples);
    for (std::size_t s = 0; s < num_samples; ++s) {
        if (amplitude[s] <= 0.0) {
            continue;
        }

        ComplexVec sample_sequence;
        if (params_.sequence_mode == SeaClutterSequenceMode::SequencePoolMode) {
            sample_sequence = extract_from_pool(
                beam_index, static_cast<int>(s), 0, num_pulses);
        } else {
            sample_sequence = generate_sequence(num_pulses,
                                                beam_index,
                                                static_cast<int>(s),
                                                0,
                                                system.prf_hz,
                                                params_.doppler_center_hz,
                                                params_.doppler_sigma_hz,
                                                params_.k_shape_nu);
        }
        if (sample_sequence.empty()) {
            last_error_ =
                "SeaClutterModel::build_range_sample_grid_contributions failed: "
                "sample sequence generation failed.";
            return false;
        }

        DelayBinContribution contribution{};
        contribution.delay_bin = static_cast<int>(s);
        contribution.pulse_coeffs.assign(num_pulses, Complex(0.0, 0.0));
        for (std::size_t p = 0; p < num_pulses; ++p) {
            contribution.pulse_coeffs[p] = amplitude[s] * sample_sequence[p];
        }
        out_contributions.push_back(std::move(contribution));
    }

    return true;
}

bool SeaClutterModel::build_physical_grid_contributions(
    const RadarSystemParams& system,
    const AntennaModel& antenna,
    const BeamPoint& beam_pointing,
    int beam_index,
    Scalar ground_range_min_m,
    Scalar ground_range_max_m,
    std::vector<DelayBinContribution>& out_contributions) {
    out_contributions.clear();

    const Scalar antenna_height_m = math::clamp_nonnegative(system.antenna_height_m);
    const Scalar slant_ref_m =
        std::sqrt(antenna_height_m * antenna_height_m + ground_range_min_m * ground_range_min_m);
    const Scalar tau_ref_s = 2.0 * slant_ref_m / C;

    const std::vector<RangeRingInfo> range_rings = build_range_rings(
        system, antenna, beam_pointing, ground_range_min_m, ground_range_max_m, tau_ref_s);
    if (range_rings.empty()) {
        return true;
    }

    const std::size_t num_pulses = static_cast<std::size_t>(system.pulses_per_cpi);
    out_contributions.reserve(range_rings.size());
    for (const RangeRingInfo& range_ring : range_rings) {
        DelayBinContribution contribution{};
        contribution.delay_bin = range_ring.start_sample_index;
        contribution.pulse_coeffs.assign(num_pulses, Complex(0.0, 0.0));

        for (const AzCellInfo& az_cell : range_ring.az_cells) {
            ComplexVec cell_sequence;
            if (params_.sequence_mode == SeaClutterSequenceMode::InTimeMode) {
                cell_sequence = generate_sequence(num_pulses,
                                                  beam_index,
                                                  range_ring.range_index,
                                                  az_cell.az_index,
                                                  system.prf_hz,
                                                  params_.doppler_center_hz,
                                                  params_.doppler_sigma_hz,
                                                  params_.k_shape_nu);
            } else {
                cell_sequence = extract_from_pool(beam_index,
                                                  range_ring.range_index,
                                                  az_cell.az_index,
                                                  num_pulses);
            }

            if (cell_sequence.empty()) {
                last_error_ =
                    "SeaClutterModel::build_physical_grid_contributions failed: "
                    "cell sequence generation failed.";
                return false;
            }

            const Scalar amplitude = std::sqrt(math::clamp_nonnegative(az_cell.receive_power_w));
            if (amplitude <= 0.0) {
                continue;
            }

            for (std::size_t p = 0; p < num_pulses; ++p) {
                contribution.pulse_coeffs[p] += amplitude * cell_sequence[p];
            }
        }

        out_contributions.push_back(std::move(contribution));
    }

    return true;
}

void SeaClutterModel::synthesize_from_delay_contributions(
    const std::vector<DelayBinContribution>& contributions,
    const ComplexVec& tx_waveform,
    int samples_per_pulse,
    CpiEcho& out_clutter) const {
    for (const DelayBinContribution& contribution : contributions) {
        const int tx_begin = std::max(0, -contribution.delay_bin);
        const int tx_end =
            std::min(static_cast<int>(tx_waveform.size()), samples_per_pulse - contribution.delay_bin);
        if (tx_begin >= tx_end) {
            continue;
        }

        for (std::size_t p = 0; p < contribution.pulse_coeffs.size() &&
                                p < out_clutter.pulses.size(); ++p) {
            const Complex coeff = contribution.pulse_coeffs[p];
            auto& pulse = out_clutter.pulses[p];
            for (int m = tx_begin; m < tx_end; ++m) {
                const int sample_index = contribution.delay_bin + m;
                pulse[static_cast<std::size_t>(sample_index)] +=
                    coeff * tx_waveform[static_cast<std::size_t>(m)];
            }
        }
    }
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

    if (!math::is_finite_positive(params.az_grid_step_deg)) {
        error = "az_grid_step_deg must be positive and finite.";
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

    if (!std::isfinite(params.morchin_sea_state) || params.morchin_sea_state < 0.0) {
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
    return compute_morchin_sigma0_linear(grazing_rad,
                                         C / math::clamp_positive_eps(fc_hz),
                                         params_.morchin_sea_state);
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

// === PhysicalGrid 模式 ===
std::vector<SeaClutterModel::RangeRingInfo> SeaClutterModel::build_range_rings(
    const RadarSystemParams& system,
    const antenna::AntennaModel& antenna,
    const BeamPoint& beam_pointing,
    Scalar ground_range_min_m,
    Scalar ground_range_max_m,
    Scalar tau_ref_s) const {
    const Scalar antenna_height_m = math::clamp_nonnegative(system.antenna_height_m);

    // 使用天线 3dB 波束宽度和配置的方位网格步长
    const Scalar beam_az_width_deg = antenna.beamwidth_3db_az_deg();
    const Scalar d_az_rad = math::deg_to_rad(params_.az_grid_step_deg);
    const int az_count = std::max(
        1, static_cast<int>(std::llround(beam_az_width_deg / params_.az_grid_step_deg)));
    const Scalar az_start_deg =
        beam_pointing.azimuth_deg - 0.5 * beam_az_width_deg + 0.5 * params_.az_grid_step_deg;

    // 使用雷达系统的距离分辨率（range_resolution_m = C/(2*B)）
    const Scalar range_step_m = system.range_resolution_m;

    const Scalar tx_power_w = math::clamp_positive_eps(system.peak_power_w);
    const Scalar lambda_m = math::clamp_positive_eps(system.wavelength_m);
    const Scalar system_loss_linear = math::clamp_positive_eps(system.system_loss_linear);

    const int range_count_estimate = std::max(
        1, static_cast<int>(std::ceil((ground_range_max_m - ground_range_min_m) /
                                      math::clamp_positive_eps(range_step_m))));

    std::vector<RangeRingInfo> range_rings;
    range_rings.reserve(static_cast<std::size_t>(range_count_estimate));
    int range_index = 0;
    for (Scalar rg_m = ground_range_min_m + 0.5 * range_step_m; rg_m < ground_range_max_m;
         rg_m += range_step_m, ++range_index) {
        const Scalar grazing_rad =
            std::atan2(antenna_height_m, math::clamp_positive_eps(rg_m));
        const Scalar slant_range_m = std::sqrt(antenna_height_m * antenna_height_m + rg_m * rg_m);
        const Scalar cell_area_m2 =
            math::clamp_nonnegative(rg_m) * range_step_m * d_az_rad;
        const Scalar sigma0_linear = morchin_sigma0_linear(grazing_rad, system.fc_hz);
        const Scalar sigma_cell = sigma0_linear * cell_area_m2;
        const Scalar tau_s = 2.0 * slant_range_m / C;

        RangeRingInfo range_ring{};
        range_ring.range_index = range_index;
        range_ring.ground_range_m = rg_m;
        range_ring.elevation_deg = -math::rad_to_deg(grazing_rad);
        range_ring.slant_range_m = slant_range_m;
        range_ring.start_sample_index = static_cast<int>(
            std::llround((tau_s - tau_ref_s) * system.fs_hz));
        range_ring.az_cells.reserve(static_cast<std::size_t>(az_count));

        for (int az_index = 0; az_index < az_count; ++az_index) {
            const Scalar azimuth_deg =
                math::wrap_azimuth_deg(az_start_deg + static_cast<Scalar>(az_index) * params_.az_grid_step_deg);
            const Scalar gain_linear =
                antenna.gain(azimuth_deg,
                             range_ring.elevation_deg,
                             beam_pointing.azimuth_deg,
                             beam_pointing.elevation_deg);

            const Scalar numerator = tx_power_w * gain_linear * gain_linear * lambda_m * lambda_m * sigma_cell;
            const Scalar denominator =
                four_pi_cubed * std::pow(math::clamp_positive_eps(slant_range_m), 4.0) *
                system_loss_linear;
            const Scalar receive_power_w =
                math::clamp_nonnegative(math::safe_div(numerator, denominator));

            AzCellInfo az_cell{};
            az_cell.az_index = az_index;
            az_cell.azimuth_deg = azimuth_deg;
            az_cell.receive_power_w = receive_power_w;
            range_ring.az_cells.push_back(az_cell);
        }

        range_rings.push_back(std::move(range_ring));
    }

    return range_rings;
}

}  // namespace radar
