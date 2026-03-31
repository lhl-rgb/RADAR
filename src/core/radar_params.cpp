/**
 * @file radar_params.cpp
 * @brief 雷达参数定义与管理实现
 */

#include "core/radar_params.h"
#include "core/math_utils.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace radar {

namespace {

/**
 * @brief 极化类型转字符串
 * @param type 极化类型
 * @return 字符串描述
 */
const char* to_string(PolarizationType type) {
    switch (type) {
    case PolarizationType::HH:
        return "HH";
    case PolarizationType::VV:
        return "VV";
    }
    return "Unknown";
}

/**
 * @brief 波形类型转字符串
 * @param type 波形类型
 * @return 字符串描述
 */
const char* to_string(WaveformType type) {
    switch (type) {
    case WaveformType::LFM:
        return "LFM";
    case WaveformType::NLFM:
        return "NLFM";
    case WaveformType::PHASE_CODED:
        return "PHASE_CODED";
    case WaveformType::CW:
        return "CW";
    }
    return "Unknown";
}

/**
 * @brief 相位编码类型转字符串
 * @param type 相位编码类型
 * @return 字符串描述
 */
const char* to_string(PhaseCodeType type) {
    switch (type) {
    case PhaseCodeType::Barker2:
        return "Barker2";
    case PhaseCodeType::Barker3:
        return "Barker3";
    case PhaseCodeType::Barker4:
        return "Barker4";
    case PhaseCodeType::Barker5:
        return "Barker5";
    case PhaseCodeType::Barker7:
        return "Barker7";
    case PhaseCodeType::Barker11:
        return "Barker11";
    case PhaseCodeType::Barker13:
        return "Barker13";
    }
    return "Unknown";
}

/**
 * @brief 窗函数类型转字符串
 * @param type 窗函数类型
 * @return 字符串描述
 */
const char* to_string(WindowType type) {
    switch (type) {
    case WindowType::Rectangular:
        return "Rectangular";
    case WindowType::Hann:
        return "Hann";
    case WindowType::Hamming:
        return "Hamming";
    case WindowType::Blackman:
        return "Blackman";
    }
    return "Unknown";
}

/**
 * @brief 相控阵模型类型转字符串
 * @param type 相控阵模型类型
 * @return 字符串描述
 */
const char* to_string(PhasedArrayModelType type) {
    switch (type) {
    case PhasedArrayModelType::ULA_1D:
        return "ULA_1D";
    case PhasedArrayModelType::UPA_2D:
        return "UPA_2D";
    }
    return "Unknown";
}

/**
 * @brief 阵元加权类型转字符串
 * @param type 阵元加权类型
 * @return 字符串描述
 */
const char* to_string(AntennaWeightType type) {
    switch (type) {
    case AntennaWeightType::Uniform:
        return "Uniform";
    case AntennaWeightType::Hamming:
        return "Hamming";
    }
    return "Unknown";
}

/**
 * @brief 海杂波慢时间序列策略转字符串
 */
const char* to_string(SeaClutterSequenceMode mode) {
    switch (mode) {
    case SeaClutterSequenceMode::InTimeMode:
        return "InTimeMode";
    case SeaClutterSequenceMode::SequencePoolMode:
        return "SequencePoolMode";
    }
    return "Unknown";
}

}  // namespace

RadarParams::RadarParams()
    : waveform_type(WaveformType::LFM),
      phase_code_type(PhaseCodeType::Barker13),
      nlfm_window_type(WindowType::Hamming),
      polarization(PolarizationType::HH),
      fc_hz(10.0e9),
      bw_hz(20.0e6),
      pulse_width_s(20.0e-6),
      prf_hz(1600.0),
      peak_power_w(5000.0),
      fs_hz(40.0e6),
      pulses_per_cpi(32),
      samples_per_pulse(0),
      noise_figure_db(4.0),
      system_loss_db(6.0),
      antenna_config(),
      antenna_height_m(15.0),
      beam_table_path(),
      min_range_m(1000.0),
      max_range_m(120000.0),
      radar_location{0.0, 0.0, 0.0},
      sea_clutter(),
      target_params(),
      wavelength_m(0.0),
      range_resolution_m(0.0),
      velocity_resolution_mps(0.0),
      max_unambiguous_range_m(0.0),
      max_unambiguous_velocity(0.0),
      noise_figure_linear(0.0),
      system_loss_linear(0.0) {
    compute_derived_params();
}

/**
 * @brief 计算全部派生参数
 *
 * @details
 * 当前派生参数包括：
 * - 波长
 * - 距离分辨率
 * - 速度分辨率
 * - 最大非模糊距离
 * - 最大非模糊速度
 * - 噪声系数线性值
 * - 系统损耗线性值
 * - 每脉冲采样点数
 *
 * 其中 samples_per_pulse 的定义采用快时间窗方式：
 * 从最小作用距离对应回波开始，到最大作用距离对应回波结束，
 * 再额外保留一个脉冲宽度，避免脉冲尾部被截断。
 */
void RadarParams::compute_derived_params() {
    const Scalar safe_fc = math::clamp_positive_eps(fc_hz);
    const Scalar safe_bw = math::clamp_positive_eps(bw_hz);
    const Scalar safe_prf = math::clamp_positive_eps(prf_hz);
    const Scalar safe_fs = math::clamp_positive_eps(fs_hz);
    const Scalar safe_pulses_per_cpi =
        std::max<Scalar>(static_cast<Scalar>(pulses_per_cpi), 1.0);

    wavelength_m = C / safe_fc;
    range_resolution_m = C / (2.0 * safe_bw);
    velocity_resolution_mps = wavelength_m * safe_prf / (2.0 * safe_pulses_per_cpi);
    max_unambiguous_range_m = C / (2.0 * safe_prf);
    max_unambiguous_velocity = wavelength_m * safe_prf / 4.0;

    noise_figure_linear = math::db_to_linear(noise_figure_db);
    system_loss_linear = math::db_to_linear(system_loss_db);

    const Scalar range_span_m = math::clamp_nonnegative(max_range_m - min_range_m);
    const Scalar fast_time_window_s =
        (2.0 * range_span_m / C) + math::clamp_nonnegative(pulse_width_s);

    samples_per_pulse =
        std::max(1, static_cast<int>(std::ceil(fast_time_window_s * safe_fs)));
}

bool RadarParams::load_from_json(const std::string& filepath) {
    (void)filepath;
    std::cerr << "[RadarParams] JSON import is disabled in current stage.\n";
    return false;
}

bool RadarParams::save_to_json(const std::string& filepath) const {
    (void)filepath;
    std::cerr << "[RadarParams] JSON export is disabled in current stage.\n";
    return false;
}

/**
 * @brief 参数有效性检查
 * @return 参数合法返回 true，否则返回 false
 *
 * @details
 * 当前检查项包括：
 * 1. 发射与接收基础参数；
 * 2. 场景距离配置；
 * 3. 脉冲宽度与 PRF 一致性；
 * 4. 复基带采样建议条件；
 * 5. 相控阵配置合法性。
 */
bool RadarParams::validate() const {
    const auto is_vec3_finite = [](const Vec3& v) {
        return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
    };

    if (fc_hz <= 0.0 || bw_hz <= 0.0 || pulse_width_s <= 0.0 ||
        prf_hz <= 0.0 || peak_power_w <= 0.0) {
        return false;
    }

    if (fs_hz <= 0.0 || pulses_per_cpi <= 0) {
        return false;
    }

    if (noise_figure_db < 0.0 || system_loss_db < 0.0) {
        return false;
    }

    if (antenna_height_m < 0.0) {
        return false;
    }

    if (min_range_m < 0.0 || max_range_m <= min_range_m) {
        return false;
    }

    /**
     * @brief 脉冲宽度不得超过 PRI
     */
    if (pulse_width_s > 1.0 / prf_hz) {
        return false;
    }

    /**
     * @brief 复基带建模建议条件
     * @details
     * 若采用复包络建模，通常至少要求 fs >= bw。
     */
    if (fs_hz < bw_hz) {
        return false;
    }

    /**
     * @brief 相控阵配置检查
     */
    if (antenna_config.num_elements_az <= 0) {
        return false;
    }
    if (antenna_config.spacing_az_lambda <= 0.0) {
        return false;
    }
    if (antenna_config.peak_gain_db < 0.0) {
        return false;
    }

    if (antenna_config.model_type == PhasedArrayModelType::UPA_2D) {
        if (antenna_config.num_elements_el <= 0) {
            return false;
        }
        if (antenna_config.spacing_el_lambda <= 0.0) {
            return false;
        }
    }

    /**
     * @brief 海杂波配置检查
     */
    if (!math::is_finite(sea_clutter.ground_range_min_m) ||
        !math::is_finite(sea_clutter.ground_range_max_m) ||
        !math::is_finite(sea_clutter.range_step_m) ||
        !math::is_finite(sea_clutter.beam_az_width_deg) ||
        !math::is_finite(sea_clutter.az_step_deg) ||
        !math::is_finite(sea_clutter.k_shape_nu) ||
        !math::is_finite(sea_clutter.doppler_center_hz) ||
        !math::is_finite(sea_clutter.doppler_sigma_hz) ||
        !math::is_finite(sea_clutter.morchin.a0_db) ||
        !math::is_finite(sea_clutter.morchin.a_g) ||
        !math::is_finite(sea_clutter.morchin.a_f) ||
        !math::is_finite(sea_clutter.morchin.a_s) ||
        !math::is_finite(sea_clutter.morchin.sea_state) ||
        !math::is_finite(sea_clutter.morchin.sin_psi_floor)) {
        return false;
    }

    if (sea_clutter.range_step_m <= 0.0 ||
        sea_clutter.beam_az_width_deg <= 0.0 ||
        sea_clutter.az_step_deg <= 0.0 ||
        sea_clutter.k_shape_nu <= 0.0 ||
        sea_clutter.doppler_sigma_hz <= 0.0 ||
        sea_clutter.pool_length_factor <= 0 ||
        sea_clutter.morchin.sin_psi_floor <= 0.0) {
        return false;
    }

    const Scalar clutter_nyquist_hz = 0.5 * prf_hz;
    if (sea_clutter.doppler_center_hz < -clutter_nyquist_hz ||
        sea_clutter.doppler_center_hz >= clutter_nyquist_hz) {
        return false;
    }

    const Scalar clutter_range_min =
        (sea_clutter.ground_range_min_m < 0.0) ? min_range_m : sea_clutter.ground_range_min_m;
    const Scalar clutter_range_max =
        (sea_clutter.ground_range_max_m < 0.0) ? max_range_m : sea_clutter.ground_range_max_m;

    if (clutter_range_min < 0.0 || clutter_range_max <= clutter_range_min) {
        return false;
    }

    if (!math::is_finite(target_params.beam_gate_threshold_db)) {
        return false;
    }

    return samples_per_pulse > 0;
}

/**
 * @brief 打印参数摘要
 */
void RadarParams::print() const {
    std::cout
        << "RadarParams:\n"
        << "  waveform_type: " << to_string(waveform_type) << "\n"
        << "  phase_code_type: " << to_string(phase_code_type) << "\n"
        << "  nlfm_window_type: " << to_string(nlfm_window_type) << "\n"
        << "  polarization: " << to_string(polarization) << "\n"
        << "  fc_hz: " << fc_hz << "\n"
        << "  bw_hz: " << bw_hz << "\n"
        << "  pulse_width_s: " << pulse_width_s << "\n"
        << "  prf_hz: " << prf_hz << "\n"
        << "  peak_power_w: " << peak_power_w << "\n"
        << "  fs_hz: " << fs_hz << "\n"
        << "  pulses_per_cpi: " << pulses_per_cpi << "\n"
        << "  samples_per_pulse: " << samples_per_pulse << "\n"
        << "  noise_figure_db: " << noise_figure_db << "\n"
        << "  system_loss_db: " << system_loss_db << "\n"
        << "  antenna_model_type: " << to_string(antenna_config.model_type) << "\n"
        << "  antenna_num_elements_az: " << antenna_config.num_elements_az << "\n"
        << "  antenna_num_elements_el: " << antenna_config.num_elements_el << "\n"
        << "  antenna_spacing_az_lambda: " << antenna_config.spacing_az_lambda << "\n"
        << "  antenna_spacing_el_lambda: " << antenna_config.spacing_el_lambda << "\n"
        << "  antenna_weight_type_az: " << to_string(antenna_config.weight_type_az) << "\n"
        << "  antenna_weight_type_el: " << to_string(antenna_config.weight_type_el) << "\n"
        << "  antenna_peak_gain_db: " << antenna_config.peak_gain_db << "\n"
        << "  antenna_height_m: " << antenna_height_m << "\n"
        << "  beam_table_path: " << beam_table_path << "\n"
        << "  min_range_m: " << min_range_m << "\n"
        << "  max_range_m: " << max_range_m << "\n"
        << "  radar_location: (" << radar_location.latitude << ", "
        << radar_location.longitude << ", " << radar_location.altitude << ")\n"
        << "  sea_clutter.enabled: " << (sea_clutter.enabled ? "true" : "false") << "\n"
        << "  sea_clutter.ground_range_min_m: " << sea_clutter.ground_range_min_m << "\n"
        << "  sea_clutter.ground_range_max_m: " << sea_clutter.ground_range_max_m << "\n"
        << "  sea_clutter.range_step_m: " << sea_clutter.range_step_m << "\n"
        << "  sea_clutter.beam_az_width_deg: " << sea_clutter.beam_az_width_deg << "\n"
        << "  sea_clutter.az_step_deg: " << sea_clutter.az_step_deg << "\n"
        << "  sea_clutter.k_shape_nu: " << sea_clutter.k_shape_nu << "\n"
        << "  sea_clutter.doppler_center_hz: " << sea_clutter.doppler_center_hz << "\n"
        << "  sea_clutter.doppler_sigma_hz: " << sea_clutter.doppler_sigma_hz << "\n"
        << "  sea_clutter.sequence_mode: " << to_string(sea_clutter.sequence_mode) << "\n"
        << "  sea_clutter.seed: " << sea_clutter.seed << "\n"
        << "  sea_clutter.pool_length_factor: " << sea_clutter.pool_length_factor << "\n"
        << "  sea_clutter.morchin.a0_db: " << sea_clutter.morchin.a0_db << "\n"
        << "  sea_clutter.morchin.a_g: " << sea_clutter.morchin.a_g << "\n"
        << "  sea_clutter.morchin.a_f: " << sea_clutter.morchin.a_f << "\n"
        << "  sea_clutter.morchin.a_s: " << sea_clutter.morchin.a_s << "\n"
        << "  sea_clutter.morchin.sea_state: " << sea_clutter.morchin.sea_state << "\n"
        << "  sea_clutter.morchin.sin_psi_floor: " << sea_clutter.morchin.sin_psi_floor << "\n"
        << "  target_params.enabled: " << (target_params.enabled ? "true" : "false") << "\n"
        << "  target_params.enable_beam_gain: " << (target_params.enable_beam_gain ? "true" : "false") << "\n"
        << "  target_params.enable_two_way_propagation_loss: "
        << (target_params.enable_two_way_propagation_loss ? "true" : "false") << "\n"
        << "  target_params.enable_phase: " << (target_params.enable_phase ? "true" : "false") << "\n"
        << "  target_params.enable_swerling: " << (target_params.enable_swerling ? "true" : "false") << "\n"
        << "  target_params.seed: " << target_params.seed << "\n"
        << "  target_params.beam_gate_threshold_db: " << target_params.beam_gate_threshold_db << "\n"
        << "  target_params.skip_out_of_beam_targets: "
        << (target_params.skip_out_of_beam_targets ? "true" : "false") << "\n"
        << "  wavelength_m: " << wavelength_m << "\n"
        << "  range_resolution_m: " << range_resolution_m << "\n"
        << "  velocity_resolution_mps: " << velocity_resolution_mps << "\n"
        << "  max_unambiguous_range_m: " << max_unambiguous_range_m << "\n"
        << "  max_unambiguous_velocity: " << max_unambiguous_velocity << "\n"
        << "  noise_figure_linear: " << noise_figure_linear << "\n"
        << "  system_loss_linear: " << system_loss_linear << "\n";
}

void RadarParamManager::add_config(const std::string& name, const RadarParams& params) {
    configs_[name] = params;

    /**
     * @brief 若当前尚未激活任何配置，则默认激活第一个加入的配置
     */
    if (active_name_.empty()) {
        set_active(name);
    }
}

std::optional<RadarParams> RadarParamManager::get_config(const std::string& name) const {
    const auto it = configs_.find(name);
    if (it == configs_.end()) {
        return std::nullopt;
    }
    return it->second;
}

void RadarParamManager::set_active(const std::string& name) {
    const auto it = configs_.find(name);
    if (it == configs_.end()) {
        throw std::invalid_argument("RadarParamManager::set_active failed: unknown config '" +
                                    name + "'");
    }

    active_name_ = name;
    active_params_ = it->second;
}

const RadarParams& RadarParamManager::get_active() const {
    if (active_name_.empty()) {
        throw std::runtime_error(
            "RadarParamManager::get_active failed: no active config has been selected.");
    }
    return active_params_;
}

}  // namespace radar
