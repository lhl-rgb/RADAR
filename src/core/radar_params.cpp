/**
 * @file radar_params.cpp
 * @brief 雷达参数定义与管理实现
 */

#include "core/radar_params.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace radar {

namespace {

/**
 * @brief dB 转线性值
 * @param db_value dB 值
 * @return 线性值
 */
Scalar db_to_linear(Scalar db_value) {
    return std::pow(10.0, db_value / 10.0);
}

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
      wavelength_m(0.0),
      range_resolution_m(0.0),
      velocity_resolution_mps(0.0),
      max_unambiguous_range_m(0.0),
      max_velocity(0.0),
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
    const Scalar safe_fc = std::max(fc_hz, EPSILON);
    const Scalar safe_bw = std::max(bw_hz, EPSILON);
    const Scalar safe_prf = std::max(prf_hz, EPSILON);
    const Scalar safe_fs = std::max(fs_hz, EPSILON);
    const Scalar safe_pulses_per_cpi =
        std::max<Scalar>(static_cast<Scalar>(pulses_per_cpi), 1.0);

    wavelength_m = C / safe_fc;
    range_resolution_m = C / (2.0 * safe_bw);
    velocity_resolution_mps = wavelength_m * safe_prf / (2.0 * safe_pulses_per_cpi);
    max_unambiguous_range_m = C / (2.0 * safe_prf);
    max_velocity = wavelength_m * safe_prf / 4.0;

    noise_figure_linear = db_to_linear(noise_figure_db);
    system_loss_linear = db_to_linear(system_loss_db);

    const Scalar range_span_m = std::max<Scalar>(0.0, max_range_m - min_range_m);
    const Scalar fast_time_window_s =
        (2.0 * range_span_m / C) + std::max<Scalar>(pulse_width_s, 0.0);

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
        << "  wavelength_m: " << wavelength_m << "\n"
        << "  range_resolution_m: " << range_resolution_m << "\n"
        << "  velocity_resolution_mps: " << velocity_resolution_mps << "\n"
        << "  max_unambiguous_range_m: " << max_unambiguous_range_m << "\n"
        << "  max_velocity: " << max_velocity << "\n"
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