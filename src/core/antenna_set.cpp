/**
 * @file antenna_set.cpp
 * @brief 相控阵天线与波位扫描模型实现
 */

#include "core/antenna_set.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace radar {

namespace {

/**
 * @brief dB 转线性值
 * @param db_value dB 数值
 * @return 线性值
 */
Scalar db_to_linear(Scalar db_value) {
    return std::pow(10.0, db_value / 10.0);
}

/**
 * @brief 线性值转 dB
 * @param linear_value 线性值
 * @return dB 数值
 */
Scalar linear_to_db(Scalar linear_value) {
    return 10.0 * std::log10(std::max(linear_value, EPSILON));
}

/**
 * @brief 去除字符串首尾空白字符
 * @param value 输入字符串
 * @return 去除首尾空白后的字符串
 */
std::string trim(const std::string& value) {
    const std::size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }

    const std::size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

/**
 * @brief 严格解析浮点数
 * @param text 输入文本
 * @param output 输出数值
 * @return 若解析成功且完整消费整个字符串则返回 true
 */
bool parse_double_strict(const std::string& text, Scalar& output) {
    std::size_t consumed = 0;
    try {
        output = std::stod(text, &consumed);
    } catch (...) {
        return false;
    }
    return consumed == text.size();
}

/**
 * @brief 空波位表异常信息
 */
constexpr const char* kEmptyBeamTableError =
    "AntennaScanModel: beam table is empty. Load or set beam table before gain query.";

}  // namespace

PhasedArrayAntenna::PhasedArrayAntenna(const PhasedArrayAntennaConfig& config) {
    set_config(config);
}

/**
 * @brief 设置相控阵天线配置参数
 * @param config 相控阵配置参数
 *
 * @details
 * 该接口会对关键参数做基本保护：
 * - 阵元数至少为 1
 * - 阵元间距至少为 EPSILON
 * 然后刷新内部缓存，包括：
 * - 峰值增益线性值
 * - 方位维加权向量
 * - 俯仰维加权向量
 */
void PhasedArrayAntenna::set_config(const PhasedArrayAntennaConfig& config) {
    config_ = config;

    config_.num_elements_az = std::max(config_.num_elements_az, 1);
    config_.num_elements_el = std::max(config_.num_elements_el, 1);
    config_.spacing_az_lambda = std::max(config_.spacing_az_lambda, EPSILON);
    config_.spacing_el_lambda = std::max(config_.spacing_el_lambda, EPSILON);

    refresh_internal_cache();
}

/**
 * @brief 更新内部缓存参数
 *
 * @details
 * 当前缓存包括：
 * 1. 峰值增益线性值；
 * 2. 方位维归一化加权向量；
 * 3. 俯仰维归一化加权向量。
 *
 * 加权向量满足元素和为 1，因此当目标方向与波束指向完全一致时，
 * 阵因子幅度为 1，归一化功率为 1。
 */
void PhasedArrayAntenna::refresh_internal_cache() {
    peak_gain_linear_ = std::max(db_to_linear(config_.peak_gain_db), EPSILON);
    weights_az_ = make_weights(config_.num_elements_az, config_.weight_type_az);
    weights_el_ = make_weights(config_.num_elements_el, config_.weight_type_el);
}

/**
 * @brief 计算指定目标方向在指定波位下的天线增益（线性）
 * @param target_az_deg 目标方位角（度）
 * @param target_el_deg 目标俯仰角（度）
 * @param beam_az_deg 当前波位方位角（度）
 * @param beam_el_deg 当前波位俯仰角（度）
 * @return 天线增益（线性）
 *
 * @details
 * 当前模型总增益表达式为：
 * G = G_peak * P_norm
 *
 * 其中：
 * - G_peak 为峰值增益（线性）
 * - P_norm 为归一化功率响应
 */
Scalar PhasedArrayAntenna::gain(Scalar target_az_deg, Scalar target_el_deg,
                                Scalar beam_az_deg, Scalar beam_el_deg) const {
    const Scalar p_norm =
        normalized_power(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg);
    return peak_gain_linear_ * p_norm;
}

/**
 * @brief 计算指定目标方向在指定波位下的天线增益（dB）
 * @param target_az_deg 目标方位角（度）
 * @param target_el_deg 目标俯仰角（度）
 * @param beam_az_deg 当前波位方位角（度）
 * @param beam_el_deg 当前波位俯仰角（度）
 * @return 天线增益（dB）
 */
Scalar PhasedArrayAntenna::gain_db(Scalar target_az_deg, Scalar target_el_deg,
                                   Scalar beam_az_deg, Scalar beam_el_deg) const {
    return linear_to_db(gain(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg));
}

/**
 * @brief 计算指定目标方向在指定波位下的归一化功率响应
 * @param target_az_deg 目标方位角（度）
 * @param target_el_deg 目标俯仰角（度）
 * @param beam_az_deg 当前波位方位角（度）
 * @param beam_el_deg 当前波位俯仰角（度）
 * @return 归一化功率响应，理论范围 [0, 1]
 *
 * @details
 * 根据配置参数中的相控阵模型类型，分别调用：
 * - ULA_1D 对应一维相扫线阵模型；
 * - UPA_2D 对应二维相扫平面阵模型。
 */
Scalar PhasedArrayAntenna::normalized_power(Scalar target_az_deg, Scalar target_el_deg,
                                            Scalar beam_az_deg, Scalar beam_el_deg) const {
    if (config_.model_type == PhasedArrayModelType::ULA_1D) {
        return normalized_power_ula(target_az_deg, beam_az_deg);
    }

    return normalized_power_upa(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg);
}

/**
 * @brief 判断目标是否位于当前波束主响应范围内
 * @param target_az_deg 目标方位角（度）
 * @param target_el_deg 目标俯仰角（度）
 * @param beam_az_deg 当前波位方位角（度）
 * @param beam_el_deg 当前波位俯仰角（度）
 * @param threshold_db 相对峰值阈值（dB）
 * @return 若归一化功率高于阈值则返回 true
 *
 * @details
 * 判定依据：
 * P_norm >= 10^(threshold_db / 10)
 *
 * 例如 threshold_db = -3 dB 时，对应功率阈值约为 0.5。
 */
bool PhasedArrayAntenna::is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                                           Scalar beam_az_deg, Scalar beam_el_deg,
                                           Scalar threshold_db) const {
    const Scalar p_norm =
        normalized_power(target_az_deg, target_el_deg, beam_az_deg, beam_el_deg);
    const Scalar threshold_linear = db_to_linear(threshold_db);
    return p_norm >= threshold_linear;
}

/**
 * @brief 计算 ULA 模型下的归一化功率响应
 * @param target_az_deg 目标方位角（度）
 * @param beam_az_deg 当前波位方位角（度）
 * @return 归一化功率响应
 *
 * @details
 * 一维相扫均匀线阵模型：
 *
 * psi = 2*pi*d_lambda*(sin(theta)-sin(theta0))
 *
 * AF = sum_{n=0}^{N-1} w_n * exp(j*n*psi)
 *
 * P_norm = |AF|^2
 *
 * 其中：
 * - theta 为目标方位角
 * - theta0 为当前波位方位角
 * - d_lambda 为阵元间距（单位：波长）
 * - w_n 为归一化加权系数
 *
 * 当前实现默认阵元方向图各向同性，因此仅计算阵因子响应。
 */
Scalar PhasedArrayAntenna::normalized_power_ula(Scalar target_az_deg,
                                                Scalar beam_az_deg) const {
    const Scalar theta = target_az_deg * PI / 180.0;
    const Scalar theta0 = beam_az_deg * PI / 180.0;

    const Scalar psi =
        2.0 * PI * config_.spacing_az_lambda * (std::sin(theta) - std::sin(theta0));

    std::complex<Scalar> af(0.0, 0.0);
    for (int n = 0; n < config_.num_elements_az; ++n) {
        const Scalar phase = static_cast<Scalar>(n) * psi;
        af += weights_az_[static_cast<std::size_t>(n)] *
              std::complex<Scalar>(std::cos(phase), std::sin(phase));
    }

    return std::clamp(std::norm(af), 0.0, 1.0);
}

/**
 * @brief 计算 UPA 模型下的归一化功率响应
 * @param target_az_deg 目标方位角（度）
 * @param target_el_deg 目标俯仰角（度）
 * @param beam_az_deg 当前波位方位角（度）
 * @param beam_el_deg 当前波位俯仰角（度）
 * @return 归一化功率响应
 *
 * @details
 * 二维相扫均匀平面阵模型采用方向余弦表示：
 *
 * u  = cos(el) * sin(az)
 * v  = sin(el)
 *
 * u0 = cos(el0) * sin(az0)
 * v0 = sin(el0)
 *
 * psix = 2*pi*dx_lambda*(u-u0)
 * psiy = 2*pi*dy_lambda*(v-v0)
 *
 * AF = sum_{m=0}^{Mx-1} sum_{n=0}^{My-1}
 *      wx(m)*wy(n)*exp(j*(m*psix+n*psiy))
 *
 * P_norm = |AF|^2
 *
 * 其中：
 * - az/el 为目标方向
 * - az0/el0 为波束扫描方向
 * - dx_lambda / dy_lambda 为两个维度阵元间距（单位：波长）
 *
 * 当前实现默认阵元方向图各向同性，因此总方向图仅由阵因子决定。
 */
Scalar PhasedArrayAntenna::normalized_power_upa(Scalar target_az_deg, Scalar target_el_deg,
                                                Scalar beam_az_deg, Scalar beam_el_deg) const {
    const Scalar az = target_az_deg * PI / 180.0;
    const Scalar el = target_el_deg * PI / 180.0;
    const Scalar az0 = beam_az_deg * PI / 180.0;
    const Scalar el0 = beam_el_deg * PI / 180.0;

    const Scalar u = std::cos(el) * std::sin(az);
    const Scalar v = std::sin(el);
    const Scalar u0 = std::cos(el0) * std::sin(az0);
    const Scalar v0 = std::sin(el0);

    const Scalar psix = 2.0 * PI * config_.spacing_az_lambda * (u - u0);
    const Scalar psiy = 2.0 * PI * config_.spacing_el_lambda * (v - v0);

    std::complex<Scalar> af(0.0, 0.0);
    for (int m = 0; m < config_.num_elements_az; ++m) {
        for (int n = 0; n < config_.num_elements_el; ++n) {
            const Scalar phase =
                static_cast<Scalar>(m) * psix + static_cast<Scalar>(n) * psiy;

            af += weights_az_[static_cast<std::size_t>(m)] *
                  weights_el_[static_cast<std::size_t>(n)] *
                  std::complex<Scalar>(std::cos(phase), std::sin(phase));
        }
    }

    return std::clamp(std::norm(af), 0.0, 1.0);
}

/**
 * @brief 生成指定长度的加权向量
 * @param length 向量长度
 * @param weight_type 加权类型
 * @return 归一化后的加权向量
 *
 * @details
 * 当前支持：
 * - Uniform：均匀加权
 * - Hamming：Hamming 加权
 *
 * 输出向量经过归一化处理，满足所有元素之和为 1。
 * 这样在波束中心方向上，阵因子幅度可直接归一化为 1。
 */
std::vector<Scalar> PhasedArrayAntenna::make_weights(int length,
                                                     AntennaWeightType weight_type) {
    length = std::max(length, 1);

    std::vector<Scalar> weights(static_cast<std::size_t>(length), 1.0);

    if (weight_type == AntennaWeightType::Hamming && length > 1) {
        for (int i = 0; i < length; ++i) {
            weights[static_cast<std::size_t>(i)] =
                0.54 - 0.46 * std::cos(2.0 * PI * static_cast<Scalar>(i) /
                                       static_cast<Scalar>(length - 1));
        }
    }

    Scalar sum_weights = 0.0;
    for (const Scalar w : weights) {
        sum_weights += w;
    }
    sum_weights = std::max(sum_weights, EPSILON);

    for (Scalar& w : weights) {
        w /= sum_weights;
    }

    return weights;
}

/**
 * @brief 从 CSV 文件读取波位表
 * @param filepath 波位表 CSV 文件路径
 * @return 成功返回 true，失败返回 false
 *
 * @details
 * 支持的 CSV 格式为：
 * az_deg,el_deg
 *
 * 支持：
 * - 空行
 * - 以 # 开头的注释行
 *
 * 例如：
 * # az,el
 * -10,0
 * 0,0
 * 10,5
 */
bool AntennaScanModel::load_beam_table_csv(const std::string& filepath) {
    last_error_.clear();

    std::ifstream input(filepath);
    if (!input.is_open()) {
        last_error_ = "Failed to open beam table CSV: " + filepath;
        return false;
    }

    std::vector<BeamPoint> parsed_table;
    std::string line;
    std::size_t line_number = 0;

    while (std::getline(input, line)) {
        ++line_number;

        const std::string trimmed_line = trim(line);
        if (trimmed_line.empty() || trimmed_line[0] == '#') {
            continue;
        }

        std::stringstream ss(trimmed_line);
        std::string az_token;
        std::string el_token;
        std::string extra_token;

        if (!std::getline(ss, az_token, ',') || !std::getline(ss, el_token, ',')) {
            last_error_ = "Invalid CSV format at line " + std::to_string(line_number) +
                          ": expected 'az_deg,el_deg'";
            return false;
        }

        if (std::getline(ss, extra_token, ',')) {
            if (!trim(extra_token).empty()) {
                last_error_ = "Invalid CSV format at line " + std::to_string(line_number) +
                              ": too many columns";
                return false;
            }
        }

        Scalar az_deg = 0.0;
        Scalar el_deg = 0.0;
        if (!parse_double_strict(trim(az_token), az_deg) ||
            !parse_double_strict(trim(el_token), el_deg)) {
            last_error_ = "Invalid numeric value at line " + std::to_string(line_number);
            return false;
        }

        parsed_table.push_back({az_deg, el_deg});
    }

    if (parsed_table.empty()) {
        last_error_ = "Beam table CSV is empty: " + filepath;
        return false;
    }

    set_beam_table(parsed_table);
    return true;
}

/**
 * @brief 直接设置波位表
 * @param beam_table 波位表
 *
 * @details
 * 每个波位点对应一个 CPI 的波束指向。
 * 调用后当前波位索引会被重置为 0。
 */
void AntennaScanModel::set_beam_table(const std::vector<BeamPoint>& beam_table) {
    beam_table_ = beam_table;
    current_beam_index_ = 0;
}

/**
 * @brief 设置相控阵天线模型
 * @param antenna 相控阵天线模型
 */
void AntennaScanModel::set_antenna(const PhasedArrayAntenna& antenna) {
    antenna_ = antenna;
}

/**
 * @brief 按一个 CPI 推进波位
 * @return 若波位表为空则返回 false，否则返回 true
 *
 * @details
 * 该接口采用循环扫描方式：
 * 当前索引到达末尾后，下一个 CPI 回到第一个波位。
 */
bool AntennaScanModel::advance_one_cpi() {
    if (beam_table_.empty()) {
        return false;
    }

    current_beam_index_ = (current_beam_index_ + 1) % beam_table_.size();
    return true;
}

/**
 * @brief 重置到第一个波位
 */
void AntennaScanModel::reset() {
    current_beam_index_ = 0;
}

/**
 * @brief 获取当前波位指向
 * @return 当前波位的方位/俯仰角
 *
 * @details
 * 若波位表为空，则返回 (0, 0)。
 */
AzEl AntennaScanModel::get_beam_pointing() const {
    if (beam_table_.empty()) {
        return {0.0, 0.0};
    }

    const BeamPoint& beam = beam_table_[current_beam_index_];
    return {beam.azimuth_deg, beam.elevation_deg};
}

/**
 * @brief 获取当前波位下目标方向的天线增益（线性）
 * @param target_az_deg 目标方位角（度）
 * @param target_el_deg 目标俯仰角（度）
 * @return 天线增益（线性）
 *
 * @details
 * 当前接口会先获取当前波位指向，然后调用相控阵天线模型完成目标增益计算。
 */
Scalar AntennaScanModel::get_gain_to_target(Scalar target_az_deg, Scalar target_el_deg) const {
    if (beam_table_.empty()) {
        throw std::runtime_error(kEmptyBeamTableError);
    }

    const AzEl beam = get_beam_pointing();
    return antenna_.gain(target_az_deg, target_el_deg, beam.azimuth, beam.elevation);
}

/**
 * @brief 获取当前波位下目标方向的天线增益（dB）
 * @param target_az_deg 目标方位角（度）
 * @param target_el_deg 目标俯仰角（度）
 * @return 天线增益（dB）
 */
Scalar AntennaScanModel::get_gain_db_to_target(Scalar target_az_deg,
                                               Scalar target_el_deg) const {
    if (beam_table_.empty()) {
        throw std::runtime_error(kEmptyBeamTableError);
    }

    const AzEl beam = get_beam_pointing();
    return antenna_.gain_db(target_az_deg, target_el_deg, beam.azimuth, beam.elevation);
}

/**
 * @brief 获取当前波位下目标方向的归一化功率响应
 * @param target_az_deg 目标方位角（度）
 * @param target_el_deg 目标俯仰角（度）
 * @return 归一化功率响应
 */
Scalar AntennaScanModel::get_normalized_power_to_target(Scalar target_az_deg,
                                                        Scalar target_el_deg) const {
    if (beam_table_.empty()) {
        throw std::runtime_error(kEmptyBeamTableError);
    }

    const AzEl beam = get_beam_pointing();
    return antenna_.normalized_power(target_az_deg, target_el_deg,
                                     beam.azimuth, beam.elevation);
}

/**
 * @brief 判断目标是否位于当前波束主响应范围内
 * @param target_az_deg 目标方位角（度）
 * @param target_el_deg 目标俯仰角（度）
 * @param threshold_db 相对峰值阈值（dB）
 * @return 若归一化功率高于阈值则返回 true
 */
bool AntennaScanModel::is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                                         Scalar threshold_db) const {
    if (beam_table_.empty()) {
        throw std::runtime_error(kEmptyBeamTableError);
    }

    const AzEl beam = get_beam_pointing();
    return antenna_.is_target_in_beam(target_az_deg, target_el_deg,
                                      beam.azimuth, beam.elevation, threshold_db);
}

}  // namespace radar
