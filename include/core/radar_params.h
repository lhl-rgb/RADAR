/**
 * @file radar_params.h
 * @brief 雷达参数定义
 */

#pragma once

#include <map>
#include <optional>
#include <string>

#include "types.h"

namespace radar {
/**
 * @brief 相控阵天线配置参数
 * @details
 * 本结构仅描述相控阵建模参数，不包含波位扫描状态。
 * 当前版本默认阵元方向图各向同性，因此总方向图由阵因子和峰值增益共同决定。
 *
 * 当 model_type = ULA_1D 时：
 * - 使用 num_elements_az、spacing_az_lambda、weight_type_az
 * - num_elements_el、spacing_el_lambda、weight_type_el 可忽略
 *
 * 当 model_type = UPA_2D 时：
 * - 方位和俯仰两个维度参数都参与建模
 */
struct PhasedArrayAntennaConfig {
    PhasedArrayModelType model_type = PhasedArrayModelType::UPA_2D;  ///< 相控阵模型类型
    int num_elements_az = 16;        ///< 方位维阵元数；ULA 时表示线阵总阵元数
    int num_elements_el = 12;        ///< 俯仰维阵元数；ULA 时可忽略
    Scalar spacing_az_lambda = 0.5;  ///< 方位维阵元间距，单位：波长
    Scalar spacing_el_lambda = 0.5;  ///< 俯仰维阵元间距，单位：波长
    AntennaWeightType weight_type_az = AntennaWeightType::Uniform;  ///< 方位维加权类型
    AntennaWeightType weight_type_el = AntennaWeightType::Uniform;  ///< 俯仰维加权类型
    Scalar peak_gain_db = 35.0;  ///< 天线峰值增益（dB）
};

/**
 * @brief 噪声强度配置模式
 * @details
 * - ComplexSigma：直接指定复噪声总 RMS（推荐，最直观）
 * - NoisePower：直接指定噪声功率（W）
 * - ThermalKTB：按热噪声公式 k*T*B*F 计算
 */
enum class NoiseLevelMode {
    ComplexSigma,  ///< 直接使用 sigma_complex（E{|n|^2}=sigma^2）
    NoisePower,    ///< 直接使用 noise_power_w（W）
    ThermalKTB     ///< 使用 noise_figure/system_temperature/noise_bandwidth 推导
};

/**
 * @brief 噪声参数配置
 * @details
 * 本结构采用“显式模式”：
 * - mode = ComplexSigma：使用 sigma_complex 作为主输入；
 * - mode = NoisePower：使用 noise_power_w 作为主输入；
 * - mode = ThermalKTB：使用 k*T*B*F 推导噪声功率。
 */
struct NoiseParams {
    NoiseLevelMode mode = NoiseLevelMode::ComplexSigma;  ///< 噪声强度配置模式

    Scalar sigma_complex = 1.0e-3;  ///< 复噪声总 RMS，满足 E{|n|^2}=sigma^2（ComplexSigma 模式）
    Scalar noise_power_w = 1.0e-6;  ///< 噪声功率（W）（NoisePower 模式）

    Scalar noise_figure_db = 4.0;         ///< 噪声系数（dB）（ThermalKTB 模式）
    Scalar system_temperature_k = 290.0;  ///< 系统噪声温度（K）（ThermalKTB 模式）
    Scalar noise_bandwidth_hz = 20.0e6;   ///< 噪声等效带宽（Hz）（ThermalKTB 模式）

    uint64_t seed = 12345;  ///< 随机种子
};



/**
 * @brief 雷达参数集合
 * @details 结构体字段按照“体制 -> 发射 -> 采样接收 -> 天线参数 -> 场景 -> 派生参数”分组。
 */
struct RadarParams {
    //=============================
    // 体制/模式参数
    //=============================
    WaveformType waveform_type = WaveformType::LFM;            ///< 波形类型。
    PhaseCodeType phase_code_type = PhaseCodeType::Barker13;   ///< 相位编码类型（仅相位编码波形使用）。
    WindowType nlfm_window_type = WindowType::Hamming;         ///< NLFM 窗函数类型（仅 NLFM 使用）。
    PolarizationType polarization = PolarizationType::HH;      ///< 极化方式（HH/VV）。
    //=============================
    // 发射参数
    //=============================
    Scalar fc_hz;             ///< 载频（Hz）。
    Scalar bw_hz;             ///< 信号带宽（Hz）。
    Scalar pulse_width_s;     ///< 脉冲宽度（s）。
    Scalar prf_hz;            ///< 脉冲重复频率（Hz）。
    Scalar peak_power_w;      ///< 峰值发射功率（W）。
    //=============================
    // 采样与接收机参数
    //=============================
    Scalar fs_hz;             ///< ADC 采样率（Hz）。
    int pulses_per_cpi;       ///< 每个 CPI 的脉冲数。
    int samples_per_pulse;    ///< 每脉冲采样点数（由 compute_derived_params 自动覆盖）。
    Scalar noise_figure_db;   ///< 接收机噪声系数（dB）。
    Scalar system_loss_db;    ///< 系统总损耗（dB）。
    
    //=============================
    // 天线与波位参数
    //=============================
    PhasedArrayAntennaConfig antenna_config;  ///< 相控阵天线配置参数
    Scalar antenna_height_m;                  ///< 天线安装高度（米）
    std::string beam_table_path;              ///< 波位表 CSV 文件路径

    //=============================
    // 场景与部署参数
    //=============================
    Scalar min_range_m;       ///< 最小作用距离（米）。
    Scalar max_range_m;       ///< 最大作用距离（米）。
    GeoCoord radar_location;  ///< 雷达地理位置（经纬高）。
    //=============================
    // 派生参数
    //=============================
    Scalar wavelength_m;             ///< 波长（米）。
    Scalar range_resolution_m;       ///< 距离分辨率（米）。
    Scalar velocity_resolution_mps;  ///< 速度分辨率（m/s）。
    Scalar max_unambiguous_range_m;  ///< 最大非模糊距离（米）。
    Scalar max_velocity;             ///< 最大非模糊速度（m/s）。
    Scalar noise_figure_linear;      ///< 噪声系数线性值。
    Scalar system_loss_linear;       ///< 系统损耗线性值。
    RadarParams();
    /**
     * @brief 计算全部派生参数
     */
    void compute_derived_params();
    /**
     * @brief 从 JSON 加载参数
     * @note 当前阶段未启用，调用返回 false。
     */
    bool load_from_json(const std::string& filepath);
    /**
     * @brief 保存参数到 JSON
     * @note 当前阶段未启用，调用返回 false。
     */
    bool save_to_json(const std::string& filepath) const;
    /**
     * @brief 参数有效性检查
     */
    bool validate() const;
    /**
     * @brief 打印参数摘要
     */
    void print() const;
};
/**
 * @brief 雷达参数配置管理器
 */
class RadarParamManager {
public:
    RadarParamManager() = default;
    ~RadarParamManager() = default;
    /**
     * @brief 添加或更新配置
     */
    void add_config(const std::string& name, const RadarParams& params);
    /**
     * @brief 按名称获取配置
     */
    std::optional<RadarParams> get_config(const std::string& name) const;
    /**
     * @brief 设置当前激活配置
     */
    void set_active(const std::string& name);
    /**
     * @brief 获取当前激活配置
     */
    const RadarParams& get_active() const;
private:
    std::map<std::string, RadarParams> configs_;  ///< 名称到配置对象映射。
    std::string active_name_;                     ///< 当前激活配置名称。
    RadarParams active_params_;                   ///< 当前激活配置副本。
};
}  // namespace radar
