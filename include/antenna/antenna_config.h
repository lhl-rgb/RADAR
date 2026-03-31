/**
 * @file antenna_config.h
 * @brief 天线模块配置参数
 */

#pragma once

#include "core/types.h"
#include <string>

namespace radar::antenna {

/**
 * @brief 相控阵天线配置参数
 * @details
 * 本结构仅描述相控阵建模参数，不包含波位扫描状态。
 * 当前版本默认阵元方向图各向同性，因此总方向图由阵因子和峰值增益共同决定。
 */
struct AntennaConfig {
    PhasedArrayModelType model_type = PhasedArrayModelType::UPA_2D;  ///< 相控阵模型类型
    int num_elements_az = 16;        ///< 方位维阵元数；ULA 时表示线阵总阵元数
    int num_elements_el = 12;        ///< 俯仰维阵元数；ULA 时可忽略
    Scalar spacing_az_lambda = 0.5;  ///< 方位维阵元间距，单位：波长
    Scalar spacing_el_lambda = 0.5;  ///< 俯仰维阵元间距，单位：波长
    AntennaWeightType weight_type_az = AntennaWeightType::Uniform;  ///< 方位维加权类型
    AntennaWeightType weight_type_el = AntennaWeightType::Uniform;  ///< 俯仰维加权类型
    Scalar peak_gain_db = 35.0;      ///< 天线峰值增益（dB）
    std::string beam_table_path;     ///< 波位表 CSV 文件路径

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

}  // namespace radar::antenna