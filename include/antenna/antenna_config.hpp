/**
 * @file antenna_config.hpp
 * @brief 天线模块配置参数
 */

#pragma once

#include "core/types.h"

#include <string>
#include <nlohmann/json.hpp>

namespace radar::antenna {

/**
 * @brief 波位表配置
 * @details 存储波位表数据，每行一个波位（方位角、俯仰角）
 */
struct BeamTableConfig {
    std::string type = "azimuth_scan";   ///< 波位表类型："azimuth_scan" 或 "custom" 或 "file"
    std::string file_path;               ///< CSV 文件路径（当 type="file" 时使用）

    // 方位扫描参数
    Scalar az_start_deg = -30.0;         ///< 方位扫描起始角度（度）
    Scalar az_end_deg = 30.0;            ///< 方位扫描结束角度（度）
    Scalar az_step_deg = 10.0;           ///< 方位扫描步长（度）
    Scalar elevation_deg = 0.0;          ///< 固定俯仰角（度）

    // 自定义波位列表（直接在 JSON 中定义）
    std::vector<AzEl> beams;             ///< 波位表数据
};

// JSON 序列化支持
inline void from_json(const nlohmann::json& j, BeamTableConfig& cfg) {
    if (j.contains("type")) j.at("type").get_to(cfg.type);
    if (j.contains("file_path")) j.at("file_path").get_to(cfg.file_path);
    if (j.contains("az_start_deg")) j.at("az_start_deg").get_to(cfg.az_start_deg);
    if (j.contains("az_end_deg")) j.at("az_end_deg").get_to(cfg.az_end_deg);
    if (j.contains("az_step_deg")) j.at("az_step_deg").get_to(cfg.az_step_deg);
    if (j.contains("elevation_deg")) j.at("elevation_deg").get_to(cfg.elevation_deg);

    if (j.contains("beams")) {
        const auto& beams = j.at("beams");
        if (beams.is_array()) {
            cfg.beams.clear();
            for (const auto& beam : beams) {
                if (beam.is_array() && beam.size() >= 2) {
                    cfg.beams.emplace_back(
                        beam[0].template get<Scalar>(),
                        beam[1].template get<Scalar>()
                    );
                } else if (beam.contains("azimuth") && beam.contains("elevation")) {
                    cfg.beams.emplace_back(
                        beam.at("azimuth").template get<Scalar>(),
                        beam.at("elevation").template get<Scalar>()
                    );
                }
            }
        }
    }
}

inline void to_json(nlohmann::json& j, const BeamTableConfig& cfg) {
    j = nlohmann::json{
        {"type", cfg.type},
        {"file_path", cfg.file_path},
        {"az_start_deg", cfg.az_start_deg},
        {"az_end_deg", cfg.az_end_deg},
        {"az_step_deg", cfg.az_step_deg},
        {"elevation_deg", cfg.elevation_deg}
    };
    if (!cfg.beams.empty()) {
        j["beams"] = nlohmann::json::array();
        for (const auto& beam : cfg.beams) {
            j["beams"].push_back({{"azimuth", beam.azimuth}, {"elevation", beam.elevation}});
        }
    }
}

/**
 * @brief 天线配置
 * @details 包含相控阵天线的物理参数和行为选项
 */
struct AntennaConfig {
    // 行为选项
    PhasedArrayModelType model_type = PhasedArrayModelType::UPA_2D;   ///< 相控阵模型类型
    AntennaWeightType weight_type_az = AntennaWeightType::Uniform;    ///< 方位维加权类型
    AntennaWeightType weight_type_el = AntennaWeightType::Uniform;    ///< 俯仰维加权类型

    // 物理参数
    int num_elements_az = 16;         ///< 方位维阵元数；ULA 时表示线阵总阵元数
    int num_elements_el = 12;         ///< 俯仰维阵元数；ULA 时可忽略
    Scalar spacing_az_lambda = 0.5;   ///< 方位维阵元间距，单位：波长
    Scalar spacing_el_lambda = 0.5;   ///< 俯仰维阵元间距，单位：波长
    Scalar peak_gain_db = 35.0;       ///< 天线峰值增益（dB）

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

inline bool AntennaConfig::validate(std::string& error) const {
    if (num_elements_az <= 0) {
        error = "num_elements_az must be positive";
        return false;
    }
    if (spacing_az_lambda <= 0.0) {
        error = "spacing_az_lambda must be positive";
        return false;
    }
    if (peak_gain_db < 0.0) {
        error = "peak_gain_db cannot be negative";
        return false;
    }

    if (model_type == PhasedArrayModelType::UPA_2D) {
        if (num_elements_el <= 0) {
            error = "num_elements_el must be positive for UPA_2D model";
            return false;
        }
        if (spacing_el_lambda <= 0.0) {
            error = "spacing_el_lambda must be positive for UPA_2D model";
            return false;
        }
    }

    return true;
}

} // namespace radar::antenna
