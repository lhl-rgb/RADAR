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
    BeamTable beams;             ///< 波位表数据
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
            j["beams"].push_back({{"azimuth", beam.azimuth_deg}, {"elevation", beam.elevation_deg}});
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

// JSON 序列化支持
inline void from_json(const nlohmann::json& j, AntennaConfig& cfg) {
    // 支持扁平化格式（向后兼容）
    if (j.contains("model_type")) j.at("model_type").get_to(cfg.model_type);
    if (j.contains("weight_type_az")) j.at("weight_type_az").get_to(cfg.weight_type_az);
    if (j.contains("weight_type_el")) j.at("weight_type_el").get_to(cfg.weight_type_el);
    if (j.contains("num_elements_az")) j.at("num_elements_az").get_to(cfg.num_elements_az);
    if (j.contains("num_elements_el")) j.at("num_elements_el").get_to(cfg.num_elements_el);
    if (j.contains("spacing_az_lambda")) j.at("spacing_az_lambda").get_to(cfg.spacing_az_lambda);
    if (j.contains("spacing_el_lambda")) j.at("spacing_el_lambda").get_to(cfg.spacing_el_lambda);
    if (j.contains("peak_gain_db")) j.at("peak_gain_db").get_to(cfg.peak_gain_db);

    // 支持嵌套格式（新格式）
    if (j.contains("options")) {
        const auto& opts = j.at("options");
        if (opts.contains("model_type")) opts.at("model_type").get_to(cfg.model_type);
        if (opts.contains("weight_type_az")) opts.at("weight_type_az").get_to(cfg.weight_type_az);
        if (opts.contains("weight_type_el")) opts.at("weight_type_el").get_to(cfg.weight_type_el);
    }

    if (j.contains("params")) {
        const auto& params = j.at("params");
        if (params.contains("num_elements_az")) params.at("num_elements_az").get_to(cfg.num_elements_az);
        if (params.contains("num_elements_el")) params.at("num_elements_el").get_to(cfg.num_elements_el);
        if (params.contains("spacing_az_lambda")) params.at("spacing_az_lambda").get_to(cfg.spacing_az_lambda);
        if (params.contains("spacing_el_lambda")) params.at("spacing_el_lambda").get_to(cfg.spacing_el_lambda);
        if (params.contains("peak_gain_db")) params.at("peak_gain_db").get_to(cfg.peak_gain_db);
    }
}

inline void to_json(nlohmann::json& j, const AntennaConfig& cfg) {
    j = nlohmann::json{
        {"model_type", cfg.model_type},
        {"weight_type_az", cfg.weight_type_az},
        {"weight_type_el", cfg.weight_type_el},
        {"num_elements_az", cfg.num_elements_az},
        {"num_elements_el", cfg.num_elements_el},
        {"spacing_az_lambda", cfg.spacing_az_lambda},
        {"spacing_el_lambda", cfg.spacing_el_lambda},
        {"peak_gain_db", cfg.peak_gain_db}
    };
}

}  // namespace radar::antenna
