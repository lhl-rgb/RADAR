/**
 * @file sea_clutter_config.hpp
 * @brief 海杂波模块配置参数
 *
 * 设计说明：
 * - SeaClutterConfig: 扁平化配置容器（行为选项 + 物理参数）
 * - Morchin 参数已扁平化到 SeaClutterConfig 中
 */

#pragma once

#include "core/types.h"
#include "core/tools/math_utils.h"
#include <cstdint>
#include <string>
#include <nlohmann/json.hpp>

namespace radar::clutter {

/**
 * @brief 海杂波配置（扁平化）
 *
 * 包含海杂波的行为选项、几何参数、频谱参数和模型参数。
 * Morchin 参数已扁平化为本结构体的成员。
 */
struct SeaClutterConfig {
    // 行为选项
    bool enabled = true;                            ///< 是否启用海杂波
    SeaClutterSequenceMode sequence_mode = SeaClutterSequenceMode::SequencePoolMode; ///< 慢时间序列生成策略
    SeaClutterGridMode grid_mode = SeaClutterGridMode::PhysicalGrid; ///< 网格划分模式
    uint64_t seed = 2026;                           ///< 随机种子

    // 几何参数（PhysicalGrid 模式使用）
    Scalar ground_range_min_m = -1.0;   ///< 地距下限（m）；-1 表示跟随 RadarSystemParams
    Scalar ground_range_max_m = -1.0;   ///< 地距上限（m）；-1 表示跟随 RadarSystemParams
    Scalar az_grid_step_deg = 0.1;      ///< 方位仿真网格步长（度），用于杂波单元离散化

    // 注：距离维步长使用 RadarSystemParams.range_bin_size_m（距离采样单元）
    // 注：方位覆盖宽度由天线 3dB 波束宽度自动确定（AntennaModel::beamwidth_3db_az_deg）

    // 频谱参数
    Scalar k_shape_nu = 0.8;            ///< K 分布形状参数 nu（Gamma 纹理参数）
    Scalar doppler_center_hz = 0.0;     ///< 高斯多普勒谱中心（Hz）
    Scalar doppler_sigma_hz = 20.0;     ///< 高斯多普勒谱标准差（Hz）

    // 序列池参数
    int pool_length_factor = 32;        ///< 序列池长度倍数

    // Morchin 模型参数
    Scalar morchin_a0_db = -40.0;           ///< 基础项系数（dB）
    Scalar morchin_a_g = 10.0;              ///< 掠射角项系数
    Scalar morchin_a_f = 0.0;               ///< 载频项系数
    Scalar morchin_a_s = 1.0;               ///< 海况项系数
    Scalar morchin_sea_state = 3.0;         ///< 海况等级（经验输入）
    Scalar morchin_sin_psi_floor = 1e-4;    ///< sin(psi) 下限，避免 log10(0)

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

// JSON 序列化支持
inline void from_json(const nlohmann::json& j, SeaClutterConfig& cfg) {
    // 行为选项
    if (j.contains("enabled")) j.at("enabled").get_to(cfg.enabled);
    if (j.contains("sequence_mode")) j.at("sequence_mode").get_to(cfg.sequence_mode);
    if (j.contains("grid_mode")) j.at("grid_mode").get_to(cfg.grid_mode);
    if (j.contains("seed")) j.at("seed").get_to(cfg.seed);

    // 几何参数
    if (j.contains("ground_range_min_m")) j.at("ground_range_min_m").get_to(cfg.ground_range_min_m);
    if (j.contains("ground_range_max_m")) j.at("ground_range_max_m").get_to(cfg.ground_range_max_m);
    if (j.contains("az_grid_step_deg")) j.at("az_grid_step_deg").get_to(cfg.az_grid_step_deg);

    // 频谱参数
    if (j.contains("k_shape_nu")) j.at("k_shape_nu").get_to(cfg.k_shape_nu);
    if (j.contains("doppler_center_hz")) j.at("doppler_center_hz").get_to(cfg.doppler_center_hz);
    if (j.contains("doppler_sigma_hz")) j.at("doppler_sigma_hz").get_to(cfg.doppler_sigma_hz);

    // 序列池参数
    if (j.contains("pool_length_factor")) j.at("pool_length_factor").get_to(cfg.pool_length_factor);

    // Morchin 参数（扁平化）
    if (j.contains("morchin_a0_db")) j.at("morchin_a0_db").get_to(cfg.morchin_a0_db);
    if (j.contains("morchin_a_g")) j.at("morchin_a_g").get_to(cfg.morchin_a_g);
    if (j.contains("morchin_a_f")) j.at("morchin_a_f").get_to(cfg.morchin_a_f);
    if (j.contains("morchin_a_s")) j.at("morchin_a_s").get_to(cfg.morchin_a_s);
    if (j.contains("morchin_sea_state")) j.at("morchin_sea_state").get_to(cfg.morchin_sea_state);
    if (j.contains("morchin_sin_psi_floor")) j.at("morchin_sin_psi_floor").get_to(cfg.morchin_sin_psi_floor);

    // 兼容旧格式：morchin 嵌套
    if (j.contains("morchin")) {
        const auto& mor = j.at("morchin");
        if (mor.contains("a0_db")) mor.at("a0_db").get_to(cfg.morchin_a0_db);
        if (mor.contains("a_g")) mor.at("a_g").get_to(cfg.morchin_a_g);
        if (mor.contains("a_f")) mor.at("a_f").get_to(cfg.morchin_a_f);
        if (mor.contains("a_s")) mor.at("a_s").get_to(cfg.morchin_a_s);
        if (mor.contains("sea_state")) mor.at("sea_state").get_to(cfg.morchin_sea_state);
        if (mor.contains("sin_psi_floor")) mor.at("sin_psi_floor").get_to(cfg.morchin_sin_psi_floor);
    }
}

inline void to_json(nlohmann::json& j, const SeaClutterConfig& cfg) {
    j = nlohmann::json{
        {"enabled", cfg.enabled},
        {"sequence_mode", cfg.sequence_mode},
        {"grid_mode", cfg.grid_mode},
        {"seed", cfg.seed},
        {"ground_range_min_m", cfg.ground_range_min_m},
        {"ground_range_max_m", cfg.ground_range_max_m},
        {"az_grid_step_deg", cfg.az_grid_step_deg},
        {"k_shape_nu", cfg.k_shape_nu},
        {"doppler_center_hz", cfg.doppler_center_hz},
        {"doppler_sigma_hz", cfg.doppler_sigma_hz},
        {"pool_length_factor", cfg.pool_length_factor},
        {"morchin_a0_db", cfg.morchin_a0_db},
        {"morchin_a_g", cfg.morchin_a_g},
        {"morchin_a_f", cfg.morchin_a_f},
        {"morchin_a_s", cfg.morchin_a_s},
        {"morchin_sea_state", cfg.morchin_sea_state},
        {"morchin_sin_psi_floor", cfg.morchin_sin_psi_floor}
    };
}

}  // namespace radar::clutter
