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

    // Morchin 模型参数（论文公式 2-24）
    Scalar morchin_base_coeff = 4.0e-5;     ///< 基础系数 (公式中的 4×10⁻⁵)
    Scalar morchin_theta_coeff = 0.65;      ///< 入射角系数 f(θ,ss) = 0.65 + 0.07×θ
    Scalar morchin_theta_rate = 0.07;       ///< 入射角变化率 (每度)
    Scalar morchin_sea_state = 3.0;         ///< 海况等级 (1-9)
    Scalar morchin_beta_deg = 2.0;          ///< 粗糙度参数β (度)，与海态相关
    Scalar morchin_grazing_angle_floor = 1e-4;  ///< 掠射角下限 (弧度)，避免数值问题

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
    if (j.contains("morchin_base_coeff")) j.at("morchin_base_coeff").get_to(cfg.morchin_base_coeff);
    if (j.contains("morchin_theta_coeff")) j.at("morchin_theta_coeff").get_to(cfg.morchin_theta_coeff);
    if (j.contains("morchin_theta_rate")) j.at("morchin_theta_rate").get_to(cfg.morchin_theta_rate);
    if (j.contains("morchin_sea_state")) j.at("morchin_sea_state").get_to(cfg.morchin_sea_state);
    if (j.contains("morchin_beta_deg")) j.at("morchin_beta_deg").get_to(cfg.morchin_beta_deg);
    if (j.contains("morchin_grazing_angle_floor")) j.at("morchin_grazing_angle_floor").get_to(cfg.morchin_grazing_angle_floor);

    // 兼容旧格式：morchin 嵌套
    if (j.contains("morchin")) {
        const auto& mor = j.at("morchin");
        if (mor.contains("base_coeff")) mor.at("base_coeff").get_to(cfg.morchin_base_coeff);
        if (mor.contains("theta_coeff")) mor.at("theta_coeff").get_to(cfg.morchin_theta_coeff);
        if (mor.contains("theta_rate")) mor.at("theta_rate").get_to(cfg.morchin_theta_rate);
        if (mor.contains("sea_state")) mor.at("sea_state").get_to(cfg.morchin_sea_state);
        if (mor.contains("beta_deg")) mor.at("beta_deg").get_to(cfg.morchin_beta_deg);
        if (mor.contains("grazing_angle_floor")) mor.at("grazing_angle_floor").get_to(cfg.morchin_grazing_angle_floor);
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
        {"morchin_base_coeff", cfg.morchin_base_coeff},
        {"morchin_theta_coeff", cfg.morchin_theta_coeff},
        {"morchin_theta_rate", cfg.morchin_theta_rate},
        {"morchin_sea_state", cfg.morchin_sea_state},
        {"morchin_beta_deg", cfg.morchin_beta_deg},
        {"morchin_grazing_angle_floor", cfg.morchin_grazing_angle_floor}
    };
}

}  // namespace radar::clutter
