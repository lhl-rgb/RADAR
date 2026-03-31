/**
 * @file sea_clutter_config.h
 * @brief 海杂波模块配置参数
 */

#pragma once

#include "core/types.h"
#include <cstdint>
#include <string>

namespace radar::clutter {

/**
 * @brief Morchin 归一化后向散射模型参数
 * @details
 * 计算口径：
 * sigma0_db = a0_db
 *          + a_g * log10(max(sin(psi), sin_psi_floor))
 *          + a_f * log10(fc_GHz)
 *          + a_s * sea_state
 */
struct MorchinConfig {
    Scalar a0_db = -40.0;       ///< 基础项系数（dB）
    Scalar a_g = 10.0;          ///< 掠射角项系数
    Scalar a_f = 0.0;           ///< 载频项系数
    Scalar a_s = 1.0;           ///< 海况项系数
    Scalar sea_state = 3.0;     ///< 海况等级（经验输入）
    Scalar sin_psi_floor = 1e-4;///< sin(psi) 下限，避免 log10(0)
};

/**
 * @brief 海杂波配置参数
 */
struct SeaClutterConfig {
    bool enabled = true;               ///< 是否启用海杂波
    Scalar ground_range_min_m = -1.0;  ///< 地距下限（m）；-1 表示跟随 RadarSystemParams
    Scalar ground_range_max_m = -1.0;  ///< 地距上限（m）；-1 表示跟随 RadarSystemParams
    Scalar range_step_m = 100.0;       ///< 地距网格步长（m）
    Scalar beam_az_width_deg = 3.0;    ///< 当前波位覆盖的方位角宽（度）
    Scalar az_step_deg = 0.1;          ///< 方位网格步长（度）

    Scalar k_shape_nu = 0.8;           ///< K 分布形状参数 nu（Gamma 纹理参数）
    Scalar doppler_center_hz = 0.0;    ///< 高斯多普勒谱中心（Hz）
    Scalar doppler_sigma_hz = 20.0;    ///< 高斯多普勒谱标准差（Hz）

    SeaClutterSequenceMode sequence_mode =
        SeaClutterSequenceMode::InTimeMode;  ///< 慢时间序列生成策略
    uint64_t seed = 2026;              ///< 随机种子
    int pool_length_factor = 32;       ///< 序列池长度倍数

    MorchinConfig morchin;             ///< Morchin 模型参数

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

}  // namespace radar::clutter