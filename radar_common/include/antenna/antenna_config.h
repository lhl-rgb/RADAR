/**
 * @file antenna_config.h
 * @brief 天线模块配置参数（机械扫描反射面天线）
 */

#pragma once

#include "core/types.h"

#include <string>

namespace radar::antenna
{

    /**
     * @brief 机械扫描配置
     * @details 控制天线连续转动与有效扫描扇区
     */
    struct MechanicalScanConfig
    {
        Scalar rotation_rate_dps = 60.0; ///< 天线转速（度/秒）
        Scalar az_start_deg = 0.0;       ///< 有效扫描起始方位角（度）
        Scalar az_end_deg = 360.0;       ///< 有效扫描终止方位角（度）
        Scalar elevation_deg = 0.0;      ///< 固定俯仰角（度）

        Scalar rotation_period_s = 0.0;        ///< 一圈旋转周期（秒）
        Scalar azimuth_step_per_prt_deg = 0.0; ///< 每个 PRT 的方位步进（度）
        int pulses_per_rotation = 0;           ///< 一圈旋转对应的脉冲数

        /**
         * @brief 根据 PRI 计算派生参数
         * @param pri_s 脉冲重复间隔（秒）
         *
         * 计算 rotation_period_s、azimuth_step_per_prt_deg、pulses_per_rotation。
         */
        void compute_derived_params(Scalar pri_s);

        /**
         * @brief 验证机械扫描配置
         * @param error 错误信息输出
         * @return 有效返回 true
         */
        bool validate(std::string &error) const;
    };

    /**
     * @brief 天线配置
     * @details 直接以 3dB 波束宽度和峰值增益描述方向图
     */
    struct AntennaConfig
    {
        Scalar az_beamwidth_deg = 5.0; ///< 方位维 3dB 波束宽度（度）
        Scalar el_beamwidth_deg = 5.0; ///< 俯仰维 3dB 波束宽度（度）
        Scalar el_beam_point_deg = 0.0; ///< 固定俯仰角（度）
        Scalar peak_gain_db = 35.0;    ///< 天线峰值增益（dB）

        /**
         * @brief 验证天线配置
         * @param error 错误信息输出
         * @return 有效返回 true
         */
        bool validate(std::string &error) const;
    };

} // namespace radar::antenna
