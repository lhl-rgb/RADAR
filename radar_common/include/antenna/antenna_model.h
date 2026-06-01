/**
 * @file antenna_model.h
 * @brief 天线方向图模型
 *
 * 使用二维高斯主瓣近似计算天线增益。
 */

#pragma once

#include "antenna/antenna_config.h"
#include "core/types.h"

#include <string>

namespace radar::antenna
{

    /**
     * @brief 反射面天线物理模型
     */
    class AntennaModel
    {
    public:
        AntennaModel() = default;
        ~AntennaModel() = default;

        // 禁止拷贝
        AntennaModel(const AntennaModel &) = delete;
        AntennaModel &operator=(const AntennaModel &) = delete;

        // ========================================================================
        // 配置注入
        // ========================================================================

        /**
         * @brief 设置天线配置
         * @param config 天线配置参数
         */
        void set_config(const AntennaConfig &config);

        // ========================================================================
        // 初始化
        // ========================================================================

        bool initialize()
        {
            initialized_ = true;
            return true;
        }
        bool is_initialized() const { return initialized_; }

        // ========================================================================
        // 核心功能
        // ========================================================================

        /**
         * @brief 计算指定目标方向在指定波位下的天线增益（线性）
         */
        Scalar gain(Scalar target_az_deg, Scalar target_el_deg,
                    Scalar beam_az_deg, Scalar beam_el_deg) const;

        /**
         * @brief 计算指定目标方向在指定波位下的天线增益（dB）
         */
        Scalar gain_db(Scalar target_az_deg, Scalar target_el_deg,
                       Scalar beam_az_deg, Scalar beam_el_deg) const;

        /**
         * @brief 计算归一化功率响应（0~1）
         */
        Scalar normalized_power(Scalar target_az_deg, Scalar target_el_deg,
                                Scalar beam_az_deg, Scalar beam_el_deg) const;

        /**
         * @brief 计算一维方位归一化功率响应（0~1）
         */
        Scalar normalized_az_power(Scalar target_az_deg, Scalar beam_az_deg) const;

        /**
         * @brief 计算一维俯仰归一化功率响应（0~1）
         */
        Scalar normalized_el_power(Scalar target_el_deg, Scalar beam_el_deg) const;

        /**
         * @brief 计算一维方位归一化幅度响应（0~1）
         */
        Scalar normalized_az_amplitude(Scalar target_az_deg, Scalar beam_az_deg) const;

        /**
         * @brief 计算一维俯仰归一化幅度响应（0~1）
         */
        Scalar normalized_el_amplitude(Scalar target_el_deg, Scalar beam_el_deg) const;

        /**
         * @brief 判断目标是否在波束主响应范围内
         */
        bool is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                               Scalar beam_az_deg, Scalar beam_el_deg,
                               Scalar threshold_db = -3.0) const;

        // ========================================================================
        // 状态查询
        // ========================================================================

        const AntennaConfig &config() const { return config_; }
        Scalar peak_gain_linear() const { return peak_gain_linear_; }

        Scalar beamwidth_3db_az_deg() const;
        Scalar beamwidth_3db_el_deg() const;

    private:
        /**
         * @brief 二维高斯方向图归一化功率响应（0~1）
         *
         * P(Δaz, Δel) = P_az(Δaz) × P_el(Δel)
         * P_axis(Δ) = exp(-4·ln2 × (Δ / beamwidth_3dB)²)
         */
        Scalar normalized_power_gaussian(Scalar target_az_deg, Scalar target_el_deg,
                                         Scalar beam_az_deg, Scalar beam_el_deg) const;

        AntennaConfig config_;               ///< 天线配置副本
        Scalar peak_gain_linear_ = 1.0;      ///< 峰值增益线性值（由 dB 转换）
        bool initialized_ = false;           ///< 是否已初始化
    };

} // namespace radar::antenna
