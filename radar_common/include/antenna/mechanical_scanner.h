/**
 * @file mechanical_scanner.h
 * @brief 机械扫描控制器
 *
 * 负责维护天线连续旋转状态，按PRT推进波束方位角。
 */

#pragma once

#include "antenna/antenna_config.h"
#include "antenna/antenna_model.h"
#include "core/types.h"

#include <string>

namespace radar::antenna
{

    /**
     * @brief 机械扫描波束控制器
     */
    class MechanicalScanner
    {
    public:
        MechanicalScanner() = default;
        ~MechanicalScanner() = default;

        MechanicalScanner(const MechanicalScanner &) = delete;
        MechanicalScanner &operator=(const MechanicalScanner &) = delete;

        // ========================================================================
        // 配置注入
        // ========================================================================
        void set_config(const AntennaConfig &ant_config, const MechanicalScanConfig &scan_config);
        void set_antenna_config(const AntennaConfig &config) { antenna_config_ = config; }
        void set_scan_config(const MechanicalScanConfig &config) { scan_config_ = config; }

        // ========================================================================
        // 初始化
        // ========================================================================

        /**
         * @brief 初始化扫描模型
         * @details 初始化天线模型，设置初始波束方位
         */
        bool initialize();

        bool is_initialized() const { return initialized_; }
        const std::string &last_error() const { return last_error_; }

        // ========================================================================
        // 核心功能
        // ========================================================================

        /**
         * @brief 按一个 PRT 推进波束方位
         * @details current_az += azimuth_step_per_prt_deg，取模 [0, 360)
         * @return 推进成功返回 true
         */
        bool advance_one_prt();

        /**
         * @brief 重置到起始方位角
         */
        void reset();

        /**
         * @brief 获取当前波束指向
         */
        BeamPoint get_beam_pointing() const;

        /**
         * @brief 获取指定时刻的波束指向
         * @param time_s 从扫描开始经过的时间（秒）
         */
        BeamPoint get_beam_at_time(Scalar time_s) const;

        bool is_echo_active() const;

        /**
         * @brief 获取当前波束方位角（度）
         */
        Scalar current_azimuth_deg() const { return current_azimuth_deg_; }

        // ========================================================================
        // 增益查询（委托给 AntennaModel）
        // ========================================================================

        Scalar get_gain_to_target(Scalar target_az_deg, Scalar target_el_deg) const;
        Scalar get_gain_db_to_target(Scalar target_az_deg, Scalar target_el_deg) const;
        Scalar get_normalized_power_to_target(Scalar target_az_deg, Scalar target_el_deg) const;
        bool is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                               Scalar threshold_db = -3.0) const;

        // ========================================================================
        // 状态查询
        // ========================================================================

        const AntennaModel &antenna_model() const { return antenna_model_; }

    private:
        MechanicalScanConfig scan_config_;
        AntennaConfig antenna_config_;
        AntennaModel antenna_model_;
        std::string last_error_;
        bool initialized_ = false;

        Scalar current_azimuth_deg_ = 0.0; ///< 当前波束方位角（度）
    };

} // namespace radar::antenna
