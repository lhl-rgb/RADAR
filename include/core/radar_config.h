/**
 * @file radar_config.h
 * @brief 雷达配置组合容器
 * @details 组合所有模块配置，作为配置入口
 */

#pragma once

#include "core/radar_system_params.h"
#include "core/waveform_config.h"
#include "antenna/antenna_config.h"
#include "noise/noise_config.h"
#include "clutter/sea_clutter_config.h"
#include "target/target_config.h"

#include <string>

namespace radar {

/**
 * @brief 雷达配置组合容器
 * @details 包含所有模块配置，用于一次性加载和管理配置
 */
struct RadarConfig {
    RadarSystemParams system;           ///< 全局共享参数
    waveform::WaveformConfig waveform;  ///< 波形配置
    antenna::AntennaConfig antenna;     ///< 天线配置
    noise::NoiseConfig noise;           ///< 噪声配置
    clutter::SeaClutterConfig clutter;  ///< 海杂波配置
    target::TargetConfig target;        ///< 目标配置

    RadarConfig();

    /**
     * @brief 计算派生参数
     * @details 调用 system.compute_derived_params()
     */
    void compute_derived_params();

    /**
     * @brief 验证所有配置
     * @param error 错误信息输出
     * @return 全部有效返回 true
     */
    bool validate(std::string& error) const;

    /**
     * @brief 验证跨模块一致性
     * @param error 错误信息输出
     * @return 一致返回 true
     */
    bool validate_full(std::string& error) const;

    /**
     * @brief 打印配置摘要
     */
    void print() const;
};

}  // namespace radar