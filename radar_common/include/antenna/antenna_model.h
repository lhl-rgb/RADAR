/**
 * @file antenna_model.h
 * @brief 相控阵天线物理模型
 *
 * 负责在给定目标方向与波位指向的条件下，计算目标方向天线增益。
 *
 * 支持两种模型：
 * 1. ULA_1D - 一维相扫均匀线阵
 * 2. UPA_2D - 二维相扫均匀平面阵
 *
 * 增益模型：
 * G = G_peak * |AF|^2
 * 其中 AF 为归一化阵因子，G_peak 为峰值增益。
 */

#pragma once

#include "antenna/antenna_config.h"
#include "core/types.h"

#include <string>
#include <vector>

namespace radar::antenna {

/**
 * @brief 相控阵天线物理模型
 */
class AntennaModel {
public:
    AntennaModel() = default;
    ~AntennaModel() = default;

    // 禁止拷贝
    AntennaModel(const AntennaModel&) = delete;
    AntennaModel& operator=(const AntennaModel&) = delete;

    // ========================================================================
    // 配置注入
    // ========================================================================

    /**
     * @brief 设置天线配置
     * @param config 天线配置参数
     *
     * 会触发内部缓存刷新（权重向量、峰值增益等）。
     */
    void set_config(const AntennaConfig& config);

    // ========================================================================
    // 初始化
    // ========================================================================

    /**
     * @brief 初始化天线
     * @return 如果初始化成功返回 true
     */
    bool initialize() { initialized_ = true; return true; }

    /**
     * @brief 检查是否已初始化
     */
    bool is_initialized() const { return initialized_; }

    // ========================================================================
    // 核心功能
    // ========================================================================

    /**
     * @brief 计算指定目标方向在指定波位下的天线增益（线性）
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标仰角（度）
     * @param beam_az_deg 波束方位指向（度）
     * @param beam_el_deg 波束仰角指向（度）
     * @return 天线增益（线性值）
     */
    Scalar gain(Scalar target_az_deg, Scalar target_el_deg,
                Scalar beam_az_deg, Scalar beam_el_deg) const;

    /**
     * @brief 计算指定目标方向在指定波位下的天线增益（dB）
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标仰角（度）
     * @param beam_az_deg 波束方位指向（度）
     * @param beam_el_deg 波束仰角指向（度）
     * @return 天线增益（dB）
     */
    Scalar gain_db(Scalar target_az_deg, Scalar target_el_deg,
                   Scalar beam_az_deg, Scalar beam_el_deg) const;

    /**
     * @brief 计算指定目标方向在指定波位下的归一化功率响应
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标仰角（度）
     * @param beam_az_deg 波束方位指向（度）
     * @param beam_el_deg 波束仰角指向（度）
     * @return 归一化功率响应（0~1）
     */
    Scalar normalized_power(Scalar target_az_deg, Scalar target_el_deg,
                            Scalar beam_az_deg, Scalar beam_el_deg) const;

    /**
     * @brief 判断目标是否位于当前波束主响应范围内
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标仰角（度）
     * @param beam_az_deg 波束方位指向（度）
     * @param beam_el_deg 波束仰角指向（度）
     * @param threshold_db 阈值（dB），默认 -3dB
     * @return 如果目标在波束主响应范围内返回 true
     */
    bool is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                           Scalar beam_az_deg, Scalar beam_el_deg,
                           Scalar threshold_db = -3.0) const;

    // ========================================================================
    // 状态查询
    // ========================================================================

    /**
     * @brief 获取当前配置
     */
    const AntennaConfig& config() const { return config_; }

    /**
     * @brief 获取峰值增益（线性值）
     */
    Scalar peak_gain_linear() const { return peak_gain_linear_; }

    /**
     * @brief 计算方位维 3dB 波束宽度（度）
     * @details 对于均匀加权线阵，3dB 波束宽度 ≈ 0.886 / (N * d_lambda) 弧度
     * @return 3dB 波束宽度（度）
     */
    Scalar beamwidth_3db_az_deg() const;

    /**
     * @brief 计算俯仰维 3dB 波束宽度（度）
     * @return 3dB 波束宽度（度）
     */
    Scalar beamwidth_3db_el_deg() const;

private:
    /**
     * @brief 刷新内部缓存（权重向量、峰值增益线性值）
     */
    void refresh_internal_cache();

    /**
     * @brief 计算一维线阵的归一化功率响应
     */
    Scalar normalized_power_ula(Scalar target_az_deg, Scalar beam_az_deg) const;

    /**
     * @brief 计算二维平面阵的归一化功率响应
     */
    Scalar normalized_power_upa(Scalar target_az_deg, Scalar target_el_deg,
                                Scalar beam_az_deg, Scalar beam_el_deg) const;

    /**
     * @brief 生成加权向量
     */
    static ScalarVector make_weights(int length, AntennaWeightType weight_type);

    AntennaConfig config_;
    Scalar peak_gain_linear_ = 1.0;
    ScalarVector weights_az_;
    ScalarVector weights_el_;
    bool initialized_ = false;
};

} // namespace radar::antenna
