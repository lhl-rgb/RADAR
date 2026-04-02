/**
 * @file beam_scanner.h
 * @brief 波束扫描控制器
 *
 * 负责维护波位表扫描顺序，并与天线物理模型配合完成当前波位下目标增益计算。
 *
 * 工作流程：
 * 1. 加载或设置波位表（方位、仰角列表）
 * 2. 按 CPI 推进波位索引
 * 3. 查询当前波位下的天线增益
 */

#pragma once

#include "antenna/antenna_config.hpp"
#include "antenna/antenna_model.h"
#include "core/types.h"

#include <string>
#include <vector>

namespace radar::antenna {

/**
 * @brief 波束扫描控制器
 */
class BeamScanner {
public:
    BeamScanner() = default;
    ~BeamScanner() = default;

    // 禁止拷贝
    BeamScanner(const BeamScanner&) = delete;
    BeamScanner& operator=(const BeamScanner&) = delete;

    // ========================================================================
    // 配置注入
    // ========================================================================

    /**
     * @brief 设置天线配置
     * @param config 天线配置参数
     *
     * 会传递给内部 AntennaModel 使用。
     */
    void set_antenna_config(const AntennaConfig& config) { antenna_config_ = config; }

    /**
     * @brief 设置波位表配置
     * @param config 波位表配置
     *
     * 支持三种波位表来源：
     * 1. CSV 文件（当 type="file" 且 file_path 有效时）
     * 2. 自定义波位列表（当 beams 非空时）
     * 3. 方位扫描生成（当 type="azimuth_scan" 时，根据 az_start/end/step 生成）
     */
    void set_beam_table_config(const BeamTableConfig& config) { beam_table_config_ = config; }

    // ========================================================================
    // 初始化
    // ========================================================================

    /**
     * @brief 初始化扫描模型
     * @return 如果初始化成功返回 true
     *
     * 根据配置加载波位表：
     * - type="file": 从 CSV 文件加载
     * - type="custom" 或 beams 非空：使用自定义波位列表
     * - type="azimuth_scan": 根据扫描参数生成波位表
     */
    bool initialize();

    /**
     * @brief 检查是否已初始化
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief 获取最近一次错误信息
     */
    const std::string& last_error() const { return last_error_; }

    // ========================================================================
    // 核心功能
    // ========================================================================

    /**
     * @brief 从 CSV 文件读取波位表
     * @param filepath CSV 文件路径
     * @return 如果加载成功返回 true
     *
     * CSV 格式：每行一个波位，格式为 "az_deg,el_deg"
     */
    bool load_beam_table_csv(const std::string& filepath);

    /**
     * @brief 直接设置波位表
     * @param beam_table 波位列表
     */
    void set_beam_table(const BeamTable& beam_table);

    /**
     * @brief 按一个 CPI 推进波位
     * @return 如果推进成功返回 true
     */
    bool advance_one_cpi();

    /**
     * @brief 重置到第一个波位
     */
    void reset();

    /**
     * @brief 获取当前波位下目标方向的天线增益（线性）
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标仰角（度）
     * @return 天线增益（线性值）
     */
    Scalar get_gain_to_target(Scalar target_az_deg, Scalar target_el_deg) const;

    /**
     * @brief 获取当前波位下目标方向的天线增益（dB）
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标仰角（度）
     * @return 天线增益（dB）
     */
    Scalar get_gain_db_to_target(Scalar target_az_deg, Scalar target_el_deg) const;

    /**
     * @brief 获取当前波位下目标方向的归一化功率响应
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标仰角（度）
     * @return 归一化功率响应（0~1）
     */
    Scalar get_normalized_power_to_target(Scalar target_az_deg, Scalar target_el_deg) const;

    /**
     * @brief 判断目标是否位于当前波束主响应范围内
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标仰角（度）
     * @param threshold_db 阈值（dB），默认 -3dB
     * @return 如果目标在波束主响应范围内返回 true
     */
    bool is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                           Scalar threshold_db = -3.0) const;

    // ========================================================================
    // 状态查询
    // ========================================================================

    /**
     * @brief 获取当前波位指向
     */
    AzEl get_beam_pointing() const;

    /**
     * @brief 获取当前波位索引
     */
    std::size_t get_beam_index() const { return current_beam_index_; }

    /**
     * @brief 获取波位总数
     */
    std::size_t beam_count() const { return beam_table_.size(); }

    /**
     * @brief 是否已加载波位表
     */
    bool has_beam_table() const { return !beam_table_.empty(); }

    /**
     * @brief 获取天线模型引用（只读）
     */
    const AntennaModel& antenna_model() const { return antenna_model_; }

private:
    BeamTable beam_table_;
    BeamTableConfig beam_table_config_;
    AntennaConfig antenna_config_;
    AntennaModel antenna_model_;
    std::string last_error_;
    bool initialized_ = false;
    std::size_t current_beam_index_ = 0;
};

} // namespace radar::antenna
