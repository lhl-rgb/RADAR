/**
 * @file antenna_set.h
 * @brief 相控阵天线与波位扫描模型
 */

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "radar_params.h"
#include "types.h"

namespace radar {

/**
 * @brief 相控阵天线模型
 * @details
 * 本类负责在给定目标方向与当前波位指向的条件下，计算目标方向天线增益。
 *
 * 支持两种模型：
 * - ULA_1D：一维相扫均匀线阵
 * - UPA_2D：二维相扫均匀平面阵
 *
 * 当前版本默认阵元方向图各向同性，因此总增益模型为：
 * G = G_peak * |AF|^2
 * 其中 AF 为归一化阵因子。
 */
class PhasedArrayAntenna {
public:
    PhasedArrayAntenna() = default;
    explicit PhasedArrayAntenna(const PhasedArrayAntennaConfig& config);

    /**
     * @brief 设置相控阵天线参数
     * @param config 相控阵配置参数
     */
    void set_config(const PhasedArrayAntennaConfig& config);

    /**
     * @brief 获取当前相控阵配置参数
     * @return 配置结构体引用
     */
    const PhasedArrayAntennaConfig& config() const { return config_; }

    /**
     * @brief 获取峰值增益（线性）
     * @return 峰值增益（线性）
     */
    Scalar peak_gain_linear() const { return peak_gain_linear_; }

    /**
     * @brief 计算指定目标方向在指定波位下的天线增益（线性）
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标俯仰角（度）
     * @param beam_az_deg 当前波位方位角（度）
     * @param beam_el_deg 当前波位俯仰角（度）
     * @return 天线增益（线性）
     */
    Scalar gain(Scalar target_az_deg, Scalar target_el_deg,
                Scalar beam_az_deg, Scalar beam_el_deg) const;

    /**
     * @brief 计算指定目标方向在指定波位下的天线增益（dB）
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标俯仰角（度）
     * @param beam_az_deg 当前波位方位角（度）
     * @param beam_el_deg 当前波位俯仰角（度）
     * @return 天线增益（dB）
     */
    Scalar gain_db(Scalar target_az_deg, Scalar target_el_deg,
                   Scalar beam_az_deg, Scalar beam_el_deg) const;

    /**
     * @brief 计算指定目标方向在指定波位下的归一化功率响应
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标俯仰角（度）
     * @param beam_az_deg 当前波位方位角（度）
     * @param beam_el_deg 当前波位俯仰角（度）
     * @return 归一化功率响应
     */
    Scalar normalized_power(Scalar target_az_deg, Scalar target_el_deg,
                            Scalar beam_az_deg, Scalar beam_el_deg) const;

    /**
     * @brief 判断目标是否位于当前波束主响应范围内
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标俯仰角（度）
     * @param beam_az_deg 当前波位方位角（度）
     * @param beam_el_deg 当前波位俯仰角（度）
     * @param threshold_db 相对峰值阈值（dB），默认 -3 dB
     * @return 若归一化功率高于阈值则返回 true
     */
    bool is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                           Scalar beam_az_deg, Scalar beam_el_deg,
                           Scalar threshold_db = -3.0) const;

private:
    PhasedArrayAntennaConfig config_;
    Scalar peak_gain_linear_ = 1.0;
    std::vector<Scalar> weights_az_;
    std::vector<Scalar> weights_el_;

    /**
     * @brief 更新内部缓存参数
     */
    void refresh_internal_cache();

    /**
     * @brief 计算 ULA 模型下的归一化功率响应
     * @param target_az_deg 目标方位角（度）
     * @param beam_az_deg 当前波位方位角（度）
     * @return 归一化功率响应
     */
    Scalar normalized_power_ula(Scalar target_az_deg, Scalar beam_az_deg) const;

    /**
     * @brief 计算 UPA 模型下的归一化功率响应
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标俯仰角（度）
     * @param beam_az_deg 当前波位方位角（度）
     * @param beam_el_deg 当前波位俯仰角（度）
     * @return 归一化功率响应
     */
    Scalar normalized_power_upa(Scalar target_az_deg, Scalar target_el_deg,
                                Scalar beam_az_deg, Scalar beam_el_deg) const;

    /**
     * @brief 生成指定长度的加权向量
     * @param length 向量长度
     * @param weight_type 加权类型
     * @return 归一化后的加权向量
     */
    static std::vector<Scalar> make_weights(int length, AntennaWeightType weight_type);
};

/**
 * @brief 波位表扫描模型
 * @details
 * 本类负责维护波位表扫描顺序，并与相控阵天线模型配合完成当前波位下目标增益计算。
 */
class AntennaScanModel {
public:
    AntennaScanModel() = default;

    /**
     * @brief 从 CSV 文件读取波位表
     * @details
     * CSV 每行格式为：
     * az_deg,el_deg
     *
     * 支持空行和 # 注释行。
     *
     * @param filepath 波位表 CSV 文件路径
     * @return 成功返回 true，失败返回 false
     */
    bool load_beam_table_csv(const std::string& filepath);

    /**
     * @brief 直接设置波位表
     * @param beam_table 波位表
     */
    void set_beam_table(const std::vector<BeamPoint>& beam_table);

    /**
     * @brief 设置相控阵天线模型
     * @param antenna 相控阵天线模型
     */
    void set_antenna(const PhasedArrayAntenna& antenna);

    /**
     * @brief 按一个 CPI 推进波位
     * @return 若波位表为空则返回 false，否则返回 true
     */
    bool advance_one_cpi();

    /**
     * @brief 重置到第一个波位
     */
    void reset();

    /**
     * @brief 获取当前波位指向
     * @return 当前波位的方位/俯仰角
     */
    AzEl get_beam_pointing() const;

    /**
     * @brief 获取当前波位索引
     * @return 当前波位索引
     */
    std::size_t get_beam_index() const { return current_beam_index_; }

    /**
     * @brief 获取波位总数
     * @return 波位表长度
     */
    std::size_t beam_count() const { return beam_table_.size(); }

    /**
     * @brief 是否已加载波位表
     * @return 若波位表非空则返回 true
     */
    bool has_beam_table() const { return !beam_table_.empty(); }

    /**
     * @brief 获取当前波位下目标方向的天线增益（线性）
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标俯仰角（度）
     * @return 天线增益（线性）
     * @throw std::runtime_error 当波位表为空时抛出异常
     */
    Scalar get_gain_to_target(Scalar target_az_deg, Scalar target_el_deg) const;

    /**
     * @brief 获取当前波位下目标方向的天线增益（dB）
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标俯仰角（度）
     * @return 天线增益（dB）
     * @throw std::runtime_error 当波位表为空时抛出异常
     */
    Scalar get_gain_db_to_target(Scalar target_az_deg, Scalar target_el_deg) const;

    /**
     * @brief 获取当前波位下目标方向的归一化功率响应
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标俯仰角（度）
     * @return 归一化功率响应
     * @throw std::runtime_error 当波位表为空时抛出异常
     */
    Scalar get_normalized_power_to_target(Scalar target_az_deg, Scalar target_el_deg) const;

    /**
     * @brief 判断目标是否位于当前波束主响应范围内
     * @param target_az_deg 目标方位角（度）
     * @param target_el_deg 目标俯仰角（度）
     * @param threshold_db 相对峰值阈值（dB）
     * @return 若归一化功率高于阈值则返回 true
     * @throw std::runtime_error 当波位表为空时抛出异常
     */
    bool is_target_in_beam(Scalar target_az_deg, Scalar target_el_deg,
                           Scalar threshold_db = -3.0) const;

    /**
     * @brief 获取最近一次加载波位表的错误信息
     * @return 错误信息字符串
     */
    const std::string& last_error() const { return last_error_; }

private:
    std::vector<BeamPoint> beam_table_;
    std::size_t current_beam_index_ = 0;
    PhasedArrayAntenna antenna_;
    std::string last_error_;
};

}  // namespace radar
