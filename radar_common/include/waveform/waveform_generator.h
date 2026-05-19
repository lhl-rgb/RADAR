/**
 * @file waveform_generator.h
 * @brief 波形生成器
 *
 * 负责生成雷达发射波形（LFM、NLFM、相位编码等）及其匹配滤波器。
 *
 * 支持的波形类型：
 * 1. LFM - 线性调频波形
 * 2. NLFM - 非线性调频波形
 * 3. PHASE_CODED - 相位编码波形（Barker 码）
 * 4. CW - 连续波
 *
 * 工作流程：
 * 1. 配置波形参数（类型、带宽、脉宽等）
 * 2. 初始化生成对应波形和匹配滤波器
 * 3. 提供波形及其频谱用于回波生成
 */

#pragma once

#include "core/types.h"
#include "core/radar_system_params.h"
#include "waveform/waveform_config.h"

#include <string>
#include <vector>

namespace radar::waveform {

/**
 * @brief 波形生成器
 */
class WaveformGenerator {
public:
    WaveformGenerator() = default;
    ~WaveformGenerator() = default;

    // 禁止拷贝
    WaveformGenerator(const WaveformGenerator&) = delete;
    WaveformGenerator& operator=(const WaveformGenerator&) = delete;

    // ========================================================================
    // 配置注入
    // ========================================================================

    /**
     * @brief 设置波形配置
     * @param cfg 波形配置参数
     */
    void set_config(const WaveformConfig& cfg) { cfg_ = cfg; }

    /**
     * @brief 设置系统参数
     * @param sys 雷达系统参数
     */
    void set_system_params(const RadarSystemParams& sys) { sys_ = sys; }

    // ========================================================================
    // 初始化
    // ========================================================================

    /**
     * @brief 初始化波形生成器
     * @return 如果初始化成功返回 true
     *
     * 根据配置的波形类型生成对应波形和匹配滤波器。
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
     * @brief 生成 LFM 波形
     * @param num_samples 采样点数
     * @return LFM 波形向量
     */
    ComplexVec generate_lfm(int num_samples) const;

    /**
     * @brief 生成相位编码波形
     * @param num_samples 采样点数
     * @param code_type 相位编码类型（Barker 码）
     * @return 相位编码波形向量
     */
    ComplexVec generate_phase_coded(int num_samples, PhaseCodeType code_type) const;

    /**
     * @brief 生成非线性调频波形
     * @param num_samples 采样点数
     * @return NLFM 波形向量
     */
    ComplexVec generate_nlfm(int num_samples) const;

    /**
     * @brief 生成匹配滤波器
     * @return 匹配滤波器向量（时域共轭反转）
     */
    ComplexVec generate_matched_filter() const;

    /**
     * @brief 计算波形的频域表示
     * @param waveform 时域波形
     * @return 频域频谱
     */
    ComplexVec compute_spectrum(const ComplexVec& waveform) const;

    // ========================================================================
    // 状态查询
    // ========================================================================

    /**
     * @brief 获取当前波形配置
     */
    const WaveformConfig& config() const { return cfg_; }

    /**
     * @brief 获取当前系统参数
     */
    const RadarSystemParams& system_params() const { return sys_; }

    /**
     * @brief 获取当前波形
     */
    const ComplexVec& get_waveform() const { return waveform_; }

    /**
     * @brief 获取匹配滤波器
     */
    const ComplexVec& get_matched_filter() const { return matched_filter_; }

    /**
     * @brief 获取波形频谱
     */
    const ComplexVec& get_spectrum() const { return spectrum_; }

private:
    /**
     * @brief 生成窗函数波形
     */
    ComplexVec generate_window(int length, WindowType type) const;

    /**
     * @brief 生成窗函数权重
     */
    ScalarVector generate_window_weights(int length, WindowType type) const;

    RadarSystemParams sys_;
    WaveformConfig cfg_;
    ComplexVec waveform_;
    ComplexVec matched_filter_;
    ComplexVec spectrum_;
    bool initialized_ = false;
    std::string last_error_;
};

} // namespace radar
