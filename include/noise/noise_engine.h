/**
 * @file noise_engine.h
 * @brief 雷达回波复高斯白噪声生成模块
 *
 * 根据雷达系统参数和噪声配置生成复高斯白噪声。
 *
 * 噪声模式：
 * 1. ComplexSigma - 直接使用复噪声标准差
 * 2. NoisePower - 使用噪声功率 (W)
 * 3. ThermalKTB - 使用热噪声公式 KTB
 *
 * 工作流程：
 * 1. 根据配置计算噪声功率
 * 2. 推导 I/Q 分量的标准差
 * 3. 生成复高斯随机样本
 * 4. 添加到信号中
 */

#pragma once

#include "core/types.h"
#include "core/radar_system_params.h"
#include "noise/noise_config.h"

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>

namespace radar::noise {

/**
 * @brief 复高斯白噪声生成引擎
 */
class NoiseEngine {
public:
    NoiseEngine() = default;
    ~NoiseEngine() = default;

    // 禁止拷贝
    NoiseEngine(const NoiseEngine&) = delete;
    NoiseEngine& operator=(const NoiseEngine&) = delete;

    // ========================================================================
    // 配置注入
    // ========================================================================

    /**
     * @brief 设置噪声配置
     * @param cfg 噪声配置参数
     */
    void set_config(const noise::NoiseConfig& cfg) { cfg_ = cfg; }

    /**
     * @brief 设置系统参数
     * @param sys 雷达系统参数
     */
    void set_system_params(const RadarSystemParams& sys) { sys_ = sys; }

    // ========================================================================
    // 初始化
    // ========================================================================

    /**
     * @brief 初始化噪声引擎
     * @return 如果初始化成功返回 true
     *
     * 验证配置参数并计算噪声功率、I/Q 标准差等缓存值。
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
    // 运行时控制
    // ========================================================================

    /**
     * @brief 重置随机种子
     * @param seed 新的随机种子
     */
    void reseed(uint64_t seed);

    // ========================================================================
    // 核心功能
    // ========================================================================

    /**
     * @brief 生成一个复高斯白噪声样本
     * @return 复噪声样本
     */
    Complex sample();

    /**
     * @brief 生成长度为 n 的复高斯白噪声序列
     * @param n 样本数量
     * @return 复噪声向量
     */
    ComplexVec generate(std::size_t n);

    /**
     * @brief 对复信号原地加噪
     * @param signal 输入输出信号（会被修改）
     */
    void add_noise(ComplexVec& signal);

    /**
     * @brief 对 CPI 回波逐脉冲原地加噪
     * @param cpi_echo 输入输出 CPI 回波（会被修改）
     */
    void add_noise(CpiEcho& cpi_echo);

    // ========================================================================
    // 状态查询
    // ========================================================================

    /**
     * @brief 获取当前配置
     */
    const noise::NoiseConfig& config() const { return cfg_; }

    /**
     * @brief 获取当前系统参数
     */
    const RadarSystemParams& system_params() const { return sys_; }

    /**
     * @brief 获取当前复噪声功率 (W)
     */
    Scalar noise_power_w() const { return noise_power_w_; }

    /**
     * @brief 获取当前复噪声总 RMS
     */
    Scalar noise_sigma() const { return sigma_complex_; }

    /**
     * @brief 获取当前 I/Q 单分量标准差
     */
    Scalar iq_sigma() const { return sigma_iq_; }

private:
    static constexpr Scalar kBoltzmann = 1.380649e-23;

    /**
     * @brief 计算噪声功率
     */
    static bool compute_noise_power(const RadarSystemParams& sys,
                                    const noise::NoiseConfig& cfg,
                                    Scalar& out_noise_power_w,
                                    std::string& error);

    /**
     * @brief 构建缓存值（sigma_complex, sigma_iq）
     */
    static bool build_cache(Scalar noise_power_w,
                            Scalar& out_sigma_complex,
                            Scalar& out_sigma_iq,
                            std::string& error);

    RadarSystemParams sys_;
    noise::NoiseConfig cfg_;
    std::mt19937_64 rng_;
    std::normal_distribution<Scalar> standard_normal_;

    Scalar noise_power_w_ = 1.0e-6;
    Scalar sigma_complex_ = 1.0e-3;
    Scalar sigma_iq_ = 7.071067811865476e-4;

    bool initialized_ = false;
    std::string last_error_;
};

} // namespace radar::noise
