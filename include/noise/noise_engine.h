/**
 * @file noise_engine.h
 * @brief 雷达回波复高斯白噪声生成模块
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <string>

#include "core/radar_params.h"

namespace radar {


/**
 * @brief 复高斯白噪声生成引擎
 *
 * @details
 * 对复基带噪声 n = I + jQ，定义：
 * - E{|n|^2} = sigma_complex^2 = noise_power_w
 * - Var(I) = Var(Q) = noise_power_w / 2
 * - std(I) = std(Q) = sigma_iq = sigma_complex / sqrt(2)
 */
class NoiseEngine {
public:
    NoiseEngine();
    explicit NoiseEngine(const NoiseParams& params);

    /**
     * @brief 设置噪声参数
     * @param params 输入参数
     * @return 成功返回 true；失败返回 false，且保持当前状态不变
     */
    bool set_params(const NoiseParams& params);

    /**
     * @brief 获取当前参数
     */
    const NoiseParams& params() const { return params_; }

    /**
     * @brief 重置随机种子
     */
    void reseed(uint64_t seed);

    /**
     * @brief 获取最近一次错误信息
     */
    const std::string& last_error() const { return last_error_; }

    /**
     * @brief 获取当前复噪声功率
     */
    Scalar noise_power_w() const { return noise_power_w_; }

    /**
     * @brief 获取当前复噪声总 RMS
     * @details 满足 E{|n|^2} = sigma^2
     */
    Scalar noise_sigma() const { return sigma_complex_; }

    /**
     * @brief 获取当前 I/Q 单分量标准差
     */
    Scalar iq_sigma() const { return sigma_iq_; }

    /**
     * @brief 生成一个复高斯白噪声样本
     */
    Complex sample();

    /**
     * @brief 生成长度为 n 的复高斯白噪声序列
     */
    ComplexVec generate(std::size_t n);

    /**
     * @brief 对复信号原地加噪
     */
    void add_noise(ComplexVec& signal);

    /**
     * @brief 对 CPI 回波逐脉冲原地加噪
     */
    void add_noise(CpiEcho& cpi_echo);

private:
    static constexpr Scalar kBoltzmann = 1.380649e-23;

    /**
     * @brief 根据参数计算复噪声功率
     */
    static bool compute_noise_power(const NoiseParams& params,
                                    Scalar& out_noise_power_w,
                                    std::string& error);

    /**
     * @brief 根据噪声功率构建缓存量
     */
    static bool build_cache(Scalar noise_power_w,
                            Scalar& out_sigma_complex,
                            Scalar& out_sigma_iq,
                            std::string& error);

private:
    NoiseParams params_;

    std::mt19937_64 rng_;
    std::normal_distribution<Scalar> standard_normal_;

    Scalar noise_power_w_ = 1.0e-6;                 ///< 当前生效复噪声功率
    Scalar sigma_complex_ = 1.0e-3;                ///< 当前生效复噪声总 RMS
    Scalar sigma_iq_ = 7.071067811865476e-4;       ///< 当前生效 I/Q 分量标准差

    std::string last_error_;
};

}  // namespace radar
