#pragma once
#include "types.h"
#include "radar_params.h"
#include <memory>
#include <string>
#include <vector>


namespace radar{


class WaveformGenerator {
public:
    WaveformGenerator() = default;
    explicit WaveformGenerator(const RadarParams& params);

    /**
     * @brief 设置雷达参数
     */
    void set_params(const RadarParams& params);

    /**
     * @brief 生成 LFM 波形
     * @param num_samples 采样点数
     * @return 基带复波形
     */
    ComplexVec generate_lfm(int num_samples) const;

    /**
     * @brief 生成相位编码波形
     * @param num_samples 采样点数
     * @param code_type 相位编码类型（Barker 家族）
     * @return 基带复波形
     */
    ComplexVec generate_phase_coded(int num_samples, PhaseCodeType code_type) const;

    /**
     * @brief 生成非线性调频波形
     * @param num_samples 采样点数
     * @return 基带复波形
     */
    ComplexVec generate_nlfm(int num_samples) const;
    /**
     * @brief 生成匹配滤波器
     * @return 匹配滤波器系数 (时域)
     */
    ComplexVec generate_matched_filter() const;

    /**
     * @brief 计算波形的频域表示
     * @param waveform 时域波形
     * @return 频域波形
     * @note 当前版本为占位接口，调用时会抛出未实现异常。
     */
    ComplexVec compute_spectrum(const ComplexVec& waveform) const;

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

    /**
     * @brief 计算波形的时频积 (Time-Bandwidth Product)
     */
    Scalar time_bandwidth_product() const;

private:
    RadarParams params_;
    ComplexVec waveform_;
    ComplexVec matched_filter_;
    ComplexVec spectrum_;

    /**
     * @brief 生成加窗函数
     */
    ComplexVec generate_window(int length, WindowType type) const;
    std::vector<Scalar> generate_window_weights(int length,WindowType type) const;
};




}
