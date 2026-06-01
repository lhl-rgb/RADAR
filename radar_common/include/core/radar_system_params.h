// include/core/radar_system_params.h
#pragma once

#include <spdlog/spdlog.h>
#include <string>

#include "types.h"
#include "core/tools/math_utils.h"
#include "core/tools/udp_config.h"

namespace radar {

/**
 * @brief 全局共享雷达系统参数
 * @details 所有模块都依赖的基础物理参数和派生参数
 */
struct RadarSystemParams {
    // ===== 基础物理参数 =====
    Scalar fc_hz = 10.0e9;              ///< 载频 (Hz)
    Scalar prf_hz = 1600.0;             ///< 脉冲重复频率 (Hz)
    Scalar fs_hz = 20.0e6;              ///< ADC采样率 (Hz)
    Scalar bw_hz = 10.0e6;              ///< 信号带宽 (Hz)
    Scalar pulse_width_s = 40.0e-6;     ///< 脉冲宽度 (s)
    Scalar peak_power_w = 5000.0;       ///< 峰值发射功率 (W)

    // ===== 场景参数 =====
    Scalar min_range_m = 1000.0;        ///< 最小作用距离 (m)
    Scalar max_range_m = 120000.0;      ///< 最大作用距离 (m)
    Scalar antenna_height_m = 15.0;     ///< 天线安装高度 (m)
    GeoCoord radar_location{0.0, 0.0, antenna_height_m}; ///< 雷达地理位置

    // ===== 系统损耗 =====
    Scalar noise_figure_db = 4.0;       ///< 接收机噪声系数 (dB)
    Scalar system_loss_db = 6.0;        ///< 系统总损耗 (dB)

    // ===== 派生参数（由 compute_derived_params 计算） =====
    Scalar wavelength_m;                ///< 波长 = C / fc
    Scalar pri_s;                       ///< 脉冲重复间隔 = 1 / prf
    Scalar duty_cycle;                  ///< 占空比 = pulse_width / pri
    Scalar average_power_w;             ///< 平均功率 = peak_power * duty_cycle
    Scalar energy_per_pulse_j;          ///< 每脉冲能量 = peak_power * pulse_width
    Scalar bandwidth_time_product;      ///< 时带宽积 = bw * pulse_width

    Scalar range_resolution_m;          ///< 距离分辨率 = C / (2 * bw)
    Scalar range_bin_size_m;            ///< 距离 bin = C / (2 * fs)

    Scalar max_unambiguous_range_m;     ///< 最大非模糊距离 = C / (2 * prf)
    Scalar max_unambiguous_velocity_mps;///< 最大非模糊速度 = λ * prf / 4
    Scalar max_doppler_hz;              ///< 最大非模糊多普勒 = prf / 2

    int samples_per_pulse;              ///< 每脉冲采样点数
    int samples_per_tx;           ///< 每脉冲发射采样点数（可能与 samples_per_pulse 不同）

    Scalar noise_figure_linear;         ///< 噪声系数线性值
    Scalar system_loss_linear;          ///< 系统损耗线性值

    RadarSystemParams();

    void compute_derived_params();
    bool validate(std::string& error) const;
    void print() const;
};

}  // namespace radar