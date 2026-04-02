/**
 * @file sea_clutter_model.h
 * @brief 海杂波双层建模生成器
 */

#pragma once

#include "antenna/antenna_model.h"
#include "clutter/sea_clutter_config.hpp"
#include "core/radar_system_params.hpp"
#include "core/types.h"

#include <cstddef>
#include <random>
#include <string>
#include <vector>

namespace radar::clutter {

using radar::antenna::AntennaModel;

/**
 * @brief 海杂波模型
 * @details
 * 采用"双层建模"：
 * 1) 几何/平均功率层：按地距-方位扇形网格计算单元平均接收功率；
 * 2) 统计起伏层：按 K 分布（SIRP）和高斯多普勒谱生成慢时间复序列。
 */
class SeaClutterModel {
public:
    SeaClutterModel() = default;
    explicit SeaClutterModel(const SeaClutterConfig& params);

    /**
     * @brief 设置海杂波参数
     * @return 成功返回 true；失败返回 false 且保持当前状态不变
     */
    bool set_params(const SeaClutterConfig& params);

    /**
     * @brief 获取当前参数
     */
    const SeaClutterConfig& params() const { return params_; }

    /**
     * @brief 获取最近一次错误信息
     */
    const std::string& last_error() const { return last_error_; }

    /**
     * @brief 生成一个 CPI 的海杂波原始回波矩阵（clutter-only）
     * @param system 雷达系统参数
     * @param antenna 当前使用的天线模型
     * @param beam_pointing 当前波位指向
     * @param beam_index 当前波位索引
     * @param tx_waveform 发射波形（时域基带）
     * @param out_clutter 输出海杂波 CPI 回波
     * @return 成功返回 true；失败返回 false
     */
    bool generate_cpi(const RadarSystemParams& system,
                      const AntennaModel& antenna,
                      const AzEl& beam_pointing,
                      int beam_index,
                      const ComplexVec& tx_waveform,
                      CpiEcho& out_clutter);

private:
    struct CellInfo {
        int range_index = 0;
        int az_index = 0;
        Scalar ground_range_m = 0.0;
        Scalar azimuth_deg = 0.0;
        Scalar elevation_deg = 0.0;
        Scalar slant_range_m = 0.0;
        Scalar receive_power_w = 0.0;
        int start_sample_index = 0;
    };

    SeaClutterConfig params_;
    std::string last_error_;

    static bool validate_params(const SeaClutterConfig& params, std::string& error);
    static bool validate_doppler_center_nyquist(Scalar doppler_center_hz,
                                                Scalar prf_hz,
                                                std::string& error);
    static uint64_t mix_u64(uint64_t value);
    static uint64_t cell_seed(uint64_t base_seed, int beam_index, int range_index, int az_index);

    bool resolve_ground_range(const RadarSystemParams& system,
                              Scalar& out_ground_range_min_m,
                              Scalar& out_ground_range_max_m,
                              std::string& error) const;

    Scalar morchin_sigma0_linear(Scalar grazing_rad, Scalar fc_hz) const;

    Complex sample_complex_gaussian(std::mt19937_64& rng) const;
    ComplexVec generate_corr_sequence(std::size_t length, Scalar prf_hz,
                                       Scalar doppler_center_hz,
                                       Scalar doppler_sigma_hz,
                                       std::mt19937_64& rng) const;
    ComplexVec apply_k_sirp(const ComplexVec& base_sequence, Scalar k_shape_nu,
                             std::mt19937_64& rng) const;
    static void normalize(ComplexVec& sequence);

    ComplexVec generate_sequence(std::size_t length, int beam_index,
                                  int range_index, int az_index,
                                  Scalar prf_hz, Scalar doppler_center_hz,
                                  Scalar doppler_sigma_hz,
                                  Scalar k_shape_nu) const;

    ComplexVec generate_sequence_pool(std::size_t pool_length, int beam_index,
                                       Scalar prf_hz, Scalar doppler_center_hz,
                                       Scalar doppler_sigma_hz,
                                       Scalar k_shape_nu) const;

    std::vector<CellInfo> build_cells(const RadarSystemParams& system,
                                       const AntennaModel& antenna,
                                       const AzEl& beam_pointing,
                                       Scalar ground_range_min_m,
                                       Scalar ground_range_max_m,
                                       Scalar tau_ref_s) const;
};

}  // namespace radar