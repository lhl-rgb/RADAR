/**
 * @file sea_clutter_model.h
 * @brief 海杂波双层建模生成器
 */

#pragma once

#include <cstddef>
#include <random>
#include <string>
#include <vector>

#include "../core/antenna_set.h"
#include "../core/radar_params.h"
#include "../core/types.h"

namespace radar {

/**
 * @brief 海杂波模型
 * @details
 * 采用“双层建模”：
 * 1) 几何/平均功率层：按地距-方位扇形网格计算单元平均接收功率；
 * 2) 统计起伏层：按 K 分布（SIRP）和高斯多普勒谱生成慢时间复序列。
 */
class SeaClutterModel {
public:
  SeaClutterModel() = default;
  explicit SeaClutterModel(const SeaClutterParams &params);

  /**
   * @brief 设置海杂波参数
   * @return 成功返回 true；失败返回 false 且保持当前状态不变
   */
  bool set_params(const SeaClutterParams &params);

  /**
   * @brief 获取当前参数
   */
  const SeaClutterParams &params() const { return params_; }

  /**
   * @brief 获取最近一次错误信息
   */
  const std::string &last_error() const { return last_error_; }

  /**
   * @brief 生成一个 CPI 的海杂波原始回波矩阵（clutter-only）
   * @param radar_params 雷达参数
   * @param antenna 当前使用的天线模型
   * @param beam_pointing 当前波位指向
   * @param beam_index 当前波位索引
   * @param tx_waveform 发射波形（时域基带）
   * @param out_clutter 输出海杂波 CPI 回波
   * @return 成功返回 true；失败返回 false
   */
  bool generate_cpi(const RadarParams &radar_params,
                    const PhasedArrayAntenna &antenna,
                    const AzEl &beam_pointing, int beam_index,
                    const ComplexVec &tx_waveform, CpiEcho &out_clutter);

private:
  /**
   * @brief 单个散射单元的几何与功率信息
   */
  struct CellInfo {
    int range_index = 0;          ///< 地距网格索引。
    int az_index = 0;             ///< 方位网格索引。
    Scalar ground_range_m = 0.0;  ///< 地距（m）。
    Scalar azimuth_deg = 0.0;     ///< 单元方位角（deg）。
    Scalar elevation_deg = 0.0;   ///< 单元俯仰角（deg）。
    Scalar slant_range_m = 0.0;   ///< 斜距（m）。
    Scalar receive_power_w = 0.0; ///< 单元平均接收功率（W）。
    int start_sample_index = 0;   ///< 相对参考时延的快时间起点采样索引。
  };

  SeaClutterParams params_;
  std::string last_error_;

  /// 参数静态合法性检查
  static bool validate_params(const SeaClutterParams &params,
                              std::string &error);
  /// 多普勒中心频率奈奎斯特约束：要求在 [-PRF/2, PRF/2)。
  static bool validate_doppler_center_nyquist(Scalar doppler_center_hz,
                                              Scalar prf_hz,
                                              std::string &error);
  /// 64-bit 混洗函数，用于把索引映射为高质量种子。
  static uint64_t mix_u64(uint64_t value);
  /// 由 (base_seed, beam, range, az) 生成可复现单元种子。
  static uint64_t cell_seed(uint64_t base_seed, int beam_index, int range_index,
                            int az_index);

  /// 解析地距上下限（支持 -1 跟随 RadarParams）。
  bool resolve_ground_range(const RadarParams &radar_params,
                            Scalar &out_ground_range_min_m,
                            Scalar &out_ground_range_max_m,
                            std::string &error) const;

  /// Morchin 归一化后向散射系数 sigma0（线性值）。
  Scalar morchin_sigma0_linear(Scalar grazing_rad, Scalar fc_hz) const;

  /// 采样 CN(0,1) 复高斯变量。
  Complex sample_complex_gaussian(std::mt19937_64 &rng) const;
  /// 产生相关高斯服系列。
  ComplexVec generate_corr_sequence(std::size_t length, Scalar prf_hz,
                                             Scalar doppler_center_hz,
                                             Scalar doppler_sigma_hz,
                                             std::mt19937_64 &rng) const;
  /// 对基序列施加 K 分布 SIRP 起伏并重新单位功率归一化。
  ComplexVec apply_k_sirp(const ComplexVec &base_sequence, Scalar k_shape_nu,
                          std::mt19937_64 &rng) const;
  /// 把复序列归一化到 mean(|x|^2)=1。
  static void normalize(ComplexVec &sequence);

  /// InTimeMode 模式下，为单元生成独立慢时间序列。
  ComplexVec generate_sequence(std::size_t length, int beam_index,
                                      int range_index, int az_index,
                                      Scalar prf_hz, Scalar doppler_center_hz,
                                      Scalar doppler_sigma_hz,
                                      Scalar k_shape_nu) const;

  /// SequencePoolMode 模式下生成长序列池，供各单元截取，每波位更新一次
  ComplexVec generate_sequence_pool(std::size_t pool_length, int beam_index,
                                 Scalar prf_hz, Scalar doppler_center_hz,
                                 Scalar doppler_sigma_hz,
                                 Scalar k_shape_nu) const;

  /// 构建当前波位下所有散射单元（几何 + 单元功率 + 时延索引）。
  std::vector<CellInfo> build_cells(const RadarParams &radar_params,
                                    const PhasedArrayAntenna &antenna,
                                    const AzEl &beam_pointing,
                                    Scalar ground_range_min_m,
                                    Scalar ground_range_max_m,
                                    Scalar tau_ref_s) const;
};

} // namespace radar
