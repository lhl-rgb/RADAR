/**
 * @file types.h
 * @brief 核心通用类型定义
 */
#pragma once

#include <complex>
#include <cstdint>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace radar {

//=============================
// 基础数值与容器类型
//=============================

using Scalar = double;                    ///< 基础标量类型。
using ScalarVector = std::vector<Scalar>; ///< 标量向量类型。
using Complex = std::complex<Scalar>;     ///< 复数类型（复基带样本）。
using ComplexVec = std::vector<Complex>;  ///< 复数向量。
using PulseEcho = std::vector<Complex>;   ///< 单脉冲回波（距离向复样本序列）。

using Vec2 = Eigen::Vector2d; ///< 二维向量。
using Vec3 = Eigen::Vector3d; ///< 三维向量。
using Vec4 = Eigen::Vector4d; ///< 四维向量。

using Mat2 = Eigen::Matrix2d; ///< 2x2 矩阵。
using Mat3 = Eigen::Matrix3d; ///< 3x3 矩阵。
using Mat4 = Eigen::Matrix4d; ///< 4x4 矩阵。

constexpr Scalar PI = 3.14159265358979323846; ///< 圆周率。
constexpr Scalar C = 299792458.0;             ///< 光速（m/s）。
constexpr Scalar EPSILON = 1e-10;             ///< 数值稳定性常数。

//=============================
// 几何/场景结构体
//=============================

/**
 * @brief 地理坐标（经纬高）
 */
struct GeoCoord {
  Scalar latitude;  ///< 纬度（度）。
  Scalar longitude; ///< 经度（度）。
  Scalar altitude;  ///< 海拔高度（米）。
};

/**
 * @brief 方位角-俯仰角
 */
struct AzEl {
  Scalar azimuth;   ///< 方位角（度）。
  Scalar elevation; ///< 俯仰角（度）。

  AzEl() : azimuth(0), elevation(0) {}
  AzEl(Scalar az, Scalar el) : azimuth(az), elevation(el) {}
};

/**
 * @brief 波位表条目（每条对应 1 个 CPI）
 */
struct BeamPoint {
  Scalar azimuth_deg = 0.0;   ///< 波位方位角（度）。
  Scalar elevation_deg = 0.0; ///< 波位俯仰角（度）。

  BeamPoint() = default;
  BeamPoint(Scalar az, Scalar el) : azimuth_deg(az), elevation_deg(el) {}
};

/**
 * @brief 距离-多普勒单元索引
 */
struct RDCell {
  int range_bin = 0;   ///< 距离单元索引。
  int doppler_bin = 0; ///< 多普勒单元索引。

  RDCell() = default;
  RDCell(int r, int d) : range_bin(r), doppler_bin(d) {}
};

//=============================
// 回波结果结构体
//=============================

/**
 * @brief 单个波位在一个 CPI 内的回波结果
 */
struct CpiEcho {
  int beam_index = 0;            ///< 当前波位在扫描序列中的索引。
  Scalar azimuth_deg = 0.0;      ///< 当前波位方位角（度）。
  Scalar elevation_deg = 0.0;    ///< 当前波位俯仰角（度）。
  std::vector<PulseEcho> pulses; ///< CPI 内全部脉冲回波。
};

/**
 * @brief 一次完整扫描结果
 */
struct ScanEcho {
  int scan_index = 0;               ///< 扫描编号（从 0 递增）。
  std::vector<CpiEcho> cpi_results; ///< 扫描内所有 CPI 结果。
};

//=============================
// 枚举类型
//=============================

/**
 * @brief 杂波分布模型
 */
enum class ClutterDistribution {
  Rayleigh,     ///< 瑞利分布杂波。
  Weibull,      ///< 韦伯分布杂波。
  LogNormal,    ///< 对数正态分布杂波。
  KDistribution ///< K 分布杂波。
};

/**
 * @brief 海杂波慢时间序列生成策略
 */
enum class SeaClutterSequenceMode {
  InTimeMode,  ///< 每个散射单元按确定性种子独立生成。
  SequencePoolMode ///< 先生成长序列池，再按单元随机起点截取。
};

/**
 * @brief 目标运动模型
 */
enum class MotionModel {
  Stationary,           ///< 静止目标。
  ConstantVelocity,     ///< 匀速目标。
  ConstantAcceleration, ///< 匀加速目标。
  VariableAcceleration  ///< 变加速目标。
};

/**
 * @brief Swerling 起伏模型
 */
enum class SwerlingType {
  Swerling0 = 0, ///< 不起伏。
  Swerling1 = 1, ///< 慢起伏，RCS 指数分布。
  Swerling2 = 2, ///< 快起伏，RCS 指数分布。
  Swerling3 = 3, ///< 慢起伏，RCS 卡方分布（4 自由度）。
  Swerling4 = 4  ///< 快起伏，RCS 卡方分布（4 自由度）。
};

/**
 * @brief 单目标状态（当前 CPI 起始时刻）
 */
struct TargetState {
  uint64_t id = 0;/// 目标唯一标识符
  Vec3 position_m = Vec3::Zero();// 位置（雷达本地直角坐标）
  Vec3 velocity_mps = Vec3::Zero();// 速度
  Vec3 acceleration_mps2 = Vec3::Zero();// 加速度
  MotionModel motion_model = MotionModel::ConstantVelocity;// 运动模型
  Scalar rcs_mean_m2 = 1.0;// 平均 RCS（平方米）
  SwerlingType swerling = SwerlingType::Swerling0;// Swerling 起伏模型
  bool enabled = true;// 是否启用该目标
};
using TargetList = std::vector<TargetState>;

/**
 * @brief 极化类型
 */
enum class PolarizationType {
  HH, ///< 发射/接收均为水平极化。
  VV  ///< 发射/接收均为垂直极化。
};

/**
 * @brief 天线方向图类型
 */
enum class AntennaPatternType {
  SincSquared, ///< Sinc^2 主瓣近似。
  Gaussian,    ///< 高斯主瓣近似。
  CosinePower  ///< 余弦指数主瓣近似。
};

/**
 * @brief 发射波形类型
 */
enum class WaveformType {
  LFM,         ///< 线性调频。
  NLFM,        ///< 非线性调频。
  PHASE_CODED, ///< 相位编码。
  CW           ///< 连续波。
};

/**
 * @brief 相位编码类型
 */
enum class PhaseCodeType {
  Barker2,  ///< Barker 长度 2。
  Barker3,  ///< Barker 长度 3。
  Barker4,  ///< Barker 长度 4。
  Barker5,  ///< Barker 长度 5。
  Barker7,  ///< Barker 长度 7。
  Barker11, ///< Barker 长度 11。
  Barker13  ///< Barker 长度 13。
};

/**
 * @brief 窗函数类型
 */
enum class WindowType {
  Rectangular, ///< 矩形窗。
  Hann,        ///< Hann 窗。
  Hamming,     ///< Hamming 窗。
  Blackman     ///< Blackman 窗。
};
/**
 * @brief 相控阵模型类型
 */
enum class PhasedArrayModelType {
  ULA_1D, ///< 一维相扫：均匀线阵
  UPA_2D  ///< 二维相扫：均匀平面阵
};
/**
 * @brief 阵元加权类型
 */
enum class AntennaWeightType {
  Uniform, ///< 均匀加权
  Hamming  ///< Hamming 加权
};

} // namespace radar
