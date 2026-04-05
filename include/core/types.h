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
#include <nlohmann/json.hpp>

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
  GeoCoord() : latitude(0), longitude(0), altitude(0) {}
  GeoCoord(Scalar lat, Scalar lon, Scalar alt) : latitude(lat), longitude(lon), altitude(alt) {}  
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

using BeamTable = std::vector<BeamPoint>; ///< 波位表类型。


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
// 枚举类型（按通用性排序：基础枚举在前，领域特定枚举在后）
//=============================

/**
 * @brief 极化类型
 */
enum class PolarizationType {
    HH,     ///< 发射/接收均为水平极化
    VV      ///< 发射/接收均为垂直极化
};

/**
 * @brief 发射波形类型
 */
enum class WaveformType {
    LFM,          ///< 线性调频
    NLFM,         ///< 非线性调频
    PHASE_CODED,  ///< 相位编码
    CW            ///< 连续波
};

/**
 * @brief 相位编码类型
 */
enum class PhaseCodeType {
    Barker2,   ///< Barker 码长度 2
    Barker3,   ///< Barker 码长度 3
    Barker4,   ///< Barker 码长度 4
    Barker5,   ///< Barker 码长度 5
    Barker7,   ///< Barker 码长度 7
    Barker11,  ///< Barker 码长度 11
    Barker13   ///< Barker 码长度 13
};

/**
 * @brief 窗函数类型
 */
enum class WindowType {
    Rectangular,  ///< 矩形窗
    Hann,         ///< Hann 窗
    Hamming,      ///< Hamming 窗
    Blackman      ///< Blackman 窗
};

/**
 * @brief 天线方向图类型
 */
enum class AntennaPatternType {
    SincSquared,  ///< Sinc^2 主瓣近似
    Gaussian,     ///< 高斯主瓣近似
    CosinePower   ///< 余弦指数主瓣近似
};

/**
 * @brief 相控阵模型类型
 */
enum class PhasedArrayModelType {
    ULA_1D,  ///< 一维相扫：均匀线阵
    UPA_2D   ///< 二维相扫：均匀平面阵
};

/**
 * @brief 阵元加权类型
 */
enum class AntennaWeightType {
    Uniform,  ///< 均匀加权
    Hamming   ///< Hamming 加权
};

/**
 * @brief 噪声电平模式
 */
enum class NoiseLevelMode {
    ComplexSigma,  ///< 使用复噪声 RMS sigma 作为输入
    NoisePower,    ///< 使用噪声功率（W）作为输入
    ThermalKTB     ///< 使用 k*T*B*F 推导噪声功率
};

/**
 * @brief 目标运动模型
 */
enum class MotionModel {
    Stationary,           ///< 静止目标
    ConstantVelocity,     ///< 匀速目标
    ConstantAcceleration, ///< 匀加速目标
    VariableAcceleration  ///< 变加速目标
};

/**
 * @brief Swerling 起伏模型
 */
enum class SwerlingType {
    Swerling0 = 0,  ///< 不起伏
    Swerling1 = 1,  ///< 慢起伏，RCS 指数分布
    Swerling2 = 2,  ///< 快起伏，RCS 指数分布
    Swerling3 = 3,  ///< 慢起伏，RCS 卡方分布（4 自由度）
    Swerling4 = 4   ///< 快起伏，RCS 卡方分布（4 自由度）
};

/**
 * @brief 杂波分布模型
 */
enum class ClutterDistribution {
    Rayleigh,      ///< 瑞利分布
    Weibull,       ///< 韦伯分布
    LogNormal,     ///< 对数正态分布
    KDistribution  ///< K 分布
};

/**
 * @brief 海杂波序列生成策略
 */
enum class SeaClutterSequenceMode {
    InTimeMode,       ///< 每个散射单元按确定性种子独立生成
    SequencePoolMode  ///< 先生成长序列池，再按单元随机起点截取
};

/**
 * @brief 海杂波网格划分模式
 */
enum class SeaClutterGridMode {
    PhysicalGrid,   ///< 按物理分辨率划分（距离分辨率+方位），精度高
    SampleGrid      ///< 直接按采样点划分，速度快，忽略方位变化
};




//=============================
// 目标参数结构体
//=============================

/**
 * @brief 单目标状态（当前 CPI 起始时刻）
 */
struct TargetState {
    uint64_t id = 0;                       ///< 目标唯一标识符
    Vec3 position_m = Vec3::Zero();        ///< 位置
    Vec3 velocity_mps = Vec3::Zero();      ///< 速度
    Vec3 acceleration_mps2 = Vec3::Zero(); ///< 加速度
    MotionModel motion_model = MotionModel::ConstantVelocity;  ///< 运动模型
    Scalar rcs_mean_m2 = 1.0;              ///< 平均 RCS
    SwerlingType swerling = SwerlingType::Swerling0;  ///< Swerling 起伏模型
    bool enabled = true;                   ///< 是否启用该目标
};

using TargetList = std::vector<TargetState>;  ///< 目标状态列表

/**
 * @brief 目标在某一时刻的快照（位置、速度、增益等）
 */
struct TargetSnapshot {
    Scalar slow_time_s = 0.0;
    Vec3 position_m = Vec3::Zero();
    Vec3 velocity_mps = Vec3::Zero();
    Vec3 acceleration_mps2 = Vec3::Zero();
    Scalar range_m = 0.0;
    Scalar radial_velocity_mps = 0.0;
    Scalar radial_acceleration_mps2 = 0.0;
    Scalar gain_linear = 1.0;
    Scalar rcs_m2 = 1.0;
};

/**
 * @brief 目标轨迹(CPI内单个目标轨迹）
 */
struct TargetTrajectory {
    uint64_t target_id = 0;
    std::vector<TargetSnapshot> snapshots;
};
using TrajectoryBatch = std::vector<TargetTrajectory>;// 目标轨迹批次(CPI内所有目标轨迹）

//=============================
// JSON 序列化（inline 函数必须在头文件中）
//=============================

inline void from_json(const nlohmann::json& j, GeoCoord& coord) {
    if (j.contains("latitude")) j.at("latitude").get_to(coord.latitude);
    if (j.contains("longitude")) j.at("longitude").get_to(coord.longitude);
    if (j.contains("altitude")) j.at("altitude").get_to(coord.altitude);
}

inline void to_json(nlohmann::json& j, const GeoCoord& coord) {
    j = nlohmann::json{
        {"latitude", coord.latitude},
        {"longitude", coord.longitude},
        {"altitude", coord.altitude}
    };
}

inline void from_json(const nlohmann::json& j, TargetState& target) {
    if (j.contains("id")) j.at("id").get_to(target.id);
    if (j.contains("position_m")) {
        const auto& pos = j.at("position_m");
        if (pos.size() >= 3) {
            target.position_m = Vec3(pos[0].get<Scalar>(), pos[1].get<Scalar>(), pos[2].get<Scalar>());
        }
    }
    if (j.contains("velocity_mps")) {
        const auto& vel = j.at("velocity_mps");
        if (vel.size() >= 3) {
            target.velocity_mps = Vec3(vel[0].get<Scalar>(), vel[1].get<Scalar>(), vel[2].get<Scalar>());
        }
    }
    if (j.contains("acceleration_mps2")) {
        const auto& acc = j.at("acceleration_mps2");
        if (acc.size() >= 3) {
            target.acceleration_mps2 = Vec3(acc[0].get<Scalar>(), acc[1].get<Scalar>(), acc[2].get<Scalar>());
        }
    }
    if (j.contains("motion_model")) j.at("motion_model").get_to(target.motion_model);
    if (j.contains("rcs_mean_m2")) j.at("rcs_mean_m2").get_to(target.rcs_mean_m2);
    if (j.contains("swerling")) j.at("swerling").get_to(target.swerling);
    if (j.contains("enabled")) j.at("enabled").get_to(target.enabled);
}

inline void to_json(nlohmann::json& j, const TargetState& target) {
    j = nlohmann::json{
        {"id", target.id},
        {"position_m", {target.position_m.x(), target.position_m.y(), target.position_m.z()}},
        {"velocity_mps", {target.velocity_mps.x(), target.velocity_mps.y(), target.velocity_mps.z()}},
        {"acceleration_mps2", {target.acceleration_mps2.x(), target.acceleration_mps2.y(), target.acceleration_mps2.z()}},
        {"motion_model", static_cast<int>(target.motion_model)},
        {"rcs_mean_m2", target.rcs_mean_m2},
        {"swerling", static_cast<int>(target.swerling)},
        {"enabled", target.enabled}
    };
}

} // namespace radar
