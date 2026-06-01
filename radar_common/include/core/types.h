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

namespace radar
{

    //=============================
    // 基础数值与容器类型
    //=============================

    using Scalar = float;                     ///< 基础标量类型。
    using ScalarVector = std::vector<Scalar>; ///< 标量向量类型。
    using Complex = std::complex<Scalar>;     ///< 复数类型（复基带样本）。
    using ComplexVec = std::vector<Complex>;  ///< 复数向量。
    using PulseEcho = std::vector<Complex>;   ///< 单脉冲回波（距离向复样本序列）。

    using Vec2 = Eigen::Vector2f; ///< 二维向量。
    using Vec3 = Eigen::Vector3f; ///< 三维向量。
    using Vec4 = Eigen::Vector4f; ///< 四维向量。

    using Mat2 = Eigen::Matrix2f; ///< 2x2 矩阵。
    using Mat3 = Eigen::Matrix3f; ///< 3x3 矩阵。
    using Mat4 = Eigen::Matrix4f; ///< 4x4 矩阵。

    constexpr Scalar PI = 3.14159265358979323846; ///< 圆周率。
    constexpr Scalar kTwoPi = 2.0f * PI;
    constexpr Scalar C = 299792458.0;             ///< 光速（m/s）。
    constexpr Scalar EPSILON = 1e-10;             ///< 数值稳定性常数。
    constexpr Scalar four_pi_cubed =
        (4.0 * PI) * (4.0 * PI) * (4.0 * PI); ///< 4π的三次方（常用于雷达方程）。
    constexpr Scalar kLog2 = 0.6931471805599453f;
    //=============================
    // 几何/场景结构体
    //=============================

    /**
     * @brief 地理坐标（经纬高）
     */
    struct GeoCoord
    {
        Scalar latitude;  ///< 纬度（度）。
        Scalar longitude; ///< 经度（度）。
        Scalar altitude;  ///< 海拔高度（米）。
        GeoCoord() : latitude(0), longitude(0), altitude(0) {}
        GeoCoord(Scalar lat, Scalar lon, Scalar alt) : latitude(lat), longitude(lon), altitude(alt) {}
    };

    /**
     * @brief 波束指向（方位角+俯仰角）
     */
    struct BeamPoint
    {
        Scalar azimuth_deg = 0.0;   ///< 波束方位角（度）。
        Scalar elevation_deg = 0.0; ///< 波束俯仰角（度）。

        BeamPoint() = default;
        BeamPoint(Scalar az, Scalar el) : azimuth_deg(az), elevation_deg(el) {}
    };

    /**
     * @brief 距离-多普勒单元索引
     */
    struct RDCell
    {
        int range_bin = 0;   ///< 距离单元索引。
        int doppler_bin = 0; ///< 多普勒单元索引。

        RDCell() = default;
        RDCell(int r, int d) : range_bin(r), doppler_bin(d) {}
    };

    //=============================
    // 回波结果结构体
    //=============================

    /**
     * @brief 单脉冲回波结果（含波束位置信息）
     */
    struct PulseResult
    {
        int pulse_index = 0;        ///< 脉冲在扫描中的序号。
        Scalar azimuth_deg = 0.0;   ///< 该脉冲波束方位角（度）。
        Scalar elevation_deg = 0.0; ///< 该脉冲波束俯仰角（度）。
        PulseEcho echo;             ///< 该脉冲的距离向回波（复数采样序列）。
    };

    /**
     * @brief 一次完整扫描结果
     */
    struct ScanEcho
    {
        int scan_index = 0;                     ///< 扫描编号（从 0 递增）。
        std::vector<PulseResult> pulse_results; ///< 扫描内所有脉冲结果。
    };

    //=============================
    // 枚举类型（按通用性排序：基础枚举在前，领域特定枚举在后）
    //=============================

    /**
     * @brief 极化类型
     */
    enum class PolarizationType
    {
        HH, ///< 发射/接收均为水平极化
        VV  ///< 发射/接收均为垂直极化
    };

    /**
     * @brief 发射波形类型
     */
    enum class WaveformType
    {
        LFM,         ///< 线性调频
        NLFM,        ///< 非线性调频
        PHASE_CODED, ///< 相位编码
        CW           ///< 连续波
    };

    /**
     * @brief 相位编码类型
     */
    enum class PhaseCodeType
    {
        Barker2,  ///< Barker 码长度 2
        Barker3,  ///< Barker 码长度 3
        Barker4,  ///< Barker 码长度 4
        Barker5,  ///< Barker 码长度 5
        Barker7,  ///< Barker 码长度 7
        Barker11, ///< Barker 码长度 11
        Barker13  ///< Barker 码长度 13
    };

    /**
     * @brief 窗函数类型
     */
    enum class WindowType
    {
        Rectangular, ///< 矩形窗
        Hann,        ///< Hann 窗
        Hamming,     ///< Hamming 窗
        Blackman     ///< Blackman 窗
    };

    /**
     * @brief 天线方向图类型
     */
    enum class AntennaPatternType
    {
        SincSquared, ///< Sinc^2 主瓣近似
        Gaussian,    ///< 高斯主瓣近似
        CosinePower  ///< 余弦指数主瓣近似
    };

    /**
     * @brief 天线照射类型
     */
    enum class IlluminationType
    {
        Uniform, ///< 均匀照射（sinc²方向图）
        Cosine   ///< 余弦加权（降低旁瓣，展宽主瓣）
    };

    /**
     * @brief 噪声电平模式
     */
    enum class NoiseLevelMode
    {
        ComplexSigma, ///< 使用复噪声 RMS sigma 作为输入
        NoisePower,   ///< 使用噪声功率（W）作为输入
        ThermalKTB    ///< 使用 k*T*B*F 推导噪声功率
    };

    /**
     * @brief 目标运动模型
     */
    enum class MotionModel
    {
        Stationary,           ///< 静止目标
        ConstantVelocity,     ///< 匀速目标
        ConstantAcceleration, ///< 匀加速目标
        VariableAcceleration  ///< 变加速目标
    };

    /**
     * @brief Swerling 起伏模型
     */
    enum class SwerlingType
    {
        Swerling0 = 0, ///< 不起伏
        Swerling1 = 1, ///< 慢起伏，RCS 指数分布
        Swerling2 = 2, ///< 快起伏，RCS 指数分布
        Swerling3 = 3, ///< 慢起伏，RCS 卡方分布（4 自由度）
        Swerling4 = 4  ///< 快起伏，RCS 卡方分布（4 自由度）
    };

    /**
     * @brief 杂波分布模型
     */
    enum class ClutterDistribution
    {
        Rayleigh = 0, ///< 复高斯杂波，幅度为瑞利分布
        Weibull = 1,  ///< ZMNL Weibull 幅度分布
        LogNormal = 2, ///< 对数正态幅度分布，保留复相位
        K = 3          ///< SIRP K 分布，Gamma texture × 复高斯 speckle
    };

    //=============================
    // 目标参数结构体
    //=============================

    /**
     * @brief 单目标状态
     */
    struct TargetState
    {
        uint64_t id = 0;                                          ///< 目标唯一标识符
        Vec3 position_m = Vec3::Zero();                           ///< 位置
        Vec3 velocity_mps = Vec3::Zero();                         ///< 速度
        Vec3 acceleration_mps2 = Vec3::Zero();                    ///< 加速度
        MotionModel motion_model = MotionModel::ConstantVelocity; ///< 运动模型
        Scalar rcs_mean_m2 = 1.0;                                 ///< 平均 RCS
        SwerlingType swerling = SwerlingType::Swerling0;          ///< Swerling 起伏模型
        bool enabled = true;                                      ///< 是否启用该目标
    };

    using TargetList = std::vector<TargetState>; ///< 目标状态列表

    /**
     * @brief 目标在某一时刻的快照（位置、速度、增益等）
     */
    struct TargetSnapshot
    {
        Scalar time_s = 0.0;
        Vec3 position_m = Vec3::Zero();
        Vec3 velocity_mps = Vec3::Zero();
        Vec3 acceleration_mps2 = Vec3::Zero();
        Scalar range_m = 0.0;
        Scalar radial_velocity_mps = 0.0;
        Scalar radial_acceleration_mps2 = 0.0;
        Scalar gain_linear = 1.0;
        Scalar rcs_m2 = 1.0;
    };

} // namespace radar
