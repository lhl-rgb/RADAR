/**
 * @file math_utils.h
 * @brief 跨模块复用的基础数学转换工具
 */

#pragma once

#include <algorithm>
#include <cmath>

#include "core/types.h"

namespace radar::math {

/**
 * @brief 判断是否为有限数
 */
inline bool is_finite(Scalar value) {
    return std::isfinite(value);
}

/**
 * @brief 判断是否有限且非负
 */
inline bool is_finite_nonnegative(Scalar value) {
    return std::isfinite(value) && value >= 0.0;
}

/**
 * @brief 判断是否有限且严格大于 0
 */
inline bool is_finite_positive(Scalar value) {
    return std::isfinite(value) && value > 0.0;
}


inline bool is_vec3_finite(const Vec3& v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

/**
 * @brief dB 转线性值
 */
inline Scalar db_to_linear(Scalar db_value) {
    return std::pow(10.0, db_value / 10.0);
}

/**
 * @brief 线性值转 dB
 * @details 对非正数值做 EPSILON 保护，避免 log10(0)。
 */
inline Scalar linear_to_db(Scalar linear_value) {
    return 10.0 * std::log10(std::max(linear_value, EPSILON));
}

/**
 * @brief 角度转弧度
 */
inline Scalar deg_to_rad(Scalar deg_value) {
    return deg_value * PI / 180.0;
}

/**
 * @brief 弧度转角度
 */
inline Scalar rad_to_deg(Scalar rad_value) {
    return rad_value * 180.0 / PI;
}

/**
 * @brief 角度归一到 [-180, 180)
 */
inline Scalar wrap_azimuth_deg(Scalar azimuth_deg) {
    Scalar wrapped = std::fmod(azimuth_deg, 360.0);
    if (wrapped < -180.0) {
        wrapped += 360.0;
    } else if (wrapped >= 180.0) {
        wrapped -= 360.0;
    }
    return wrapped;
}

inline void to_az_el_deg(const Vec3& position_m, Scalar& az_deg, Scalar& el_deg) {
    const Scalar x = position_m.x();
    const Scalar y = position_m.y();
    const Scalar z = position_m.z();
    const Scalar xy = std::sqrt(x * x + y * y);

    az_deg = math::rad_to_deg(std::atan2(y, x));
    el_deg = math::rad_to_deg(std::atan2(z, xy));
}

/**
 * @brief 把值截断到非负区间（物理下限）
 */
inline Scalar clamp_nonnegative(Scalar value) {
    return std::max(value, 0.0);
}

/**
 * @brief 把值截断到正下限（数值稳定下限）
 */
inline Scalar clamp_positive_eps(Scalar value, Scalar eps = EPSILON) {
    return std::max(value, eps);
}

/**
 * @brief 安全除法：分母最小取 EPSILON
 */
inline Scalar safe_div(Scalar numerator, Scalar denominator, Scalar eps = EPSILON) {
    return numerator / clamp_positive_eps(denominator, eps);
}





}  // namespace radar::math
