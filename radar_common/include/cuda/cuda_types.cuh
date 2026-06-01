#pragma once

/// @file cuda_types.cuh
/// @brief CUDA 端通用 POD 类型定义。

#include "core/types.h"

#include <cstddef>

#ifdef __CUDACC__
#define RADAR_CUDA_HD __host__ __device__
#else
#define RADAR_CUDA_HD
#endif

namespace radar::cuda
{
    /// @brief CUDA 内部使用的复数 POD。
    ///
    /// 不使用 std::complex，避免设备端兼容性和 ABI 假设；也不绑定 cuFloatComplex，
    /// 避免在未调用 cuComplex/cuFFT/cuBLAS 时引入额外类型转换。
    struct DeviceComplex
    {
        Scalar real;
        Scalar imag;
    };

    /// @brief 复高斯样本对（偶数脉冲 + 奇数脉冲）
    ///
    /// 一次 Philox 调用产生 4 个 32-bit 值，分别用于生成
    /// 偶数脉冲和奇数脉冲的复高斯样本，提高 FIR 卷积吞吐量。
    struct DeviceComplexPair
    {
        DeviceComplex even; ///< 偶数脉冲样本
        DeviceComplex odd;  ///< 奇数脉冲样本
    };

    /// @brief 计算二维行主序数组的扁平索引。
    ///
    /// 使用 std::size_t 承载数组偏移，避免较大距离-方位网格下 int 乘法溢出。
    RADAR_CUDA_HD inline std::size_t row_major_index(int row, int column, int column_count)
    {
        return static_cast<std::size_t>(row) * static_cast<std::size_t>(column_count) +
               static_cast<std::size_t>(column);
    }

} // namespace radar::cuda

#undef RADAR_CUDA_HD
