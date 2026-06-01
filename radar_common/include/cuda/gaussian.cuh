#pragma once

/// @file gaussian.cuh
/// @brief CUDA 复高斯噪声批量生成接口
///
/// 提供基于 GPU 的复高斯噪声批量生成功能声明。当 CUDA 不可用时，
/// 对应的实现将返回 false 并设置错误信息。

#include "core/types.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace radar::cuda
{

    /// @brief 检查 CUDA 高斯噪声生成器是否可用
    ///
    /// @param error 可选，失败时写入错误信息
    /// @return true 如果 CUDA 设备可用，false 否则
    bool is_gaussian_generator_available(std::string *error = nullptr);

    /// @brief 在 GPU 上批量生成复高斯噪声并回传至主机内存
    ///
    /// 生成 n 个复高斯样本，每维（I/Q）标准差为 sigma_iq。
    /// 使用 seed + sample_offset 作为随机种子以保证可复现性。
    ///
    /// @param out 输出缓冲区，至少 n 个 Complex 元素
    /// @param n 需要生成的样本数量
    /// @param sigma_iq I/Q 分量的标准差
    /// @param seed 随机种子
    /// @param sample_offset 样本起始偏移，用于多批次调用的序列连贯性
    /// @param error 可选，失败时写入错误信息
    /// @return true 生成成功，false 失败
    bool generate_complex_gaussian_host(Complex *out,
                                        std::size_t n,
                                        Scalar sigma_iq,
                                        std::uint64_t seed,
                                        std::uint64_t sample_offset,
                                        std::string *error = nullptr);

} // namespace radar::cuda
