#pragma once

/// @file clutter_rng.cuh
/// @brief 基于 Counter-based RNG (Philox) 的海杂波随机数生成模块
///
/// 本文件实现了基于 Philox4x32-10 算法的确定性随机数生成器，用于在 GPU 和 CPU 上
/// 为海杂波仿真生成复高斯随机样本。采用 counter-based 设计，通过 (seed, range_idx,
/// az_idx, channel_idx, source_id, sample_index) 六元组作为输入，保证每个
/// (距离单元, 方位单元, 通道, 随机源, 脉冲/快拍) 的随机数可确定性复现。

#include "cuda/cuda_types.cuh"
#include <cstdint>

// 根据是否在 CUDA 编译环境中，定义 RADAR_CUDA_HD 宏
// __host__ __device__ 让函数同时可在 CPU 和 GPU 端调用
#ifdef __CUDACC__
#define RADAR_CUDA_HD __host__ __device__
#else
#include <cmath>
#include <algorithm>
#define RADAR_CUDA_HD
#endif

namespace radar::cuda
{
    /// @brief 返回 FIR 抽头对应的历史脉冲索引。
    ///
    /// 仿真开始阶段可能出现 pulse_index < tap。此时 uint64_t 的回绕是有意行为：
    /// 它为第 0 个 PRT 之前的虚拟历史样本提供稳定且互不重复的 counter 编码。
    RADAR_CUDA_HD inline std::uint64_t wrapped_history_pulse_index(std::uint64_t pulse_index,
                                                                   int tap)
    {
        return pulse_index - static_cast<std::uint64_t>(tap);
    }

    /// @brief 4 个 32 位无符号整数组成的向量，用于 Philox 输出的 128-bit 随机数
    struct Uint4
    {
        std::uint32_t x;
        std::uint32_t y;
        std::uint32_t z;
        std::uint32_t w;
    };

    /// @brief 2 个 32 位无符号整数组成的向量，用作 Philox 的 64-bit 密钥
    struct Uint2
    {
        std::uint32_t x;
        std::uint32_t y;
    };

    /// @brief Philox4x32-10 随机数生成器（10 轮 Feistel 网络）
    ///
    /// 使用两个 32-bit 乘法常数和两个 Weyl 序列常数，经过 10 轮迭代生成
    /// 4 个 32-bit 的伪随机数。
    ///
    /// @param ctr 4 个 32-bit 的计数器输入（128-bit 状态）
    /// @param key 2 个 32-bit 的密钥输入（64-bit 密钥）
    /// @return 4 个 32-bit 的伪随机数输出
    RADAR_CUDA_HD inline Uint4 philox4x32_10(Uint4 ctr, Uint2 key)
    {
        // Philox 算法常数：乘法器 M0, M1 和 Weyl 序列增量 W0, W1
        constexpr std::uint32_t kPhiloxM0 = 0xD2511F53U;
        constexpr std::uint32_t kPhiloxM1 = 0xCD9E8D57U;
        constexpr std::uint32_t kPhiloxW0 = 0x9E3779B9U;
        constexpr std::uint32_t kPhiloxW1 = 0xBB67AE85U;

        for (int round = 0; round < 10; ++round)
        {
            // 64-bit 乘法：M0 * ctr.x 和 M1 * ctr.z
            const std::uint64_t p0 = static_cast<std::uint64_t>(kPhiloxM0) * ctr.x;
            const std::uint64_t p1 = static_cast<std::uint64_t>(kPhiloxM1) * ctr.z;
            // 拆分为低 32 位和高 32 位
            const std::uint32_t lo0 = static_cast<std::uint32_t>(p0);
            const std::uint32_t hi0 = static_cast<std::uint32_t>(p0 >> 32U);
            const std::uint32_t lo1 = static_cast<std::uint32_t>(p1);
            const std::uint32_t hi1 = static_cast<std::uint32_t>(p1 >> 32U);

            // 一轮 Feistel 更新
            ctr = Uint4{
                hi1 ^ ctr.y ^ key.x,
                lo1,
                hi0 ^ ctr.w ^ key.y,
                lo0,
            };

            // Weyl 序列更新密钥
            key.x += kPhiloxW0;
            key.y += kPhiloxW1;
        }

        return ctr;
    }

    /// @brief 将 32-bit 随机整数转换为 (0, 1) 开区间的均匀分布浮点数
    ///
    /// 使用 (bits + 0.5) / 2^32 的映射方式，保证输出在 (0, 1) 开区间内，
    /// 避免 log(0) 的数值问题。
    ///
    /// @param bits 32-bit 随机整数
    /// @return (0, 1) 范围内的均匀分布浮点数
    RADAR_CUDA_HD inline Scalar uniform_open_01(std::uint32_t bits)
    {
        constexpr Scalar kInvTwo32 = 1.0f / 4294967296.0f; // 1 / 2^32
        return (static_cast<Scalar>(bits) + 0.5f) * kInvTwo32;
    }

    /// @brief 基于 counter-based 方法生成 4 个 32-bit 随机数
    ///
    /// 将 (seed, range_idx, az_idx, channel_idx, source_id, sample_index) 编码为 Philox 的
    /// 计数器 ctr 和密钥 key，调用 philox4x32_10 生成确定性随机数。
    ///
    /// @param seed 64-bit 随机种子
    /// @param range_idx 距离单元索引
    /// @param az_idx 方位单元索引
    /// @param channel_idx 阵元/通道索引
    /// @param sample_index 样本/脉冲索引
    /// @param source_id 噪声源 ID（区分不同噪声源）
    /// @return 4 个 32-bit 的伪随机数
    RADAR_CUDA_HD inline Uint4 counter_random4(std::uint64_t seed,
                                               std::uint32_t range_idx,
                                               std::uint32_t az_idx,
                                               std::uint32_t channel_idx,
                                               std::uint64_t sample_index,
                                               std::uint32_t source_id)
    {
        // 将空间/通道/源/时间索引打包为 128-bit 计数器。
        // key:   seed / 蒙卡试验编号
        // ctr.x: bit[15:0] range_idx + bit[31:16] az_idx
        // ctr.y: bit[15:0] channel_idx + bit[23:16] source_id + bit[31:24] reserved
        // ctr.z: sample_index 低 32 位
        // ctr.w: sample_index 高 32 位
        // 空间索引各占 16 bit；调用方必须保证 range_idx 和 az_idx 不超过 65535。
        const Uint4 ctr{
            static_cast<std::uint32_t>(((az_idx & 0xFFFFU) << 16U) | (range_idx & 0xFFFFU)),
            static_cast<std::uint32_t>(((source_id & 0xFFU) << 16U) |
                                       (channel_idx & 0xFFFFU)),
            static_cast<std::uint32_t>(sample_index),
            static_cast<std::uint32_t>(sample_index >> 32U),
        };
        // 64-bit 种子作为密钥
        const Uint2 key{
            static_cast<std::uint32_t>(seed),
            static_cast<std::uint32_t>(seed >> 32U),
        };

        return philox4x32_10(ctr, key);
    }

    /// @brief 单通道便捷重载，保持当前海杂波调用点默认 channel_idx=0。
    RADAR_CUDA_HD inline Uint4 counter_random4(std::uint64_t seed,
                                               std::uint32_t range_idx,
                                               std::uint32_t az_idx,
                                               std::uint64_t sample_index,
                                               std::uint32_t source_id)
    {
        return counter_random4(seed, range_idx, az_idx, 0U, sample_index, source_id);
    }

    /// @brief 使用 Box-Muller 变换从两个 32-bit 随机数生成复高斯样本
    ///
    /// 实部随机数 -> 幅度 (Rayleigh 分布)，相位随机数 -> 相位 (均匀分布)，
    /// 合成为复高斯样本 Z = R * exp(j*θ)。
    ///
    /// @param real_bits 用于生成幅度的随机比特
    /// @param phase_bits 用于生成相位的随机比特
    /// @return 标准复高斯样本 (均值为 0，每维方差为 0.5)
    RADAR_CUDA_HD inline DeviceComplex complex_gaussian_from_uint2(std::uint32_t real_bits,
                                                                   std::uint32_t phase_bits)
    {
        const Scalar u0 = uniform_open_01(real_bits);   // 均匀分布 -> 幅度
        const Scalar u1 = uniform_open_01(phase_bits);  // 均匀分布 -> 相位
        constexpr Scalar kTwoPi = 2.0f * PI;
#ifdef __CUDA_ARCH__
        // GPU 端：使用 CUDA 内置数学函数以提高性能
        // fmaxf 避免 log(0)，1.1754943508222875e-38f 是 float 的最小正规数
        const Scalar radius = sqrtf(-logf(fmaxf(u0, 1.1754943508222875e-38f)));
        const Scalar phase = kTwoPi * u1;
        Scalar sin_phase;
        Scalar cos_phase;
        sincosf(phase, &sin_phase, &cos_phase); // 同时计算 sin 和 cos
        return DeviceComplex{radius * cos_phase, radius * sin_phase};
#else
        // CPU 端：使用标准 C++ 数学函数
        const Scalar radius = std::sqrt(-std::log(std::max(u0, 1.1754943508222875e-38f)));
        const Scalar phase = kTwoPi * u1;
        return DeviceComplex{radius * std::cos(phase), radius * std::sin(phase)};
#endif
    }

    /// @brief 生成单个复高斯样本（用于单个脉冲）
    ///
    /// @param seed 随机种子
    /// @param range_idx 距离单元索引
    /// @param az_idx 方位单元索引
    /// @param pulse_index 脉冲索引
    /// @param source_id 噪声源 ID
    /// @return 标准复高斯样本
    RADAR_CUDA_HD inline DeviceComplex counter_complex_gaussian_sample(std::uint64_t seed,
                                                                       std::uint32_t range_idx,
                                                                       std::uint32_t az_idx,
                                                                       std::uint32_t channel_idx,
                                                                       std::uint64_t pulse_index,
                                                                       std::uint32_t source_id)
    {
        const Uint4 rnd = counter_random4(seed, range_idx, az_idx, channel_idx, pulse_index, source_id);
        // 使用前两个 32-bit 输出生成复高斯样本
        return complex_gaussian_from_uint2(rnd.x, rnd.y);
    }

    /// @brief 单通道便捷重载。
    RADAR_CUDA_HD inline DeviceComplex counter_complex_gaussian_sample(std::uint64_t seed,
                                                                       std::uint32_t range_idx,
                                                                       std::uint32_t az_idx,
                                                                       std::uint64_t pulse_index,
                                                                       std::uint32_t source_id)
    {
        return counter_complex_gaussian_sample(seed, range_idx, az_idx, 0U, pulse_index, source_id);
    }

    /// @brief 生成一对复高斯样本（用于相邻的偶数/奇数脉冲对）
    ///
    /// 一次 Philox 调用输出 4 个 32-bit 值，分别用前两个和后两个生成
    /// 偶数脉冲和奇数脉冲的复高斯样本，提高 FIR 卷积的效率。
    ///
    /// @param seed 随机种子
    /// @param range_idx 距离单元索引
    /// @param az_idx 方位单元索引
    /// @param pair_index 脉冲对索引（脉冲号除以 2）
    /// @param source_id 噪声源 ID
    /// @return 一对标准复高斯样本
    RADAR_CUDA_HD inline DeviceComplexPair counter_complex_gaussian_sample_pair(std::uint64_t seed,
                                                                                std::uint32_t range_idx,
                                                                                std::uint32_t az_idx,
                                                                                std::uint32_t channel_idx,
                                                                                std::uint64_t pair_index,
                                                                                std::uint32_t source_id)
    {
        const Uint4 rnd = counter_random4(seed, range_idx, az_idx, channel_idx, pair_index, source_id);
        return DeviceComplexPair{
            complex_gaussian_from_uint2(rnd.x, rnd.y), // 偶数脉冲样本
            complex_gaussian_from_uint2(rnd.z, rnd.w), // 奇数脉冲样本
        };
    }

    /// @brief 单通道便捷重载。
    RADAR_CUDA_HD inline DeviceComplexPair counter_complex_gaussian_sample_pair(std::uint64_t seed,
                                                                                std::uint32_t range_idx,
                                                                                std::uint32_t az_idx,
                                                                                std::uint64_t pair_index,
                                                                                std::uint32_t source_id)
    {
        return counter_complex_gaussian_sample_pair(seed, range_idx, az_idx, 0U, pair_index, source_id);
    }

} // namespace radar::cuda

#undef RADAR_CUDA_HD
