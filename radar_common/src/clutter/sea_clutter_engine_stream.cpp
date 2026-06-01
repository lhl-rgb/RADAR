/**
 * @file sea_clutter_engine_stream.cpp
 * @brief 海杂波引擎异步双缓冲连续 PRT 生成实现
 *
 * 此文件实现了 generate_clutter_sequence_2d_fast_stream 方法，
 * 通过 CUDA Stream 异步双缓冲实现连续多个 PRT 的流水线并行生成。
 *
 * 设计动机：
 *   单 PRT 接口（get_clutter_for_pulse）每次调用都需要 GPU kernel 启动、
 *   执行、结果回传的完整同步周期，主机端在 GPU 执行期间空闲等待。
 *   流式批量接口通过双缓冲流水线将 GPU 计算与 CPU 准备工作重叠，
 *   显著降低连续 PRT 生成的总延迟。
 *
 * 双缓冲流水线示意（以 3 个 PRT 为例）：
 *   PRT 0: launch(slot=0) → GPU 异步执行...
 *   PRT 1: launch(slot=1) → GPU 异步执行...    wait(slot=0) → 取回 PRT 0 结果
 *   PRT 2: launch(slot=0) → GPU 异步执行...    wait(slot=1) → 取回 PRT 1 结果
 *   tail:                                      wait(slot=0) → 取回 PRT 2 结果
 *
 * 每个 slot 持有独立的：
 *   - CUDA Stream（异步执行流）
 *   - 俯仰幅度权重缓冲区（避免与单 PRT 路径共享权重冲突）
 *   - K 分布 texture 状态缓冲区（连续 PRT 间的纹理状态传递）
 *
 * 校验和累积：对每个取回的 PRT 结果计算 I² + Q² 的总和，
 * 用于回归测试验证生成结果的确定性（相同种子 + 相同波束序列 → 相同校验和）。
 */

#include "clutter/sea_clutter_engine.h"

#include "core/tools/math_utils.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>

namespace radar::clutter
{
    namespace
    {
        using Clock = std::chrono::steady_clock;

        /// @brief 计算两个时间点之间的毫秒间隔
        double elapsed_ms(const Clock::time_point &start, const Clock::time_point &finish)
        {
            return std::chrono::duration<double, std::milli>(finish - start).count();
        }

        /// @brief 累加 PRT 回波的校验和（I² + Q² 的总和）
        ///
        /// 用于回归测试：相同输入应产生相同的校验和，验证生成的确定性。
        void accumulate_checksum(const PulseEcho &echo, double &checksum)
        {
            for (const auto &sample : echo)
            {
                checksum += static_cast<double>(sample.real()) * sample.real() +
                            static_cast<double>(sample.imag()) * sample.imag();
            }
        }
    } // namespace

    /// @brief 异步双缓冲连续生成多个 PRT 的海杂波序列
    ///
    /// 通过 CUDA Stream 双缓冲实现主机端等待与 GPU 计算/传输的流水线重叠。
    /// 使用二维 fast 路径 kernel，支持 K 分布连续 PRT 纹理状态传递。
    ///
    /// 工作流程：
    /// 1. 初始化 CUDA Stream 双缓冲状态（两个 slot 各分配独立资源）
    /// 2. 逐 PRT 循环：
    ///    a. 若 submitted >= 2，等待两个 PRT 之前的结果（流水线已满）
    ///    b. 构建当前波束的活动方位 patch 列表
    ///    c. 重新计算俯仰幅度权重（stream 路径不使用缓存）
    ///    d. 组装 GPU 参数并异步启动 (launch)
    /// 3. 尾部收尾：等待最后两个已提交但未取回的 PRT 结果
    /// 4. 释放 stream 资源，使俯仰权重缓存失效（后续单 PRT 路径需重新上传）
    ///
    /// 限制：要求每个 PRT 至少有一个活跃 patch，否则返回失败。
    /// 这是因为 stream 路径的 slot 资源在无 patch 时无法正确完成流水线状态机。
    ///
    /// @param beams 波束指向数组（每个 PRT 一个 BeamPoint，长度 pulse_count）
    /// @param start_pulse_index 起始脉冲序号
    /// @param pulse_count 要生成的 PRT 数量
    /// @param[out] checksum 校验和（I² + Q² 总和），用于回归测试验证确定性
    /// @param[out] timing 可选，各阶段累计性能统计
    bool SeaClutterEngine::generate_clutter_sequence_2d_fast_stream(const BeamPoint *beams,
                                                                    std::uint64_t start_pulse_index,
                                                                    std::uint64_t pulse_count,
                                                                    double *checksum,
                                                                    SeaClutterStreamTiming *timing)
    {
        // ---- 初始化计时和校验和 ----
        if (timing != nullptr)
        {
            *timing = SeaClutterStreamTiming{};
        }
        if (checksum != nullptr)
        {
            *checksum = 0.0;
        }

        // ---- 参数校验 ----
        if (pulse_count <= 0)
        {
            return true; // 空请求，直接返回成功
        }
        if (beams == nullptr)
        {
            last_error_ = "generate_clutter_sequence_2d_fast_stream: beams is null";
            return false;
        }
        if (!cfg_.enabled)
        {
            return true; // 未启用时直接返回，不报错
        }
        if (!initialized_)
        {
            last_error_ = "SeaClutterEngine not initialized";
            return false;
        }

        // ---- 初始化 CUDA Stream 双缓冲状态 ----
        // 每个 slot 拥有独立的 CUDA Stream、俯仰权重 buffer 和 K texture 状态
        cuda::ClutterGpuStreamState stream_state;
        std::string gpu_error;
        if (!cuda::initialize_clutter_gpu_stream_state(stream_state, gpu_state_, &gpu_error))
        {
            last_error_ = "SeaClutter stream initialization failed: " + gpu_error;
            return false;
        }

        // 两个 slot 的回波缓冲区（各 PRT 的输出暂存于此，pinned memory 在 GPU 端写入）
        PulseEcho slot_echo[2];
        slot_echo[0].assign(static_cast<std::size_t>(num_range_cells_), Complex(0.0, 0.0));
        slot_echo[1].assign(static_cast<std::size_t>(num_range_cells_), Complex(0.0, 0.0));

        double local_checksum = 0.0;

        // RAII 清理函数：确保 stream 资源释放 + 俯仰缓存失效
        auto cleanup = [&]() {
            cuda::release_clutter_gpu_stream_state(stream_state);
            // Stream slot 使用了私有的俯仰权重缓冲区，与单 PRT 路径的共享权重不同。
            // 强制后续单 PRT 路径重新上传其共享的俯仰权重表。
            el_amplitude_weight_valid_ = false;
        };

        const auto run_start = Clock::now();
        int submitted = 0;

        // ---- 逐 PRT 流水线循环 ----
        // 双缓冲策略：slot 0 和 slot 1 交替使用
        // submitted=0 → slot=0, submitted=1 → slot=1, submitted=2 → slot=0, ...
        for (; submitted < pulse_count; ++submitted)
        {
            const int slot = submitted & 1; // ping-pong: 偶数用 slot 0，奇数用 slot 1

            // 流水线填满后（submitted >= 2），等待前面已完成的 PRT 结果
            // 此时前一个使用同一 slot 的 PRT 已经异步执行完毕
            if (submitted >= 2)
            {
                cuda::ClutterGpuTiming wait_timing;
                const auto wait_start = Clock::now();
                if (!cuda::wait_clutter_gpu_stream_slot(stream_state,
                                                        slot,
                                                        slot_echo[slot].data(),
                                                        &wait_timing,
                                                        &gpu_error))
                {
                    last_error_ = "SeaClutter stream wait failed: " + gpu_error;
                    cleanup();
                    return false;
                }
                const auto wait_finish = Clock::now();
                if (timing != nullptr)
                {
                    timing->gpu_wait_ms_sum += elapsed_ms(wait_start, wait_finish);
                    timing->host_convert_ms_sum += wait_timing.host_convert_ms;
                }
                // 累加校验和（I² + Q²）
                accumulate_checksum(slot_echo[slot], local_checksum);
            }

            // ---- 构建当前波束的活动方位 patch 列表 ----
            const auto active_start = Clock::now();
            const int active_count = build_active_patch_list(beams[submitted].azimuth_deg, active_patches_);
            const auto active_finish = Clock::now();
            last_active_count_ = active_count;
            if (timing != nullptr)
            {
                timing->active_build_ms_sum += elapsed_ms(active_start, active_finish);
                if (submitted == 0)
                {
                    timing->active_count_min = active_count;
                    timing->active_count_max = active_count;
                }
                else
                {
                    timing->active_count_min = std::min(timing->active_count_min, active_count);
                    timing->active_count_max = std::max(timing->active_count_max, active_count);
                }
                timing->active_count_sum += active_count;
            }

            // Stream 路径要求至少有一个活跃 patch（流水线状态机限制）
            if (active_count <= 0)
            {
                last_error_ = "SeaClutter stream path requires at least one active patch";
                cleanup();
                return false;
            }

            // ---- 组装 GPU 参数 ----
            const int pulse_index = start_pulse_index + submitted;
            const std::uint64_t pulse_u64 =
                pulse_index < 0 ? 0ULL : static_cast<std::uint64_t>(pulse_index);

            // 重新计算俯仰幅度权重（stream 路径不使用缓存，直接写入 slot 私有 buffer）
            build_elevation_amplitude_weight(beams[submitted].elevation_deg);

            cuda::ClutterPrtParams gpu_params{};
            gpu_params.pulse_index = pulse_u64;
            gpu_params.seed = cfg_.seed;
            gpu_params.peak_amplitude_weight = model_.peak_amplitude_weight();
            gpu_params.weibull_shape = cfg_.weibull_shape;
            gpu_params.weibull_scale = cfg_.weibull_scale;
            gpu_params.lognormal_mu = cfg_.lognormal_mu;
            gpu_params.lognormal_sigma = cfg_.lognormal_sigma;
            gpu_params.k_texture_rho = k_texture_rho_;
            gpu_params.k_texture_innovation_scale = k_texture_innovation_scale_;
            gpu_params.num_range_cells = num_range_cells_;
            gpu_params.active_count = active_count;
            gpu_params.k_lut_size = static_cast<int>(k_sqrt_tau_lut_.size());
            gpu_params.distribution = cfg_.distribution;

            // ---- 异步启动当前 PRT 的 GPU 计算 ----
            // previous_kernel_slot: 前一个 PRT 的 slot，用于保证 K texture 状态写入顺序
            // 第一个 PRT 没有前任，传 -1 表示跳过纹理状态同步
            if (!cuda::launch_clutter_prt_gpu_2d_fast_stream(gpu_state_,
                                                             stream_state,
                                                             slot,
                                                             submitted > 0 ? ((submitted - 1) & 1) : -1,
                                                             gpu_params,
                                                             active_patches_.data(),
                                                             el_amplitude_weight_.data(),
                                                             &gpu_error))
            {
                last_error_ = "SeaClutter stream launch failed: " + gpu_error;
                cleanup();
                return false;
            }
        }

        // ---- 尾部收尾：等待最后两个已提交但未取回的 PRT 结果 ----
        // 流水线中有最多 2 个 PRT 在飞（最后两个 slot），逐个等待并取回
        // 由于 slot 交替使用，最后两个 PRT 一定在 slot (submitted-2)&1 和 (submitted-1)&1 上
        for (int tail = std::max(0, submitted - 2); tail < submitted; ++tail)
        {
            const int slot = tail & 1;
            cuda::ClutterGpuTiming wait_timing;
            const auto wait_start = Clock::now();
            if (!cuda::wait_clutter_gpu_stream_slot(stream_state,
                                                    slot,
                                                    slot_echo[slot].data(),
                                                    &wait_timing,
                                                    &gpu_error))
            {
                last_error_ = "SeaClutter stream tail wait failed: " + gpu_error;
                cleanup();
                return false;
            }
            const auto wait_finish = Clock::now();
            if (timing != nullptr)
            {
                timing->gpu_wait_ms_sum += elapsed_ms(wait_start, wait_finish);
                timing->host_convert_ms_sum += wait_timing.host_convert_ms;
            }
            accumulate_checksum(slot_echo[slot], local_checksum);
        }

        // ---- 记录总耗时和校验和 ----
        // 整轮耗时 = 从 launch 第一个 PRT 到取回最后一个 PRT 的总时间
        const auto run_finish = Clock::now();
        if (timing != nullptr)
        {
            timing->run_ms = elapsed_ms(run_start, run_finish);
        }
        if (checksum != nullptr)
        {
            *checksum = local_checksum;
        }

        cleanup();
        return true;
    }

} // namespace radar::clutter
