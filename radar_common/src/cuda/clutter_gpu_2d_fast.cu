/// @file clutter_gpu_2d_fast.cu
/// @brief 二维 range×patch 并行海杂波 fast 实验路径
///
/// 与 clutter_gpu_2d.cu 的区别：
/// - K 分布 texture AR(1) 更新增加了 delta == 1 的快速路径：
///   直接使用 k_texture_rho 和 k_texture_innovation_scale（已在 CPU 端预计算）
///   避免了 powf() 和 sqrtf() 的额外开销
/// - 校验逻辑中增加了 k_texture_innovation_scale 的有效性检查
///
/// 适用场景：连续 PRT 场景（delta 通常为 1）下 K 分布海杂波生成，
/// 能显著减少 GPU 内核中的数学函数调用开销。

#include "cuda/clutter_gpu.cuh"

#include <algorithm>
#include <chrono>
#include <cuda_runtime.h>
#include <string>
#include <vector>

namespace radar::cuda
{
    namespace
    {
        constexpr std::uint32_t kClutterWhiteSourceId = 0U;
        constexpr std::uint32_t kTextureInitSourceId = 1U;
        constexpr std::uint32_t kTextureJumpSourceId = 2U;
        constexpr Scalar kSqrt2 = 1.4142135623730951f;
        constexpr std::uint64_t kTexturePulseUnset = UINT64_MAX;

        using Clock = std::chrono::steady_clock;

        double elapsed_ms(const Clock::time_point &start, const Clock::time_point &finish)
        {
            return std::chrono::duration<double, std::milli>(finish - start).count();
        }

        const char *cuda_error_string(cudaError_t error)
        {
            return cudaGetErrorString(error);
        }

        bool set_cuda_error(cudaError_t cuda_error, const char *context, std::string *error)
        {
            if (cuda_error == cudaSuccess)
            {
                return true;
            }
            if (error != nullptr)
            {
                *error = std::string(context) + ": " + cuda_error_string(cuda_error);
            }
            return false;
        }

        /// cudaMalloc 使用 void** 接口；将必要的转换集中在分配函数中。
        template <typename T>
        bool allocate_device(T *&ptr, std::size_t bytes, const char *context, std::string *error)
        {
            return set_cuda_error(
                cudaMalloc(reinterpret_cast<void **>(&ptr), bytes),
                context,
                error);
        }

        __device__ __forceinline__ void accumulate_fir_tap_2d(const DeviceComplex h,
                                                              const DeviceComplex white,
                                                              Scalar &accum_re,
                                                              Scalar &accum_im)
        {
            accum_re += h.real * white.real - h.imag * white.imag;
            accum_im += h.real * white.imag + h.imag * white.real;
        }

        __device__ __forceinline__ Scalar standard_real_gaussian_2d(std::uint64_t seed,
                                                                    std::uint32_t range_idx,
                                                                    std::uint32_t az_idx,
                                                                    std::uint64_t pulse_index,
                                                                    std::uint32_t source_id)
        {
            const DeviceComplex sample =
                counter_complex_gaussian_sample(seed, range_idx, az_idx, pulse_index, source_id);
            return kSqrt2 * sample.real;
        }

        __device__ __forceinline__ Scalar lookup_k_sqrt_tau_2d(const Scalar *lut,
                                                               int lut_size,
                                                               Scalar z)
        {
            if (lut == nullptr || lut_size <= 1)
            {
                return 1.0f;
            }

            const Scalar z_clamped = fminf(fmaxf(z, -8.0f), 8.0f);
            const Scalar x = (z_clamped + 8.0f) *
                             (static_cast<Scalar>(lut_size - 1) / 16.0f);
            int i = static_cast<int>(floorf(x));
            i = max(0, min(i, lut_size - 2));
            const Scalar frac = x - static_cast<Scalar>(i);
            return lut[i] * (1.0f - frac) + lut[i + 1] * frac;
        }

        /// @brief K 分布 texture AR(1) 状态更新（含 delta == 1 快速路径）
        ///
        /// 与标准版本 (update_k_texture_ar1_2d) 的区别：
        /// - delta == 1 时使用 CPU 端预计算的 k_texture_rho 和 k_texture_innovation_scale
        ///   直接进行 AR(1) 递推，避免了 powf(ρ, 1) 和 sqrtf(1-ρ²) 的 GPU 端计算
        /// - 连续 PRT 场景下（delta 通常为 1），此优化能显著提升性能
        ///
        /// 当 delta != 1 时（如非连续 PRT），回退到通用路径计算 ρ^δ 和 sqrt(1-ρ^(2δ))。
        __device__ __forceinline__ Scalar update_k_texture_ar1_2d_fast(Scalar *texture_state,
                                                                       std::uint64_t *last_pulse,
                                                                       const Scalar *sqrt_tau_lut,
                                                                       ClutterPrtParams params,
                                                                       int range_idx,
                                                                       int az_idx)
        {
            // 快速路径：K 分布未启用时返回 1.0
            if (texture_state == nullptr || last_pulse == nullptr || sqrt_tau_lut == nullptr ||
                params.k_lut_size <= 1 || params.k_texture_rho <= 0.0f)
            {
                return 1.0f;
            }

            const std::size_t state_idx =
                row_major_index(az_idx, range_idx, params.num_range_cells);
            const std::uint64_t current_pulse = params.pulse_index;
            const std::uint64_t previous_pulse = last_pulse[state_idx];

            Scalar z = 0.0f;
            if (previous_pulse == kTexturePulseUnset || previous_pulse > current_pulse)
            {
                // 首次访问或 pulse 回绕：标准正态初始化
                z = standard_real_gaussian_2d(params.seed,
                                              static_cast<std::uint32_t>(range_idx),
                                              static_cast<std::uint32_t>(az_idx),
                                              current_pulse,
                                              kTextureInitSourceId);
            }
            else
            {
                const std::uint64_t delta = current_pulse - previous_pulse;
                if (delta == 0ULL)
                {
                    // 同一 pulse 重复访问：直接复用
                    z = texture_state[state_idx];
                }
                else if (delta == 1ULL)
                {
                    // 【快速路径】连续 PRT：z = ρ × z_prev + σ_innov × N(0,1)
                    // ρ 和 σ_innov 已在 CPU 端预计算，无需 GPU 端 powf/sqrtf
                    const Scalar innovation =
                        standard_real_gaussian_2d(params.seed,
                                                  static_cast<std::uint32_t>(range_idx),
                                                  static_cast<std::uint32_t>(az_idx),
                                                  current_pulse,
                                                  kTextureJumpSourceId);
                    z = params.k_texture_rho * texture_state[state_idx] +
                        params.k_texture_innovation_scale * innovation;
                }
                else
                {
                    // 【通用路径】非连续 PRT：z = ρ^δ × z_prev + sqrt(1-ρ^(2δ)) × N(0,1)
                    const Scalar rho_delta =
                        powf(fminf(fmaxf(params.k_texture_rho, 0.0f), 0.99999994f),
                             static_cast<Scalar>(delta));
                    const Scalar innovation_scale =
                        sqrtf(fmaxf(1.0f - rho_delta * rho_delta, 0.0f));
                    const Scalar innovation =
                        standard_real_gaussian_2d(params.seed,
                                                  static_cast<std::uint32_t>(range_idx),
                                                  static_cast<std::uint32_t>(az_idx),
                                                  current_pulse,
                                                  kTextureJumpSourceId);
                    z = rho_delta * texture_state[state_idx] + innovation_scale * innovation;
                }
            }

            // 持久化状态
            texture_state[state_idx] = z;
            last_pulse[state_idx] = current_pulse;
            return lookup_k_sqrt_tau_2d(sqrt_tau_lut, params.k_lut_size, z);
        }

        __device__ __forceinline__ void apply_distribution_transform_2d(ClutterPrtParams params,
                                                                        Scalar &sample_re,
                                                                        Scalar &sample_im)
        {
            if (params.distribution == ClutterDistribution::Rayleigh)
            {
                return;
            }

            const Scalar power = sample_re * sample_re + sample_im * sample_im;
            if (power <= 1.1754943508222875e-38f)
            {
                sample_re = 0.0f;
                sample_im = 0.0f;
                return;
            }

            const Scalar amplitude = sqrtf(power);
            Scalar target_amplitude = amplitude;

            if (params.distribution == ClutterDistribution::Weibull)
            {
                const Scalar safe_shape = fmaxf(params.weibull_shape, EPSILON);
                const Scalar safe_scale = fmaxf(params.weibull_scale, EPSILON);
                target_amplitude =
                    (safe_scale / powf(2.0f, 1.0f / safe_shape)) *
                    powf(power, 1.0f / safe_shape);
            }
            else if (params.distribution == ClutterDistribution::LogNormal)
            {
                const Scalar gaussian_driver = kSqrt2 * sample_re;
                target_amplitude = expf(params.lognormal_mu + params.lognormal_sigma * gaussian_driver);
            }

            const Scalar factor = target_amplitude / amplitude;
            sample_re *= factor;
            sample_im *= factor;
        }

        __global__ void clutter_patch_contrib_kernel_2d_fast(const Scalar *patch_amplitude_map,
                                                        const Scalar *el_amplitude_weight,
                                                        const ClutterActivePatch *active_patches,
                                                        const DeviceComplex *fir_coeffs,
                                                        DeviceComplex *patch_contrib,
                                                        Scalar *k_texture_state,
                                                        std::uint64_t *k_texture_last_pulse,
                                                        const Scalar *k_sqrt_tau_lut,
                                                        ClutterPrtParams params,
                                                        int fir_length,
                                                        int active_stride)
        {
            const int range_idx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
            const int active_idx = static_cast<int>(blockIdx.y);
            if (range_idx >= params.num_range_cells || active_idx >= params.active_count)
            {
                return;
            }

            const std::size_t out_idx =
                row_major_index(active_idx, range_idx, active_stride);

            const ClutterActivePatch active = active_patches[active_idx];
            if (active.az_amplitude_weight <= 0.0f)
            {
                patch_contrib[out_idx] = DeviceComplex{0.0f, 0.0f};
                return;
            }

            const std::size_t amplitude_idx =
                row_major_index(active.az_idx, range_idx, params.num_range_cells);
            const Scalar patch_amplitude = patch_amplitude_map[amplitude_idx];
            const Scalar range_scale =
                params.peak_amplitude_weight * el_amplitude_weight[range_idx] * patch_amplitude;
            if (range_scale <= 0.0f)
            {
                patch_contrib[out_idx] = DeviceComplex{0.0f, 0.0f};
                return;
            }

            Scalar shaped_re = 0.0f;
            Scalar shaped_im = 0.0f;
            int tap = 0;

            // ---- FIR 时间相关性卷积 ----
            // 数学关系固定为 y[n] = Σ h[k] * w[n-k]，FIR 长度为 L 时需要 L 个历史样本。
            // RNG pair 只是取样优化：
            //   pair_index = pulse / 2
            //   pair.even  = w[2 * pair_index]
            //   pair.odd   = w[2 * pair_index + 1]
            // 偶数 n 的 tap0 单独处理；之后 tap1/tap2、tap3/tap4... 按 odd/even 成对。
            // 奇数 n 从 tap0/tap1 开始即可按 odd/even 成对。
            if ((params.pulse_index & 1ULL) == 0ULL)
            {
                const DeviceComplexPair white_pair = counter_complex_gaussian_sample_pair(
                    params.seed,
                    static_cast<std::uint32_t>(range_idx),
                    static_cast<std::uint32_t>(active.az_idx),
                    params.pulse_index >> 1U,
                    kClutterWhiteSourceId);
                accumulate_fir_tap_2d(fir_coeffs[tap], white_pair.even, shaped_re, shaped_im);
                ++tap;
            }

            // 进入本循环时，tap 对应的历史脉冲总是奇数：
            //   tap   -> pair.odd
            //   tap+1 -> pair.even
            for (; tap + 1 < fir_length; tap += 2)
            {
                const std::uint64_t tap_pulse0 =
                    wrapped_history_pulse_index(params.pulse_index, tap);
                const DeviceComplexPair white_pair = counter_complex_gaussian_sample_pair(
                    params.seed,
                    static_cast<std::uint32_t>(range_idx),
                    static_cast<std::uint32_t>(active.az_idx),
                    tap_pulse0 >> 1U,
                    kClutterWhiteSourceId);
                accumulate_fir_tap_2d(fir_coeffs[tap], white_pair.odd, shaped_re, shaped_im);
                accumulate_fir_tap_2d(fir_coeffs[tap + 1], white_pair.even, shaped_re, shaped_im);
            }

            // 当前 FIR 长度会被初始化逻辑规范为奇数。
            // 因此这里通常只出现在当前 n 为奇数时，剩余最后一个未成对 tap。
            // 下面仍按 pair 规则选择 lane，避免未来放开偶数 FIR 长度时引入隐含假设。
            if (tap < fir_length)
            {
                const std::uint64_t tap_pulse =
                    wrapped_history_pulse_index(params.pulse_index, tap);
                const DeviceComplexPair white_pair = counter_complex_gaussian_sample_pair(
                    params.seed,
                    static_cast<std::uint32_t>(range_idx),
                    static_cast<std::uint32_t>(active.az_idx),
                    tap_pulse >> 1U,
                    kClutterWhiteSourceId);
                const DeviceComplex white = ((tap_pulse & 1ULL) == 0ULL) ? white_pair.even : white_pair.odd;
                accumulate_fir_tap_2d(fir_coeffs[tap], white, shaped_re, shaped_im);
            }

            if (params.distribution == ClutterDistribution::K)
            {
                const Scalar sqrt_tau = update_k_texture_ar1_2d_fast(k_texture_state,
                                                                      k_texture_last_pulse,
                                                                      k_sqrt_tau_lut,
                                                                      params,
                                                                      range_idx,
                                                                      active.az_idx);
                shaped_re *= sqrt_tau;
                shaped_im *= sqrt_tau;
            }
            else
            {
                apply_distribution_transform_2d(params, shaped_re, shaped_im);
            }

            const Scalar scale = range_scale * active.az_amplitude_weight;
            patch_contrib[out_idx] = DeviceComplex{scale * shaped_re, scale * shaped_im};
        }

        __global__ void clutter_patch_reduce_kernel_2d_fast(const DeviceComplex *patch_contrib,
                                                       DeviceComplex *echo,
                                                       ClutterPrtParams params,
                                                       int active_stride)
        {
            const int range_idx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
            if (range_idx >= params.num_range_cells)
            {
                return;
            }

            Scalar accum_re = 0.0f;
            Scalar accum_im = 0.0f;
            for (int active_idx = 0; active_idx < params.active_count; ++active_idx)
            {
                const std::size_t in_idx =
                    row_major_index(active_idx, range_idx, active_stride);
                const DeviceComplex sample = patch_contrib[in_idx];
                accum_re += sample.real;
                accum_im += sample.imag;
            }

            echo[range_idx] = DeviceComplex{accum_re, accum_im};
        }
    } // namespace

    bool generate_clutter_prt_gpu_2d_fast(ClutterGpuState &state,
                                     const ClutterPrtParams &params,
                                     const ClutterActivePatch *active_patches,
                                     Complex *out_echo,
                                     ClutterGpuTiming *timing,
                                     std::string *error)
    {
        if (timing != nullptr)
        {
            *timing = ClutterGpuTiming{};
        }

        const auto total_start = Clock::now();

        if (!state.initialized ||
            state.d_patch_amplitude_map == nullptr || state.d_el_amplitude_weight == nullptr ||
            state.d_active_patches == nullptr || state.d_fir_coeffs == nullptr ||
            state.d_echo == nullptr)
        {
            if (error != nullptr)
            {
                *error = "generate_clutter_prt_gpu_2d_fast: state is not initialized";
            }
            return false;
        }

        if (out_echo == nullptr || params.num_range_cells <= 0 ||
            params.num_range_cells != state.num_range_cells ||
            state.fir_length <= 0 ||
            params.active_count < 0 ||
            params.active_count > state.active_patch_capacity ||
            (params.distribution == ClutterDistribution::K &&
             (state.d_k_texture_state == nullptr ||
              state.d_k_texture_last_pulse == nullptr ||
              state.d_k_sqrt_tau_lut == nullptr ||
              params.k_lut_size != state.k_lut_size ||
              params.k_texture_rho <= 0.0f ||
              params.k_texture_innovation_scale < 0.0f)) ||
            (params.active_count > 0 && active_patches == nullptr))
        {
            if (error != nullptr)
            {
                *error = "generate_clutter_prt_gpu_2d_fast: invalid output or size mismatch";
            }
            return false;
        }

        if (params.active_count <= 0)
        {
            std::fill(out_echo, out_echo + params.num_range_cells, Complex(0.0f, 0.0f));
            return true;
        }

        const std::size_t contrib_count =
            static_cast<std::size_t>(state.num_range_cells) *
            static_cast<std::size_t>(state.active_patch_capacity);
        const std::size_t contrib_bytes = contrib_count * sizeof(DeviceComplex);
        if (state.d_patch_contrib == nullptr)
        {
            if (!allocate_device(state.d_patch_contrib,
                                 contrib_bytes,
                                 "cudaMalloc d_patch_contrib",
                                 error))
            {
                return false;
            }
        }

        const auto upload_start = Clock::now();
        const std::size_t active_bytes =
            static_cast<std::size_t>(params.active_count) * sizeof(ClutterActivePatch);
        auto cuda_error = cudaMemcpy(state.d_active_patches,
                                     active_patches,
                                     active_bytes,
                                     cudaMemcpyHostToDevice);
        const auto upload_finish = Clock::now();
        if (timing != nullptr)
        {
            timing->active_upload_ms = elapsed_ms(upload_start, upload_finish);
        }
        if (!set_cuda_error(cuda_error, "cudaMemcpy d_active_patches", error))
        {
            return false;
        }

        constexpr int kThreadsPerBlock = 256;
        const dim3 contrib_blocks((params.num_range_cells + kThreadsPerBlock - 1) / kThreadsPerBlock,
                                  params.active_count,
                                  1);
        const int reduce_blocks = (params.num_range_cells + kThreadsPerBlock - 1) / kThreadsPerBlock;

        const auto kernel_start = Clock::now();
        clutter_patch_contrib_kernel_2d_fast<<<contrib_blocks, kThreadsPerBlock>>>(
            state.d_patch_amplitude_map,
            state.d_el_amplitude_weight,
            state.d_active_patches,
            state.d_fir_coeffs,
            state.d_patch_contrib,
            state.d_k_texture_state,
            state.d_k_texture_last_pulse,
            state.d_k_sqrt_tau_lut,
            params,
            state.fir_length,
            state.num_range_cells);

        cuda_error = cudaGetLastError();
        if (!set_cuda_error(cuda_error, "clutter_patch_contrib_kernel_2d_fast launch", error))
        {
            return false;
        }

        clutter_patch_reduce_kernel_2d_fast<<<reduce_blocks, kThreadsPerBlock>>>(
            state.d_patch_contrib,
            state.d_echo,
            params,
            state.num_range_cells);

        cuda_error = cudaGetLastError();
        if (!set_cuda_error(cuda_error, "clutter_patch_reduce_kernel_2d_fast launch", error))
        {
            return false;
        }

        cuda_error = cudaDeviceSynchronize();
        const auto kernel_finish = Clock::now();
        if (timing != nullptr)
        {
            timing->kernel_sync_ms = elapsed_ms(kernel_start, kernel_finish);
        }
        if (!set_cuda_error(cuda_error, "clutter_gpu_2d_fast synchronize", error))
        {
            return false;
        }

        std::vector<DeviceComplex> host_echo(static_cast<std::size_t>(params.num_range_cells));
        const std::size_t echo_bytes =
            static_cast<std::size_t>(params.num_range_cells) * sizeof(DeviceComplex);
        const auto copy_start = Clock::now();
        cuda_error = cudaMemcpy(host_echo.data(), state.d_echo, echo_bytes, cudaMemcpyDeviceToHost);
        const auto copy_finish = Clock::now();
        if (timing != nullptr)
        {
            timing->device_to_host_ms = elapsed_ms(copy_start, copy_finish);
        }
        if (!set_cuda_error(cuda_error, "cudaMemcpy clutter echo 2d", error))
        {
            return false;
        }

        const auto convert_start = Clock::now();
        for (int i = 0; i < params.num_range_cells; ++i)
        {
            const auto &sample = host_echo[static_cast<std::size_t>(i)];
            out_echo[static_cast<std::size_t>(i)] = Complex(sample.real, sample.imag);
        }
        const auto convert_finish = Clock::now();

        if (timing != nullptr)
        {
            timing->host_convert_ms = elapsed_ms(convert_start, convert_finish);
            timing->total_ms = elapsed_ms(total_start, convert_finish);
        }

        return true;
    }

} // namespace radar::cuda
