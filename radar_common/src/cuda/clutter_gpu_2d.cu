/// @file clutter_gpu_2d.cu
/// @brief 二维 range×patch 并行海杂波实验路径
///
/// 与一维版本 (clutter_gpu.cu) 的区别：
/// - 一维版本：每个线程负责一个距离单元，内层循环遍历所有活跃方位补丁
/// - 二维版本：grid 扩展为 2D (range × active_patch)，每个线程只处理一个 (range, patch) 对
///   然后通过归约内核将同一距离单元的所有 patch 贡献求和
///
/// 二维版本的优势：
/// - 活跃补丁数量多时，二维 grid 能提供更高的并行度
/// - 中间贡献缓冲可复用，减少对常量内存的竞争
///
/// 代价：
/// - 需要额外的归约步骤（两个内核）
/// - 中间缓冲占用更多显存 (num_range × max_active_patch × sizeof(DeviceComplex))

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

        /// @brief FIR 抽头累加：accum += h × white（复数乘法）
        ///
        /// 将 FIR 系数 h 与白噪声样本 white 进行复数乘法后累加到 accum。
        /// 使用 __forceinline__ 确保在 kernel 中被内联展开，避免函数调用开销。
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

        __device__ __forceinline__ Scalar update_k_texture_ar1_2d(Scalar *texture_state,
                                                                  std::uint64_t *last_pulse,
                                                                  const Scalar *sqrt_tau_lut,
                                                                  ClutterPrtParams params,
                                                                  int range_idx,
                                                                  int az_idx)
        {
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
                    z = texture_state[state_idx];
                }
                else
                {
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

        /// @brief GPU 内核：计算每个 (range, patch) 对的贡献（二维并行版本）
        ///
        /// 二维 grid 布局：
        /// - blockIdx.x: 距离单元维度
        /// - blockIdx.y: 活跃方位补丁维度
        /// 每个线程处理一个 (range_idx, active_idx) 对，执行完整的 FIR 卷积 + 分布变换。
        ///
        /// 结果写入 patch_contrib[active_idx][range_idx]，供后续归约内核沿 patch 维度求和。
        ///
        /// @param fir_coeffs FIR 系数（全局内存，非常量内存），供 2D 实验路径独立测试
        /// @param patch_contrib 输出缓冲区 [active_count][active_stride]
        /// @param active_stride patch_contrib 的行步长（通常为 num_range_cells）
        __global__ void clutter_patch_contrib_kernel_2d(const Scalar *patch_amplitude_map,
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
            // 二维 grid 索引：x → range, y → active patch
            const int range_idx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
            const int active_idx = static_cast<int>(blockIdx.y);
            if (range_idx >= params.num_range_cells || active_idx >= params.active_count)
            {
                return;
            }

            // 中间贡献输出的扁平索引：[active_idx][range_idx]
            const std::size_t out_idx =
                row_major_index(active_idx, range_idx, active_stride);

            const ClutterActivePatch active = active_patches[active_idx];
            if (active.az_amplitude_weight <= 0.0f)
            {
                patch_contrib[out_idx] = DeviceComplex{0.0f, 0.0f};
                return;
            }

            // 查 patch 幅度图获取 sqrt(P)
            const std::size_t amplitude_idx =
                row_major_index(active.az_idx, range_idx, params.num_range_cells);
            const Scalar patch_amplitude = patch_amplitude_map[amplitude_idx];
            // 距离向总缩放 = 峰值增益 × 俯仰权重 × sqrt(P)
            const Scalar range_scale =
                params.peak_amplitude_weight * el_amplitude_weight[range_idx] * patch_amplitude;
            if (range_scale <= 0.0f)
            {
                patch_contrib[out_idx] = DeviceComplex{0.0f, 0.0f};
                return;
            }

            // ---- FIR 时间相关性卷积 ----
            // 数学关系固定为 y[n] = Σ h[k] * w[n-k]，FIR 长度为 L 时需要 L 个历史样本。
            // RNG pair 只是取样优化：
            //   pair_index = pulse / 2
            //   pair.even  = w[2 * pair_index]
            //   pair.odd   = w[2 * pair_index + 1]
            // 偶数 n 的 tap0 单独处理；之后 tap1/tap2、tap3/tap4... 按 odd/even 成对。
            // 奇数 n 从 tap0/tap1 开始即可按 odd/even 成对。
            Scalar shaped_re = 0.0f;
            Scalar shaped_im = 0.0f;
            int tap = 0;

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

            // ---- 幅度分布变换 ----
            if (params.distribution == ClutterDistribution::K)
            {
                const Scalar sqrt_tau = update_k_texture_ar1_2d(k_texture_state,
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

            // ---- 方位方向图加权，写入中间贡献 ----
            const Scalar scale = range_scale * active.az_amplitude_weight;
            patch_contrib[out_idx] = DeviceComplex{scale * shaped_re, scale * shaped_im};
        }

        /// @brief GPU 内核：归约 — 沿 patch 维度对贡献求和得到距离向回波
        ///
        /// 每个线程负责一个距离单元，遍历所有活跃方位补丁的贡献并累加。
        /// 这是二维并行路径的第二阶段（归约阶段）。
        ///
        /// z(r) = Σ_{active} patch_contrib[active][r]
        __global__ void clutter_patch_reduce_kernel_2d(const DeviceComplex *patch_contrib,
                                                       DeviceComplex *echo,
                                                       ClutterPrtParams params,
                                                       int active_stride)
        {
            const int range_idx = static_cast<int>(blockIdx.x * blockDim.x + threadIdx.x);
            if (range_idx >= params.num_range_cells)
            {
                return;
            }

            Scalar accum_re = 0.0f; // 实部累加器
            Scalar accum_im = 0.0f; // 虚部累加器
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

    /// @brief 二维 range×patch 并行路径：在 GPU 上按单个 PRT 生成海杂波回波
    ///
    /// 完整流程（两阶段内核）：
    /// 1. 参数校验：状态初始化、输出缓冲区、尺寸匹配、K 分布参数完整性
    /// 2. 上传活跃方位补丁列表到设备端
    /// 3. 第一阶段 — clutter_patch_contrib_kernel_2d：
    ///    二维 grid (range × active)，每个线程计算一个 (range, patch) 对的贡献
    /// 4. 第二阶段 — clutter_patch_reduce_kernel_2d：
    ///    一维 grid (range)，每个线程沿 patch 维度归约求和
    /// 5. 同步等待两个内核完成
    /// 6. 结果从设备端拷贝回主机端
    /// 7. 格式转换：DeviceComplex → Complex
    ///
    /// 适用场景：活跃补丁数量多时（>~16），二维版本比一维版本有更好的并行度。
    bool generate_clutter_prt_gpu_2d(ClutterGpuState &state,
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

        // =====================================================================
        // 阶段 1: 参数校验
        // =====================================================================
        if (!state.initialized ||
            state.d_patch_amplitude_map == nullptr || state.d_el_amplitude_weight == nullptr ||
            state.d_active_patches == nullptr || state.d_fir_coeffs == nullptr ||
            state.d_echo == nullptr)
        {
            if (error != nullptr)
            {
                *error = "generate_clutter_prt_gpu_2d: state is not initialized";
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
              params.k_texture_rho <= 0.0f)) ||
            (params.active_count > 0 && active_patches == nullptr))
        {
            if (error != nullptr)
            {
                *error = "generate_clutter_prt_gpu_2d: invalid output or size mismatch";
            }
            return false;
        }

        // 无活跃补丁时直接填零返回
        if (params.active_count <= 0)
        {
            std::fill(out_echo, out_echo + params.num_range_cells, Complex(0.0f, 0.0f));
            return true;
        }

        // 按需分配二维中间贡献缓冲区（首次分配，后续 PRT 复用）
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

        // =====================================================================
        // 阶段 2: 上传活跃方位补丁
        // =====================================================================
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

        // =====================================================================
        // 阶段 3: 启动 GPU 内核
        // =====================================================================
        constexpr int kThreadsPerBlock = 256;
        // 二维 grid: x=range, y=active_patch
        const dim3 contrib_blocks((params.num_range_cells + kThreadsPerBlock - 1) / kThreadsPerBlock,
                                  params.active_count,
                                  1);
        const int reduce_blocks = (params.num_range_cells + kThreadsPerBlock - 1) / kThreadsPerBlock;

        const auto kernel_start = Clock::now();

        // 3a. 第一阶段内核：计算每个 (range, patch) 对的贡献
        clutter_patch_contrib_kernel_2d<<<contrib_blocks, kThreadsPerBlock>>>(
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
        if (!set_cuda_error(cuda_error, "clutter_patch_contrib_kernel_2d launch", error))
        {
            return false;
        }

        // 3b. 第二阶段内核：沿 patch 维度归约求和
        clutter_patch_reduce_kernel_2d<<<reduce_blocks, kThreadsPerBlock>>>(
            state.d_patch_contrib,
            state.d_echo,
            params,
            state.num_range_cells);

        cuda_error = cudaGetLastError();
        if (!set_cuda_error(cuda_error, "clutter_patch_reduce_kernel_2d launch", error))
        {
            return false;
        }

        // 同步等待两个内核执行完成
        cuda_error = cudaDeviceSynchronize();
        const auto kernel_finish = Clock::now();
        if (timing != nullptr)
        {
            timing->kernel_sync_ms = elapsed_ms(kernel_start, kernel_finish);
        }
        if (!set_cuda_error(cuda_error, "clutter_gpu_2d synchronize", error))
        {
            return false;
        }

        // =====================================================================
        // 阶段 4: 结果拷贝回主机端
        // =====================================================================
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

        // =====================================================================
        // 阶段 5: 格式转换 DeviceComplex → Complex
        // =====================================================================
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
