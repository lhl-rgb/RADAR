/// @file clutter_gpu_stream.cu
/// @brief 2D-fast 海杂波连续 PRT 异步双缓冲实验路径
///
/// 设计目标：通过 CUDA Stream 异步执行 + 双缓冲 (ping-pong) 实现连续 PRT 的流水线并行。
///
/// 工作流程：
/// 1. initialize_clutter_gpu_stream_state:
///    为两个 slot 各分配独立的设备内存、pinned 主机内存、CUDA stream 和 event。
///    - slot 0 和 slot 1 交替使用（ping-pong）
///    - pinned 主机内存 (cudaHostAlloc) 实现高速的异步 DMA 传输
/// 2. launch_clutter_prt_gpu_2d_fast_stream:
///    在指定 stream 上异步执行：活跃补丁上传 → 贡献内核 → 归约内核 → 结果回传
///    - 通过 cudaEvent 保证前一个 PRT 的内核在 stream 上完成后再启动当前 PRT
/// 3. wait_clutter_gpu_stream_slot:
///    在主机端同步等待指定 slot 的 copy_done event，然后转换格式
///
/// 双缓冲流水线示意（连续 3 个 PRT）：
///   PRT 0: launch(slot=0) → GPU 异步执行...
///   PRT 1: launch(slot=1) → GPU 异步执行...
///          wait(slot=0)    → 取回 PRT 0 结果
///   PRT 2: launch(slot=0) → GPU 异步执行...
///          wait(slot=1)    → 取回 PRT 1 结果
///
/// 优势：主机端等待与 GPU 计算/传输重叠，减少空闲时间。

#include "cuda/clutter_gpu.cuh"

#include <algorithm>
#include <chrono>
#include <cuda_runtime.h>
#include <string>

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

        /// cudaMalloc/cudaHostAlloc 使用 void** 接口；将必要的转换集中在分配函数中。
        template <typename T>
        bool allocate_device(T *&ptr, std::size_t bytes, const char *context, std::string *error)
        {
            return set_cuda_error(
                cudaMalloc(reinterpret_cast<void **>(&ptr), bytes),
                context,
                error);
        }

        template <typename T>
        bool allocate_pinned_host(T *&ptr, std::size_t bytes, const char *context, std::string *error)
        {
            return set_cuda_error(
                cudaHostAlloc(reinterpret_cast<void **>(&ptr), bytes, cudaHostAllocDefault),
                context,
                error);
        }

        __device__ __forceinline__ void accumulate_fir_tap_stream(const DeviceComplex h,
                                                                  const DeviceComplex white,
                                                                  Scalar &accum_re,
                                                                  Scalar &accum_im)
        {
            accum_re += h.real * white.real - h.imag * white.imag;
            accum_im += h.real * white.imag + h.imag * white.real;
        }

        __device__ __forceinline__ Scalar standard_real_gaussian_stream(std::uint64_t seed,
                                                                        std::uint32_t range_idx,
                                                                        std::uint32_t az_idx,
                                                                        std::uint64_t pulse_index,
                                                                        std::uint32_t source_id)
        {
            const DeviceComplex sample =
                counter_complex_gaussian_sample(seed, range_idx, az_idx, pulse_index, source_id);
            return kSqrt2 * sample.real;
        }

        __device__ __forceinline__ Scalar lookup_k_sqrt_tau_stream(const Scalar *lut,
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

        __device__ __forceinline__ Scalar update_k_texture_ar1_stream(Scalar *texture_state,
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
                z = standard_real_gaussian_stream(params.seed,
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
                else if (delta == 1ULL)
                {
                    const Scalar innovation =
                        standard_real_gaussian_stream(params.seed,
                                                      static_cast<std::uint32_t>(range_idx),
                                                      static_cast<std::uint32_t>(az_idx),
                                                      current_pulse,
                                                      kTextureJumpSourceId);
                    z = params.k_texture_rho * texture_state[state_idx] +
                        params.k_texture_innovation_scale * innovation;
                }
                else
                {
                    const Scalar rho_delta =
                        powf(fminf(fmaxf(params.k_texture_rho, 0.0f), 0.99999994f),
                             static_cast<Scalar>(delta));
                    const Scalar innovation_scale =
                        sqrtf(fmaxf(1.0f - rho_delta * rho_delta, 0.0f));
                    const Scalar innovation =
                        standard_real_gaussian_stream(params.seed,
                                                      static_cast<std::uint32_t>(range_idx),
                                                      static_cast<std::uint32_t>(az_idx),
                                                      current_pulse,
                                                      kTextureJumpSourceId);
                    z = rho_delta * texture_state[state_idx] + innovation_scale * innovation;
                }
            }

            texture_state[state_idx] = z;
            last_pulse[state_idx] = current_pulse;
            return lookup_k_sqrt_tau_stream(sqrt_tau_lut, params.k_lut_size, z);
        }

        __device__ __forceinline__ void apply_distribution_transform_stream(ClutterPrtParams params,
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

        __global__ void clutter_patch_contrib_kernel_stream(const Scalar *patch_amplitude_map,
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
                accumulate_fir_tap_stream(fir_coeffs[tap], white_pair.even, shaped_re, shaped_im);
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
                accumulate_fir_tap_stream(fir_coeffs[tap], white_pair.odd, shaped_re, shaped_im);
                accumulate_fir_tap_stream(fir_coeffs[tap + 1], white_pair.even, shaped_re, shaped_im);
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
                accumulate_fir_tap_stream(fir_coeffs[tap], white, shaped_re, shaped_im);
            }

            if (params.distribution == ClutterDistribution::K)
            {
                const Scalar sqrt_tau = update_k_texture_ar1_stream(k_texture_state,
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
                apply_distribution_transform_stream(params, shaped_re, shaped_im);
            }

            const Scalar scale = range_scale * active.az_amplitude_weight;
            patch_contrib[out_idx] = DeviceComplex{scale * shaped_re, scale * shaped_im};
        }

        __global__ void clutter_patch_reduce_kernel_stream(const DeviceComplex *patch_contrib,
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

        /// @brief 校验 stream 启动参数的有效性
        ///
        /// 检查项：
        /// - base_state 和 stream_state 是否已初始化
        /// - 共享内存资源指针非空（base_state 的 d_patch_amplitude_map、d_fir_coeffs）
        /// - stream_state 的两个 slot 的 d_el_amplitude_weight 均已分配
        /// - 尺寸一致性（num_range_cells、fir_length、active_count 等）
        /// - K 分布参数完整性（若启用）
        bool validate_stream_inputs(const ClutterGpuState &base_state,
                                    const ClutterGpuStreamState &stream_state,
                                    const ClutterPrtParams &params,
                                    const ClutterActivePatch *active_patches,
                                    const Scalar *el_amplitude_weight,
                                    std::string *error)
        {
            if (!base_state.initialized || !stream_state.initialized ||
                base_state.d_patch_amplitude_map == nullptr ||
                base_state.d_fir_coeffs == nullptr ||
                stream_state.d_el_amplitude_weight[0] == nullptr ||
                stream_state.d_el_amplitude_weight[1] == nullptr)
            {
                if (error != nullptr)
                {
                    *error = "launch_clutter_prt_gpu_2d_fast_stream: state is not initialized";
                }
                return false;
            }

            if (params.num_range_cells <= 0 ||
                params.num_range_cells != base_state.num_range_cells ||
                params.num_range_cells != stream_state.num_range_cells ||
                base_state.fir_length <= 0 ||
                params.active_count < 0 ||
                params.active_count > stream_state.active_patch_capacity ||
                (params.active_count > 0 && active_patches == nullptr) ||
                el_amplitude_weight == nullptr ||
                (params.distribution == ClutterDistribution::K &&
                 (base_state.d_k_texture_state == nullptr ||
                  base_state.d_k_texture_last_pulse == nullptr ||
                  base_state.d_k_sqrt_tau_lut == nullptr ||
                  params.k_lut_size != base_state.k_lut_size ||
                  params.k_texture_rho <= 0.0f ||
                  params.k_texture_innovation_scale < 0.0f)))
            {
                if (error != nullptr)
                {
                    *error = "launch_clutter_prt_gpu_2d_fast_stream: invalid input";
                }
                return false;
            }

            return true;
        }
    } // namespace

    /// @brief 初始化异步双缓冲 stream 状态
    ///
    /// 为两个 slot (ping-pong) 各分配独立的资源：
    /// - 设备内存：d_active_patches, d_echo, d_patch_contrib, d_el_amplitude_weight
    /// - Pinned 主机内存（cudaHostAlloc）：h_active_patches, h_el_amplitude_weight, h_echo
    ///   pinned 内存支持异步 DMA 传输 (cudaMemcpyAsync)，实现与内核执行的重叠
    /// - CUDA Stream (cudaStreamNonBlocking)：每个 slot 独立的命令队列
    /// - CUDA Event (cudaEventDisableTiming)：用于 stream 间同步
    ///   kernel_done_events: 标记内核完成
    ///   copy_done_events: 标记结果回传完成
    ///
    /// 共享资源（不复制）：
    /// - patch_amplitude_map（来自 base_state，只读）
    /// - fir_coeffs（来自 base_state，只读）
    /// - k_texture_state / k_texture_last_pulse / k_sqrt_tau_lut（来自 base_state，读写）
    bool initialize_clutter_gpu_stream_state(ClutterGpuStreamState &stream_state,
                                             const ClutterGpuState &base_state,
                                             std::string *error)
    {
        // 先释放之前可能已分配的资源
        release_clutter_gpu_stream_state(stream_state);

        if (!base_state.initialized || base_state.num_range_cells <= 0 ||
            base_state.active_patch_capacity <= 0)
        {
            if (error != nullptr)
            {
                *error = "initialize_clutter_gpu_stream_state: invalid base state";
            }
            return false;
        }

        // 计算各缓冲区字节大小
        const std::size_t active_bytes =
            static_cast<std::size_t>(base_state.active_patch_capacity) * sizeof(ClutterActivePatch);
        const std::size_t echo_bytes =
            static_cast<std::size_t>(base_state.num_range_cells) * sizeof(DeviceComplex);
        const std::size_t el_weight_bytes =
            static_cast<std::size_t>(base_state.num_range_cells) * sizeof(Scalar);
        const std::size_t contrib_bytes =
            static_cast<std::size_t>(base_state.num_range_cells) *
            static_cast<std::size_t>(base_state.active_patch_capacity) *
            sizeof(DeviceComplex);

        // 为两个 slot 各分配独立资源
        for (int slot = 0; slot < 2; ++slot)
        {
            auto cuda_error = cudaSuccess;

            // --- 设备内存分配 ---
            if (!allocate_device(stream_state.d_active_patches[slot],
                                 active_bytes,
                                 "cudaMalloc stream d_active_patches",
                                 error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }

            if (!allocate_device(stream_state.d_echo[slot],
                                 echo_bytes,
                                 "cudaMalloc stream d_echo",
                                 error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }

            if (!allocate_device(stream_state.d_patch_contrib[slot],
                                 contrib_bytes,
                                 "cudaMalloc stream d_patch_contrib",
                                 error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }

            if (!allocate_device(stream_state.d_el_amplitude_weight[slot],
                                 el_weight_bytes,
                                 "cudaMalloc stream d_el_amplitude_weight",
                                 error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }

            // --- Pinned 主机内存分配（支持异步 DMA 传输） ---
            if (!allocate_pinned_host(stream_state.h_active_patches[slot],
                                      active_bytes,
                                      "cudaHostAlloc stream h_active_patches",
                                      error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }

            if (!allocate_pinned_host(stream_state.h_el_amplitude_weight[slot],
                                      el_weight_bytes,
                                      "cudaHostAlloc stream h_el_amplitude_weight",
                                      error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }

            if (!allocate_pinned_host(stream_state.h_echo[slot],
                                      echo_bytes,
                                      "cudaHostAlloc stream h_echo",
                                      error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }

            // --- CUDA Stream 创建 ---
            // cudaStreamNonBlocking: stream 间互不阻塞，实现真正的异步并行
            cudaStream_t stream{};
            cuda_error = cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);
            if (!set_cuda_error(cuda_error, "cudaStreamCreateWithFlags", error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }
            stream_state.streams[slot] = stream;

            // --- CUDA Event 创建 ---
            // cudaEventDisableTiming: 禁用计时功能以减少 event 开销
            cudaEvent_t kernel_done{};
            cuda_error = cudaEventCreateWithFlags(&kernel_done, cudaEventDisableTiming);
            if (!set_cuda_error(cuda_error, "cudaEventCreate kernel_done", error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }
            stream_state.kernel_done_events[slot] = kernel_done;

            cudaEvent_t copy_done{};
            cuda_error = cudaEventCreateWithFlags(&copy_done, cudaEventDisableTiming);
            if (!set_cuda_error(cuda_error, "cudaEventCreate copy_done", error))
            {
                release_clutter_gpu_stream_state(stream_state);
                return false;
            }
            stream_state.copy_done_events[slot] = copy_done;
        }

        stream_state.num_range_cells = base_state.num_range_cells;
        stream_state.active_patch_capacity = base_state.active_patch_capacity;
        stream_state.initialized = true;
        return true;
    }

    void release_clutter_gpu_stream_state(ClutterGpuStreamState &stream_state)
    {
        for (int slot = 0; slot < 2; ++slot)
        {
            if (stream_state.streams[slot] != nullptr)
            {
                cudaStreamSynchronize(static_cast<cudaStream_t>(stream_state.streams[slot]));
            }
            if (stream_state.d_active_patches[slot] != nullptr)
            {
                cudaFree(stream_state.d_active_patches[slot]);
            }
            if (stream_state.d_echo[slot] != nullptr)
            {
                cudaFree(stream_state.d_echo[slot]);
            }
            if (stream_state.d_patch_contrib[slot] != nullptr)
            {
                cudaFree(stream_state.d_patch_contrib[slot]);
            }
            if (stream_state.d_el_amplitude_weight[slot] != nullptr)
            {
                cudaFree(stream_state.d_el_amplitude_weight[slot]);
            }
            if (stream_state.h_active_patches[slot] != nullptr)
            {
                cudaFreeHost(stream_state.h_active_patches[slot]);
            }
            if (stream_state.h_el_amplitude_weight[slot] != nullptr)
            {
                cudaFreeHost(stream_state.h_el_amplitude_weight[slot]);
            }
            if (stream_state.h_echo[slot] != nullptr)
            {
                cudaFreeHost(stream_state.h_echo[slot]);
            }
            if (stream_state.kernel_done_events[slot] != nullptr)
            {
                cudaEventDestroy(static_cast<cudaEvent_t>(stream_state.kernel_done_events[slot]));
            }
            if (stream_state.copy_done_events[slot] != nullptr)
            {
                cudaEventDestroy(static_cast<cudaEvent_t>(stream_state.copy_done_events[slot]));
            }
            if (stream_state.streams[slot] != nullptr)
            {
                cudaStreamDestroy(static_cast<cudaStream_t>(stream_state.streams[slot]));
            }
        }
        stream_state = ClutterGpuStreamState{};
    }

    /// @brief 在指定 stream 上异步启动连续 PRT 海杂波生成
    ///
    /// 将以下操作按顺序提交到 slot 对应的 CUDA stream 上异步执行：
    /// 1. cudaMemcpyAsync: 活跃补丁上传 (pinned host → device)
    /// 2. cudaMemcpyAsync: 俯仰幅度权重上传 (pinned host → device)
    /// 3. cudaStreamWaitEvent: 等待前一个 PRT 的内核完成（保证 K texture 状态一致性）
    /// 4. clutter_patch_contrib_kernel_stream: 贡献计算内核
    /// 5. clutter_patch_reduce_kernel_stream: 归约内核
    /// 6. cudaEventRecord(kernel_done): 记录内核完成事件
    /// 7. cudaMemcpyAsync: 结果回传 (device → pinned host)
    /// 8. cudaEventRecord(copy_done): 记录回传完成事件
    ///
    /// @param base_state 共享的 GPU 状态（patch 幅度图、FIR 系数、K texture 状态）
    /// @param stream_state 异步双缓冲状态
    /// @param slot 当前使用的 slot (0 或 1)
    /// @param previous_kernel_slot 前一个 PRT 的 slot 号（-1 表示无前序依赖）
    /// @param params PRT 参数
    /// @param active_patches 活跃方位补丁数组（主机端）
    /// @param el_amplitude_weight 距离向俯仰幅度权重数组（主机端）
    bool launch_clutter_prt_gpu_2d_fast_stream(ClutterGpuState &base_state,
                                               ClutterGpuStreamState &stream_state,
                                               int slot,
                                               int previous_kernel_slot,
                                               const ClutterPrtParams &params,
                                               const ClutterActivePatch *active_patches,
                                               const Scalar *el_amplitude_weight,
                                               std::string *error)
    {
        slot &= 1; // 确保 slot 在 [0, 1] 范围内
        if (!validate_stream_inputs(base_state,
                                    stream_state,
                                    params,
                                    active_patches,
                                    el_amplitude_weight,
                                    error))
        {
            return false;
        }

        auto cuda_error = cudaSuccess;
        cudaStream_t stream = static_cast<cudaStream_t>(stream_state.streams[slot]);

        // ---- 步骤 1: 异步上传活跃补丁 (pinned host → device) ----
        const std::size_t active_bytes =
            static_cast<std::size_t>(params.active_count) * sizeof(ClutterActivePatch);
        if (params.active_count > 0)
        {
            // 先拷贝到 pinned 内存，再异步 DMA 到设备
            std::copy(active_patches,
                      active_patches + params.active_count,
                      stream_state.h_active_patches[slot]);
            cuda_error = cudaMemcpyAsync(stream_state.d_active_patches[slot],
                                         stream_state.h_active_patches[slot],
                                         active_bytes,
                                         cudaMemcpyHostToDevice,
                                         stream);
            if (!set_cuda_error(cuda_error, "cudaMemcpyAsync stream active patches", error))
            {
                return false;
            }
        }

        // ---- 步骤 2: 异步上传俯仰幅度权重 (pinned host → device) ----
        const std::size_t el_weight_bytes =
            static_cast<std::size_t>(params.num_range_cells) * sizeof(Scalar);
        std::copy(el_amplitude_weight,
                  el_amplitude_weight + params.num_range_cells,
                  stream_state.h_el_amplitude_weight[slot]);
        cuda_error = cudaMemcpyAsync(stream_state.d_el_amplitude_weight[slot],
                                     stream_state.h_el_amplitude_weight[slot],
                                     el_weight_bytes,
                                     cudaMemcpyHostToDevice,
                                     stream);
        if (!set_cuda_error(cuda_error, "cudaMemcpyAsync stream el_amplitude_weight", error))
        {
            return false;
        }

        // ---- 步骤 3: Stream 间同步 —— 等待前一个 PRT 的内核完成 ----
        // K texture 状态是共享的，必须确保前一个 PRT 的 texture 更新完成后才能启动当前 PRT
        if (previous_kernel_slot >= 0)
        {
            cuda_error = cudaStreamWaitEvent(
                stream,
                static_cast<cudaEvent_t>(stream_state.kernel_done_events[previous_kernel_slot & 1]),
                0);
            if (!set_cuda_error(cuda_error, "cudaStreamWaitEvent previous kernel", error))
            {
                return false;
            }
        }

        // ---- 步骤 4-5: 启动两阶段内核（贡献 + 归约） ----
        constexpr int kThreadsPerBlock = 256;
        const dim3 contrib_blocks((params.num_range_cells + kThreadsPerBlock - 1) / kThreadsPerBlock,
                                  params.active_count,
                                  1);
        const int reduce_blocks = (params.num_range_cells + kThreadsPerBlock - 1) / kThreadsPerBlock;

        // 4a. 贡献计算内核
        clutter_patch_contrib_kernel_stream<<<contrib_blocks, kThreadsPerBlock, 0, stream>>>(
            base_state.d_patch_amplitude_map,
            stream_state.d_el_amplitude_weight[slot],
            stream_state.d_active_patches[slot],
            base_state.d_fir_coeffs,
            stream_state.d_patch_contrib[slot],
            base_state.d_k_texture_state,
            base_state.d_k_texture_last_pulse,
            base_state.d_k_sqrt_tau_lut,
            params,
            base_state.fir_length,
            base_state.num_range_cells);

        cuda_error = cudaGetLastError();
        if (!set_cuda_error(cuda_error, "clutter_patch_contrib_kernel_stream launch", error))
        {
            return false;
        }

        // 4b. 归约内核
        clutter_patch_reduce_kernel_stream<<<reduce_blocks, kThreadsPerBlock, 0, stream>>>(
            stream_state.d_patch_contrib[slot],
            stream_state.d_echo[slot],
            params,
            base_state.num_range_cells);

        cuda_error = cudaGetLastError();
        if (!set_cuda_error(cuda_error, "clutter_patch_reduce_kernel_stream launch", error))
        {
            return false;
        }

        // ---- 步骤 6: 记录内核完成事件 ----
        // 后续 PRT 通过 cudaStreamWaitEvent 等待此事件
        cuda_error = cudaEventRecord(static_cast<cudaEvent_t>(stream_state.kernel_done_events[slot]), stream);
        if (!set_cuda_error(cuda_error, "cudaEventRecord stream kernel_done", error))
        {
            return false;
        }

        // ---- 步骤 7: 异步回传结果 (device → pinned host) ----
        const std::size_t echo_bytes =
            static_cast<std::size_t>(params.num_range_cells) * sizeof(DeviceComplex);
        cuda_error = cudaMemcpyAsync(stream_state.h_echo[slot],
                                     stream_state.d_echo[slot],
                                     echo_bytes,
                                     cudaMemcpyDeviceToHost,
                                     stream);
        if (!set_cuda_error(cuda_error, "cudaMemcpyAsync stream echo", error))
        {
            return false;
        }

        // ---- 步骤 8: 记录回传完成事件 ----
        // 主机端通过 cudaEventSynchronize 等待此事件以取回结果
        cuda_error = cudaEventRecord(static_cast<cudaEvent_t>(stream_state.copy_done_events[slot]), stream);
        if (!set_cuda_error(cuda_error, "cudaEventRecord stream copy_done", error))
        {
            return false;
        }

        // 标记此 slot 有待取回的结果
        stream_state.slot_pending[slot] = true;
        return true;
    }

    /// @brief 同步等待指定 slot 的异步操作完成并取回结果
    ///
    /// 1. 通过 cudaEventSynchronize(copy_done) 阻塞等待结果回传完成
    /// 2. 从 pinned 主机内存读取结果（无需额外 cudaMemcpy）
    /// 3. 格式转换 DeviceComplex → Complex
    ///
    /// @note 若 slot 没有待取回的结果 (slot_pending == false)，直接返回 true
    bool wait_clutter_gpu_stream_slot(ClutterGpuStreamState &stream_state,
                                      int slot,
                                      Complex *out_echo,
                                      ClutterGpuTiming *timing,
                                      std::string *error)
    {
        if (timing != nullptr)
        {
            *timing = ClutterGpuTiming{};
        }

        slot &= 1;
        if (!stream_state.initialized || out_echo == nullptr)
        {
            if (error != nullptr)
            {
                *error = "wait_clutter_gpu_stream_slot: invalid state or output";
            }
            return false;
        }

        // 无待取回结果时直接返回
        if (!stream_state.slot_pending[slot])
        {
            return true;
        }

        // ---- 同步等待 copy_done event ----
        const auto wait_start = Clock::now();
        auto cuda_error =
            cudaEventSynchronize(static_cast<cudaEvent_t>(stream_state.copy_done_events[slot]));
        const auto wait_finish = Clock::now();
        if (!set_cuda_error(cuda_error, "cudaEventSynchronize stream copy_done", error))
        {
            return false;
        }

        // ---- 格式转换（pinned host 内存直接读取，无需额外拷贝） ----
        const auto convert_start = Clock::now();
        const auto *host_echo = stream_state.h_echo[slot];
        for (int i = 0; i < stream_state.num_range_cells; ++i)
        {
            const auto &sample = host_echo[static_cast<std::size_t>(i)];
            out_echo[static_cast<std::size_t>(i)] = Complex(sample.real, sample.imag);
        }
        const auto convert_finish = Clock::now();

        if (timing != nullptr)
        {
            timing->kernel_sync_ms = elapsed_ms(wait_start, wait_finish);
            timing->host_convert_ms = elapsed_ms(convert_start, convert_finish);
            timing->total_ms = elapsed_ms(wait_start, convert_finish);
        }

        // 清除 pending 标记，允许该 slot 被下一个 PRT 复用
        stream_state.slot_pending[slot] = false;
        return true;
    }

} // namespace radar::cuda
