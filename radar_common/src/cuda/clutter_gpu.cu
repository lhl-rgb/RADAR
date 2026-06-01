/// @file clutter_gpu.cu
/// @brief GPU 端海杂波仿真核心实现
///
/// 在 CUDA 设备上按 PRT（脉冲重复周期）实时生成海杂波回波。
/// 核心流程：白噪声生成 -> FIR 时间相关性滤波 -> 分布变换（Rayleigh/Weibull/LogNormal/K）
/// -> 空间方向图加权，最后通过 cudaMemcpy 将结果传回主机端。

#include "cuda/clutter_gpu.cuh"
#include <cmath>
#include <cuda_runtime.h>

#include <chrono>
#include <string>
#include <vector>
#include "math.h"
namespace radar::cuda
{
    namespace 
    {
        /// @brief 随机源 ID 常量定义
        ///
        /// Counter-based RNG 通过 source_id 区分同一 (range, az) 单元内不同用途的随机流，
        /// 保证各流独立且可确定性复现。
        constexpr std::uint32_t kClutterWhiteSourceId = 0U; ///< Speckle 白噪声流（FIR 卷积输入）
        constexpr std::uint32_t kTextureInitSourceId = 1U;  ///< K 分布 texture 首次初始化随机流
        constexpr std::uint32_t kTextureJumpSourceId = 2U;  ///< K 分布 texture 逐 PRT 跳步更新随机流
        constexpr Scalar kSqrt2 = 1.4142135623730951f;     ///< sqrt(2)，用于标准正态变换
        constexpr std::uint64_t kTexturePulseUnset = UINT64_MAX; ///< texture 尚未生成过的标记值
        /// @brief GPU 常量内存中的 FIR 滤波器系数
        ///
        /// 存储在 GPU 常量内存中，所有线程共享且缓存命中率高。
        /// 最大长度受 kClutterMaxFirLength (4096) 限制。
        __constant__ DeviceComplex c_clutter_fir[kClutterMaxFirLength];

        using Clock = std::chrono::steady_clock;
 
        /// @brief 计算两个时间点之间的毫秒间隔，用于性能计时
        double elapsed_ms(const Clock::time_point &start, const Clock::time_point &finish)
        {
            return std::chrono::duration<double, std::milli>(finish - start).count();
        }

        /// @brief 累加一个 FIR 抽头的复乘加运算
        ///
        /// 对 FIR 系数 h 和白噪声样本 white 进行复数乘法：
        /// accum += h * white，结果累加到 accum_re/accum_im 中。
        ///
        /// @param h FIR 滤波器系数（复数）
        /// @param white 白噪声复样本
        /// @param[in,out] accum_re 实部累加器
        /// @param[in,out] accum_im 虚部累加器
        __device__ __forceinline__ void accumulate_fir_tap(const DeviceComplex h,
                                                           const DeviceComplex white,
                                                           Scalar &accum_re,
                                                           Scalar &accum_im)
        {
            // 复数乘法: h * white = (h.real + j*h.imag) * (white.real + j*white.imag)
            accum_re += h.real * white.real - h.imag * white.imag;
            accum_im += h.real * white.imag + h.imag * white.real;
        }

        /// @brief 生成标准实高斯随机变量（均值 0，方差 1）
        ///
        /// 从 counter-based RNG 获取复高斯样本后，取其实部并乘以 sqrt(2) 得到标准实高斯变量。
        /// 复高斯样本 I/Q 独立，每维方差 0.5，乘以 sqrt(2) 后方差变为 1。
        __device__ __forceinline__ Scalar standard_real_gaussian(std::uint64_t seed,
                                                                 std::uint32_t range_idx,
                                                                 std::uint32_t az_idx,
                                                                 std::uint64_t pulse_index,
                                                                 std::uint32_t source_id)
        {
            const DeviceComplex sample =
                counter_complex_gaussian_sample(seed, range_idx, az_idx, pulse_index, source_id);
            return kSqrt2 * sample.real;
        }

        /// @brief K 分布 sqrt(tau) 查找表线性插值查询
        ///
        /// 将标准正态变量 z 映射为 sqrt(tau)：
        /// 1. z 钳位到 [-8, 8]
        /// 2. 线性映射到 LUT 索引 [0, lut_size-1]
        /// 3. 在相邻两个 LUT 条目间做线性插值
        ///
        /// 若 LUT 为空或长度 ≤ 1，直接返回 1.0（退化为 Rayleigh）。
        __device__ __forceinline__ Scalar lookup_k_sqrt_tau(const Scalar *lut,
                                                            int lut_size,
                                                            Scalar z)
        {
            if (lut == nullptr || lut_size <= 1)
            {
                return 1.0f;
            }

            // z 钳位到 [-8, 8] 范围，对应正态 CDF 的 (6.22e-16, 1-6.22e-16)
            const Scalar z_clamped = fminf(fmaxf(z, -8.0f), 8.0f);
            // 将 z 映射到 LUT 浮点索引 [0, lut_size-1]
            const Scalar x = (z_clamped + 8.0f) *
                             (static_cast<Scalar>(lut_size - 1) / 16.0f);
            int i = static_cast<int>(floorf(x));
            i = max(0, min(i, lut_size - 2)); // 确保线性插值有 i+1 可用
            const Scalar frac = x - static_cast<Scalar>(i); // 插值小数部分
            return lut[i] * (1.0f - frac) + lut[i + 1] * frac;
        }

        /// @brief K 分布 texture AR(1) 状态更新与 sqrt(tau) 查询
        ///
        /// K 分布复合模型：杂波 = speckle（快变复高斯） × sqrt(tau)（慢变 Gamma texture）。
        /// texture 服从均值为 1 的 Gamma 分布，通过 AR(1) 过程在 PRT 间缓慢演化。
        ///
        /// 更新逻辑：
        /// 1. 若首次访问或 pulse 回绕 → 从标准正态初始化
        /// 2. 若 delta = 0（同一 pulse 重复访问）→ 直接复用当前状态
        /// 3. 若 delta > 0 → AR(1) 递推：z = ρ^δ × z_prev + sqrt(1-ρ^(2δ)) × innovation
        /// 4. 通过 LUT 将 z (标准正态) 映射为 sqrt(tau) (Gamma 纹理)
        ///
        /// @return sqrt(tau)，用于乘以 speckle 复样本实现 K 分布调制
        __device__ __forceinline__ Scalar update_k_texture_ar1(Scalar *texture_state,
                                                               std::uint64_t *last_pulse,
                                                               const Scalar *sqrt_tau_lut,
                                                               ClutterPrtParams params,
                                                               int range_idx,
                                                               int az_idx)
        {
            // 快速路径：K 分布未启用或参数无效时返回 1.0（退化为 Rayleigh）
            if (texture_state == nullptr || last_pulse == nullptr || sqrt_tau_lut == nullptr ||
                params.k_lut_size <= 1 || params.k_texture_rho <= 0.0f)
            {
                return 1.0f;
            }

            // 计算当前 (az, range) 单元在 texture 状态数组中的扁平索引
            const std::size_t state_idx =
                row_major_index(az_idx, range_idx, params.num_range_cells);
            const std::uint64_t current_pulse = params.pulse_index;
            const std::uint64_t previous_pulse = last_pulse[state_idx];

            Scalar z = 0.0f;
            if (previous_pulse == kTexturePulseUnset || previous_pulse > current_pulse)
            {
                // 首次访问 或 pulse_index 回绕：从标准正态初始化 texture 驱动变量 z
                z = standard_real_gaussian(params.seed,
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
                    // 同一 pulse 内重复访问同一 (az, range)，直接复用当前 z
                    z = texture_state[state_idx];
                }
                else
                {
                    // AR(1) 递推：z = ρ^δ × z_prev + sqrt(1-ρ^(2δ)) × N(0,1)
                    const Scalar rho_delta =
                        powf(fminf(fmaxf(params.k_texture_rho, 0.0f), 0.99999994f),
                             static_cast<Scalar>(delta));
                    const Scalar innovation_scale =
                        sqrtf(fmaxf(1.0f - rho_delta * rho_delta, 0.0f));
                    const Scalar innovation =
                        standard_real_gaussian(params.seed,
                                               static_cast<std::uint32_t>(range_idx),
                                               static_cast<std::uint32_t>(az_idx),
                                               current_pulse,
                                               kTextureJumpSourceId);
                    z = rho_delta * texture_state[state_idx] + innovation_scale * innovation;
                }
            }

            // 持久化当前状态，供后续 PRT 递推使用
            texture_state[state_idx] = z;
            last_pulse[state_idx] = current_pulse;
            // 将标准正态 z 通过 LUT 映射为 sqrt(tau)
            return lookup_k_sqrt_tau(sqrt_tau_lut, params.k_lut_size, z);
        }

        /// @brief 对 FIR 滤波后的复样本施加幅度分布变换
        ///
        /// 支持非 K 分布海杂波的幅度变换：
        /// - Rayleigh: 无需变换，FIR 滤波后自然服从 Rayleigh 分布
        /// - Weibull: 通过功率映射将 Rayleigh 变换为 Weibull 分布
        /// - LogNormal: 使用复高斯的实部作为驱动变量生成 LogNormal 幅度
        ///
        /// @param params PRT 参数（包含分布类型及分布参数）
        /// @param[in,out] sample_re 输入复样本实部，输出变换后实部
        /// @param[in,out] sample_im 输入复样本虚部，输出变换后虚部
        __device__ __forceinline__ void apply_distribution_transform(ClutterPrtParams params,
                                                                     Scalar &sample_re,
                                                                     Scalar &sample_im)
        {
            // Rayleigh 分布：FIR 滤波后自然服从，无需额外变换
            if (params.distribution == ClutterDistribution::Rayleigh)
            {
                return;
            }

            // 计算功率
            const Scalar power = sample_re * sample_re + sample_im * sample_im;
            // 功率过小直接置零，避免数值问题
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
                // Weibull 变换：target = scale / 2^(1/shape) * power^(1/shape)
                const Scalar safe_shape = fmaxf(params.weibull_shape, EPSILON);
                const Scalar safe_scale = fmaxf(params.weibull_scale, EPSILON);
                target_amplitude =
                    (safe_scale / powf(2.0f, 1.0f / safe_shape)) *
                    powf(power, 1.0f / safe_shape);
            }
            else if (params.distribution == ClutterDistribution::LogNormal)
            {
                // LogNormal 变换：target = exp(mu + sigma * sqrt(2) * gaussian_driver)
                const Scalar gaussian_driver = kSqrt2 * sample_re; // 利用复高斯实部
                target_amplitude = expf(params.lognormal_mu + params.lognormal_sigma * gaussian_driver);
            }

            // 按比例缩放复样本以匹配目标幅度
            const Scalar factor = target_amplitude / amplitude;
            sample_re *= factor;
            sample_im *= factor;
        }

        /// @brief GPU 内核：按 PRT 实时生成海杂波回波
        ///
        /// 每个线程负责一个距离单元，执行以下流程：
        /// 1. 将活跃方位补丁载入共享内存
        /// 2. 计算俯仰方向图增益
        /// 3. 对每个方位补丁，通过 FIR 卷积施加时间相关性
        /// 4. 应用幅度分布变换（Rayleigh/Weibull/LogNormal）
        /// 5. 加权累加所有方位补丁的贡献
        ///
        /// @param patch_amplitude_map 静态 patch 幅度图（设备端，布局为 [az][range]）
        /// @param el_amplitude_weight 距离向俯仰幅度权重数组（设备端）
        /// @param active_patches 活跃方位补丁列表（设备端）
        /// @param[out] echo 输出回波数组（设备端，DeviceComplex 格式）
        /// @param params PRT 参数（值传递，包含所有标量参数）
        /// @param fir_length FIR 滤波器长度
        /// @brief GPU 内核：按 PRT 实时生成海杂波回波（一维距离并行版本）
        ///
        /// 每个线程负责一个距离单元，执行以下流程：
        /// 1. 将活跃方位补丁从全局内存载入共享内存（协作加载）
        /// 2. 计算距离向俯仰 + 峰值增益加权系数
        /// 3. 对每个活跃方位补丁：
        ///    a. 查 patch 幅度图获取 sqrt(P) 功率系数
        ///    b. Counter-based RNG + FIR 卷积施加时间相关性（高斯谱整形）
        ///    c. 施加幅度分布变换（Rayleigh / Weibull / LogNormal / K）
        ///    d. 方位方向图加权累加到累加器
        /// 4. 写入该距离单元的最终复回波
        ///
        /// FIR 卷积采用偶/奇脉冲对优化：一次 RNG 调用生成一对相邻脉冲的白噪声样本，
        /// 减少 Philox 调用的开销。
        __global__ void clutter_prt_kernel(const Scalar *patch_amplitude_map,
                                           const Scalar *el_amplitude_weight,
                                           const ClutterActivePatch *active_patches,
                                           DeviceComplex *echo,
                                           Scalar *k_texture_state,
                                           std::uint64_t *k_texture_last_pulse,
                                           const Scalar *k_sqrt_tau_lut,
                                           ClutterPrtParams params,
                                           int fir_length)
        {
            // 每个线程处理一个距离单元（一维 grid，沿距离方向展开）
            const int range_idx = blockIdx.x * blockDim.x + threadIdx.x;

            // 将活跃补丁从全局内存载入共享内存（所有线程协作加载，减少重复读取）
            extern __shared__ ClutterActivePatch shared_active[];
            for (int i = threadIdx.x; i < params.active_count; i += blockDim.x)
            {
                shared_active[i] = active_patches[i];
            }
            __syncthreads(); // 确保所有补丁已载入，之后所有线程可安全读取

            // 边界检查：超出距离单元范围则直接返回
            if (range_idx >= params.num_range_cells)
            {
                return;
            }

            // 距离向总缩放系数 = 峰值天线增益 × 俯仰方向图权重
            const Scalar range_scale =
                params.peak_amplitude_weight * el_amplitude_weight[range_idx];
            if (range_scale <= 0.0f)
            {
                // 俯仰方向图在此距离单元无贡献，输出置零
                echo[range_idx] = DeviceComplex{0.0f, 0.0f};
                return;
            }

            Scalar accum_re = 0.0f; // 复回波实部累加器
            Scalar accum_im = 0.0f; // 复回波虚部累加器

            // 遍历所有活跃方位补丁，累加各补丁的回波贡献
            for (int active_idx = 0; active_idx < params.active_count; ++active_idx)
            {
                const ClutterActivePatch active = shared_active[active_idx];
                if (active.az_amplitude_weight <= 0.0f)
                {
                    continue; // 该方位补丁增益过低，无贡献，跳过
                }
                // 查二维 patch 幅度图：[az_idx][range_idx]
                const std::size_t amplitude_idx =
                    row_major_index(active.az_idx, range_idx, params.num_range_cells);
                const Scalar patch_amplitude = patch_amplitude_map[amplitude_idx];
                if (patch_amplitude <= 0.0f)
                {
                    continue; // 此 patch 无回波功率（如超出地距范围），跳过
                }

                Scalar shaped_re = 0.0f; // FIR 整形后的复样本实部
                Scalar shaped_im = 0.0f; // FIR 整形后的复样本虚部
                int tap = 0;

                // ---- FIR 时间相关性卷积 ----
                // 数学关系固定为：
                //   y[n] = Σ h[k] * w[n-k], k = 0..L-1
                // 因此 FIR 长度为 L 时，每个 PRT 都需要 L 个历史白噪声样本。
                //
                // RNG pair 只是取样优化，不改变上述 FIR 关系：
                //   pair_index = pulse / 2
                //   pair.even  = w[2 * pair_index]
                //   pair.odd   = w[2 * pair_index + 1]
                //   01  23  45   67
                // 如果当前 n 为偶数，tap0 对应 w[n] = pair.even，不能和 tap1 的 w[n-1]
                // 组成同一个 pair，所以先单独处理 tap0；随后 tap1/tap2、tap3/tap4...
                // 分别对应同一个 pair 的 odd/even。
                // 如果当前 n 为奇数，则从 tap0/tap1 开始天然按 odd/even 成对。
                if ((params.pulse_index & 1ULL) == 0ULL)
                {
                    const DeviceComplexPair white_pair = counter_complex_gaussian_sample_pair(
                        params.seed,
                        static_cast<std::uint32_t>(range_idx),
                        static_cast<std::uint32_t>(active.az_idx),
                        params.pulse_index >> 1U,
                        kClutterWhiteSourceId);
                    accumulate_fir_tap(c_clutter_fir[tap], white_pair.even, shaped_re, shaped_im);
                    ++tap;
                }

                // 主循环每次处理两个相邻历史样本：
                //   tap   -> w[tap_pulse0]     -> pair.odd
                //   tap+1 -> w[tap_pulse0 - 1] -> pair.even
                // 这里能成立是因为进入本循环时 tap_pulse0 总是奇数。
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

                    accumulate_fir_tap(c_clutter_fir[tap], white_pair.odd, shaped_re, shaped_im);
                    accumulate_fir_tap(c_clutter_fir[tap + 1], white_pair.even, shaped_re, shaped_im);
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
                    accumulate_fir_tap(c_clutter_fir[tap], white, shaped_re, shaped_im);
                }

                // ---- 幅度分布变换 ----
                if (params.distribution == ClutterDistribution::K)
                {
                    // K 分布：speckle × sqrt(tau)，tau 通过 AR(1) 过程逐 PRT 演化
                    const Scalar sqrt_tau = update_k_texture_ar1(k_texture_state,
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
                    // 施加幅度分布变换（Weibull / LogNormal）
                    // Rayleigh 分布无需变换，FIR 滤波后自然服从
                    apply_distribution_transform(params, shaped_re, shaped_im);
                }

                // ---- 方位方向图加权累加 ----
                // z(r) = Σ_{θ} G_az(θ-θ_beam) × sqrt(P_patch(r,θ)) × c(r,θ)
                const Scalar scale =
                    range_scale * active.az_amplitude_weight * patch_amplitude;
                accum_re += scale * shaped_re;
                accum_im += scale * shaped_im;
            }

            // 写入该距离单元的最终复回波结果
            echo[range_idx] = DeviceComplex{accum_re, accum_im};
        }

        /// @brief 获取 CUDA 错误的可读描述字符串
        const char *cuda_error_string(cudaError_t error)
        {
            return cudaGetErrorString(error);
        }

        /// @brief 统一处理 CUDA 错误：成功返回 true，失败则写入错误信息到 error 并返回 false
        ///
        /// @param cuda_error CUDA 运行时 API 返回的错误码
        /// @param context 错误上下文描述（如函数名）
        /// @param[out] error 错误信息输出（可选）
        /// @return true 成功，false 失败
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
    } // namespace (anonymous)

    /// @brief 检查 CUDA 海杂波生成器是否可用
    ///
    /// 通过查询 CUDA 设备数量来判断是否有可用的 GPU。
    ///
    /// @param error 可选，失败时写入错误信息
    /// @return true 如果至少有一个 CUDA 设备可用
    bool is_clutter_gpu_available(std::string *error)
    {
        int device_count = 0;
        const auto cuda_error = cudaGetDeviceCount(&device_count);
        if (!set_cuda_error(cuda_error, "cudaGetDeviceCount", error))
        {
            return false;
        }
        if (device_count <= 0)
        {
            if (error != nullptr)
            {
                *error = "cudaGetDeviceCount: no CUDA-capable device found";
            }
            return false;
        }
        return true;
    }

    /// @brief 编译时验证内核：确保 counter-based RNG 相关设备函数能被正确编译和链接
    ///
    /// 此内核不会被实际调用，仅用于在 CUDA 编译阶段触发设备代码的实例化，
    /// 避免链接时出现未定义符号。
    __global__ void clutter_rng_compile_kernel(DeviceComplex *out)
    {
        if (threadIdx.x == 0 && blockIdx.x == 0)
        {
            const DeviceComplex sample = counter_complex_gaussian_sample(2026ULL, 0U, 0U, 0ULL, 0U);
            out[0] = sample;
        }
    }

    /// @brief 初始化海杂波 GPU 状态，分配设备内存并上传静态数据
    ///
    /// 在 GPU 上分配以下资源：
    /// - d_patch_amplitude_map: 二维 patch 幅度图（只读，从主机拷贝）
    /// - d_el_amplitude_weight: 距离向俯仰幅度权重（只读，从主机拷贝）
    /// - d_active_patches: 活跃方位补丁缓冲区（每 PRT 更新）
    /// - d_echo: 输出回波缓冲区
    /// - c_clutter_fir (常量内存): FIR 滤波器系数
    ///
    /// @param[out] state 要初始化的 GPU 状态结构体
    /// @param fir_coeffs FIR 滤波器系数数组（主机端）
    /// @param fir_length FIR 滤波器长度（≤ kClutterMaxFirLength）
    /// @param patch_amplitude_map 二维 patch 幅度图（主机端，布局为 [az][range]）
    /// @param el_amplitude_weight 距离向俯仰幅度权重数组（主机端）
    /// @param num_range_cells 距离单元数量
    /// @param active_patch_capacity 活跃方位补丁的最大容量
    /// @param error 可选，失败时写入错误信息
    /// @return true 初始化成功，false 失败
    bool initialize_clutter_gpu_state(ClutterGpuState &state,
                                      const Complex *fir_coeffs,
                                      int fir_length,
                                      const Scalar *patch_amplitude_map,
                                      const Scalar *el_amplitude_weight,
                                      int num_range_cells,
                                      int active_patch_capacity,
                                      int num_az_cells,
                                      const Scalar *k_sqrt_tau_lut,
                                      int k_lut_size,
                                      std::string *error)
    {
        // 先释放之前可能已分配的资源，防止内存泄漏
        release_clutter_gpu_state(state);

        // =====================================================================
        // 阶段 1: 参数合法性检查
        // =====================================================================
        if (fir_coeffs == nullptr || patch_amplitude_map == nullptr || el_amplitude_weight == nullptr ||
            fir_length <= 0 || fir_length > kClutterMaxFirLength ||
            num_range_cells <= 0 || num_range_cells > kClutterMaxSpatialCells ||
            active_patch_capacity <= 0 ||
            num_az_cells <= 0 || num_az_cells > kClutterMaxSpatialCells ||
            (k_lut_size > 0 && k_sqrt_tau_lut == nullptr))
        {
            if (error != nullptr)
            {
                *error = "initialize_clutter_gpu_state: invalid input buffers or sizes";
            }
            return false;
        }

        // =====================================================================
        // 阶段 2: FIR 系数格式转换 (Complex → DeviceComplex POD)
        // =====================================================================
        // Complex (std::complex<Scalar>) 不能直接在设备端使用，需转换为 POD 格式
        std::vector<DeviceComplex> host_fir(static_cast<std::size_t>(fir_length));
        for (int i = 0; i < fir_length; ++i)
        {
            host_fir[static_cast<std::size_t>(i)] =
                DeviceComplex{fir_coeffs[i].real(), fir_coeffs[i].imag()};
        }

        // =====================================================================
        // 阶段 3: 计算各缓冲区所需的字节大小
        // =====================================================================
        const std::size_t fir_bytes = static_cast<std::size_t>(fir_length) * sizeof(DeviceComplex);
        const std::size_t amplitude_map_bytes =
            static_cast<std::size_t>(num_range_cells) * static_cast<std::size_t>(num_az_cells) *
            sizeof(Scalar);
        const std::size_t elevation_weight_bytes =
            static_cast<std::size_t>(num_range_cells) * sizeof(Scalar);
        const std::size_t active_bytes =
            static_cast<std::size_t>(active_patch_capacity) * sizeof(ClutterActivePatch);
        const std::size_t echo_bytes = static_cast<std::size_t>(num_range_cells) * sizeof(DeviceComplex);
        // K 分布 texture 状态：每个 (az, range) 单元各存一个 Scalar + 一个 uint64_t
        const bool enable_k_texture = k_sqrt_tau_lut != nullptr && k_lut_size > 1;
        const std::size_t texture_state_count =
            static_cast<std::size_t>(num_range_cells) * static_cast<std::size_t>(num_az_cells);
        const std::size_t texture_state_bytes = texture_state_count * sizeof(Scalar);
        const std::size_t texture_pulse_bytes = texture_state_count * sizeof(std::uint64_t);
        const std::size_t k_lut_bytes = static_cast<std::size_t>(k_lut_size) * sizeof(Scalar);

        // =====================================================================
        // 阶段 4: 分配设备内存并上传数据
        // =====================================================================
        auto cuda_error = cudaSuccess;

        // 4a. 分配并上传二维 patch 幅度图（只读，初始化后不变）
        if (!allocate_device(state.d_patch_amplitude_map,
                             amplitude_map_bytes,
                             "cudaMalloc d_patch_amplitude_map",
                             error))
        {
            release_clutter_gpu_state(state);
            return false;
        }

        // 4b. 分配输出回波缓冲区（每 PRT 写入）
        if (!allocate_device(state.d_echo, echo_bytes, "cudaMalloc d_echo", error))
        {
            release_clutter_gpu_state(state);
            return false;
        }

        // 4c. 分配并上传 FIR 系数到全局内存（供 2D 实验路径使用）
        if (!allocate_device(state.d_fir_coeffs, fir_bytes, "cudaMalloc d_fir_coeffs", error))
        {
            release_clutter_gpu_state(state);
            return false;
        }

        cuda_error = cudaMemcpy(state.d_fir_coeffs, host_fir.data(), fir_bytes, cudaMemcpyHostToDevice);
        if (!set_cuda_error(cuda_error, "cudaMemcpy d_fir_coeffs", error))
        {
            release_clutter_gpu_state(state);
            return false;
        }

        // 4d. 将 FIR 系数拷贝到 GPU 常量内存（所有线程共享，缓存命中率最高）
        //     常量内存通过 c_clutter_fir 在 kernel 中直接访问
        cuda_error = cudaMemcpyToSymbol(c_clutter_fir, host_fir.data(), fir_bytes);
        if (!set_cuda_error(cuda_error, "cudaMemcpyToSymbol c_clutter_fir", error))
        {
            release_clutter_gpu_state(state);
            return false;
        }

        // 4e. 将二维 patch 幅度图上传到设备端
        cuda_error = cudaMemcpy(state.d_patch_amplitude_map,
                                patch_amplitude_map,
                                amplitude_map_bytes,
                                cudaMemcpyHostToDevice);
        if (!set_cuda_error(cuda_error, "cudaMemcpy d_patch_amplitude_map", error))
        {
            release_clutter_gpu_state(state);
            return false;
        }

        // 4f. 分配活跃方位补丁缓冲区（内容每 PRT 更新）
        if (!allocate_device(state.d_active_patches,
                             active_bytes,
                             "cudaMalloc d_active_patches",
                             error))
        {
            release_clutter_gpu_state(state);
            return false;
        }

        // 4g. 分配并上传距离向俯仰幅度权重
        if (!allocate_device(state.d_el_amplitude_weight,
                             elevation_weight_bytes,
                             "cudaMalloc d_el_amplitude_weight",
                             error))
        {
            release_clutter_gpu_state(state);
            return false;
        }

        cuda_error = cudaMemcpy(state.d_el_amplitude_weight,
                                el_amplitude_weight,
                                elevation_weight_bytes,
                                cudaMemcpyHostToDevice);
        if (!set_cuda_error(cuda_error, "cudaMemcpy d_el_amplitude_weight", error))
        {
            release_clutter_gpu_state(state);
            return false;
        }

        // =====================================================================
        // 阶段 5: K 分布 texture 状态分配（仅当启用 K 分布时）
        // =====================================================================
        if (enable_k_texture)
        {
            // 5a. 分配 texture 状态数组（标准正态驱动变量 z）
            if (!allocate_device(state.d_k_texture_state,
                                 texture_state_bytes,
                                 "cudaMalloc d_k_texture_state",
                                 error))
            {
                release_clutter_gpu_state(state);
                return false;
            }

            // 初始化 texture 状态为零
            cuda_error = cudaMemset(state.d_k_texture_state, 0, texture_state_bytes);
            if (!set_cuda_error(cuda_error, "cudaMemset d_k_texture_state", error))
            {
                release_clutter_gpu_state(state);
                return false;
            }

            // 5b. 分配最近更新脉冲索引数组
            if (!allocate_device(state.d_k_texture_last_pulse,
                                 texture_pulse_bytes,
                                 "cudaMalloc d_k_texture_last_pulse",
                                 error))
            {
                release_clutter_gpu_state(state);
                return false;
            }

            // 初始化所有单元为 kTexturePulseUnset (0xFFFFFFFFFFFFFFFF)
            cuda_error = cudaMemset(state.d_k_texture_last_pulse, 0xFF, texture_pulse_bytes);
            if (!set_cuda_error(cuda_error, "cudaMemset d_k_texture_last_pulse", error))
            {
                release_clutter_gpu_state(state);
                return false;
            }

            // 5c. 分配并上传 K 分布 sqrt(tau) 查找表
            if (!allocate_device(state.d_k_sqrt_tau_lut,
                                 k_lut_bytes,
                                 "cudaMalloc d_k_sqrt_tau_lut",
                                 error))
            {
                release_clutter_gpu_state(state);
                return false;
            }

            cuda_error = cudaMemcpy(state.d_k_sqrt_tau_lut,
                                    k_sqrt_tau_lut,
                                    k_lut_bytes,
                                    cudaMemcpyHostToDevice);
            if (!set_cuda_error(cuda_error, "cudaMemcpy d_k_sqrt_tau_lut", error))
            {
                release_clutter_gpu_state(state);
                return false;
            }
        }

        // =====================================================================
        // 阶段 6: 记录状态元数据
        // =====================================================================
        state.num_range_cells = num_range_cells;
        state.num_az_cells = num_az_cells;
        state.active_patch_capacity = active_patch_capacity;
        state.fir_length = fir_length;
        state.k_lut_size = enable_k_texture ? k_lut_size : 0;
        state.initialized = true;
        return true;
    }

    bool update_clutter_gpu_elevation_amplitude_weight(ClutterGpuState &state,
                                                       const Scalar *el_amplitude_weight,
                                                       int num_range_cells,
                                                       std::string *error)
    {
        if (!state.initialized || state.d_el_amplitude_weight == nullptr ||
            el_amplitude_weight == nullptr || num_range_cells <= 0 ||
            num_range_cells != state.num_range_cells)
        {
            if (error != nullptr)
            {
                *error = "update_clutter_gpu_elevation_amplitude_weight: invalid state or input";
            }
            return false;
        }

        const std::size_t bytes = static_cast<std::size_t>(num_range_cells) * sizeof(Scalar);
        const auto cuda_error = cudaMemcpy(state.d_el_amplitude_weight,
                                           el_amplitude_weight,
                                           bytes,
                                           cudaMemcpyHostToDevice);
        return set_cuda_error(cuda_error, "cudaMemcpy d_el_amplitude_weight", error);
    }

    /// @brief 释放海杂波 GPU 状态中的所有设备内存
    ///
    /// 安全释放所有已分配的设备内存，并将状态重置为默认值。
    /// 可安全重复调用（已释放的指针为 nullptr 时 cudaFree 会被跳过）。
    ///
    /// @param[in,out] state 要释放的 GPU 状态
    void release_clutter_gpu_state(ClutterGpuState &state)
    {
        if (state.d_patch_amplitude_map != nullptr)
        {
            cudaFree(state.d_patch_amplitude_map);
        }
        if (state.d_el_amplitude_weight != nullptr)
        {
            cudaFree(state.d_el_amplitude_weight);
        }
        if (state.d_active_patches != nullptr)
        {
            cudaFree(state.d_active_patches);
        }
        if (state.d_fir_coeffs != nullptr)
        {
            cudaFree(state.d_fir_coeffs);
        }
        if (state.d_patch_contrib != nullptr)
        {
            cudaFree(state.d_patch_contrib);
        }
        if (state.d_echo != nullptr)
        {
            cudaFree(state.d_echo);
        }
        if (state.d_k_texture_state != nullptr)
        {
            cudaFree(state.d_k_texture_state);
        }
        if (state.d_k_texture_last_pulse != nullptr)
        {
            cudaFree(state.d_k_texture_last_pulse);
        }
        if (state.d_k_sqrt_tau_lut != nullptr)
        {
            cudaFree(state.d_k_sqrt_tau_lut);
        }
        // 重置为默认状态
        state = ClutterGpuState{};
    }

    /// @brief 在 GPU 上按单个 PRT 生成海杂波回波
    ///
    /// 完整流程：
    /// 1. 参数校验
    /// 2. 上传活跃方位补丁列表到设备端
    /// 3. 启动 clutter_prt_kernel 执行 GPU 计算
    /// 4. 同步等待内核完成
    /// 5. 将结果从设备端拷贝回主机端
    /// 6. 将 DeviceComplex 格式转换为 Complex 格式
    ///
    /// @param state GPU 状态（需已初始化）
    /// @param params PRT 参数（脉冲索引、分布参数、方向图参数等）
    /// @param active_patches 活跃方位补丁数组（主机端）
    /// @param[out] out_echo 输出回波缓冲区（主机端，Complex 格式）
    /// @param[out] timing 可选，记录各阶段耗时用于性能分析
    /// @param[out] error 可选，失败时写入错误信息
    /// @return true 生成成功，false 失败
    bool generate_clutter_prt_gpu(ClutterGpuState &state,
                                  const ClutterPrtParams &params,
                                  const ClutterActivePatch *active_patches,
                                  Complex *out_echo,
                                  ClutterGpuTiming *timing,
                                  std::string *error)
    {
        // 初始化计时数据
        if (timing != nullptr)
        {
            *timing = ClutterGpuTiming{};
        }

        const auto total_start = Clock::now();

        // 状态校验：确保所有设备内存已分配
        if (!state.initialized ||
            state.d_patch_amplitude_map == nullptr || state.d_el_amplitude_weight == nullptr ||
            state.d_active_patches == nullptr ||
            state.d_echo == nullptr)
        {
            if (error != nullptr)
            {
                *error = "generate_clutter_prt_gpu: state is not initialized";
            }
            return false;
        }

        // 参数校验：输出缓冲区、尺寸匹配、补丁数量等
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
                *error = "generate_clutter_prt_gpu: invalid output or size mismatch";
            }
            return false;
        }

        // 阶段 1: 上传活跃方位补丁到设备端
        if (params.active_count > 0)
        {
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
        }

        // 阶段 2: 启动 GPU 内核
        constexpr int kThreadsPerBlock = 256; // 每个线程块 256 个线程
        const int blocks = (params.num_range_cells + kThreadsPerBlock - 1) / kThreadsPerBlock;
        const std::size_t shared_bytes =
            static_cast<std::size_t>(params.active_count) * sizeof(ClutterActivePatch); // 共享内存大小

        const auto kernel_start = Clock::now();
        clutter_prt_kernel<<<blocks, kThreadsPerBlock, shared_bytes>>>(
            state.d_patch_amplitude_map,
            state.d_el_amplitude_weight,
            state.d_active_patches,
            state.d_echo,
            state.d_k_texture_state,
            state.d_k_texture_last_pulse,
            state.d_k_sqrt_tau_lut,
            params,
            state.fir_length);

        // 检查内核启动错误
        auto cuda_error = cudaGetLastError();
        if (!set_cuda_error(cuda_error, "clutter_prt_kernel launch", error))
        {
            return false;
        }

        // 同步等待内核执行完成
        cuda_error = cudaDeviceSynchronize();
        const auto kernel_finish = Clock::now();
        if (timing != nullptr)
        {
            timing->kernel_sync_ms = elapsed_ms(kernel_start, kernel_finish);
        }
        if (!set_cuda_error(cuda_error, "clutter_prt_kernel synchronize", error))
        {
            return false;
        }

        // 阶段 3: 将结果从设备端拷贝回主机端
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
        if (!set_cuda_error(cuda_error, "cudaMemcpy clutter echo", error))
        {
            return false;
        }

        // 阶段 4: 格式转换 DeviceComplex -> Complex
        const auto convert_start = Clock::now();
        for (int i = 0; i < params.num_range_cells; ++i)
        {
            const auto &sample = host_echo[static_cast<std::size_t>(i)];
            out_echo[static_cast<std::size_t>(i)] = Complex(sample.real, sample.imag);
        }
        const auto convert_finish = Clock::now();

        // 记录总耗时
        if (timing != nullptr)
        {
            timing->host_convert_ms = elapsed_ms(convert_start, convert_finish);
            timing->total_ms = elapsed_ms(total_start, convert_finish);
        }

        return true;
    }

} // namespace cuda
