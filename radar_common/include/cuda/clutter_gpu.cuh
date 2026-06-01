#pragma once

/// @file clutter_gpu.cuh
/// @brief GPU 端海杂波仿真接口声明
///
/// 定义在 GPU 上按 PRT（脉冲重复周期）实时生成海杂波回波的数据结构
/// 和函数接口。核心流程：白噪声 -> FIR 时间相关性滤波 -> 分布变换 -> 空间方向图加权。

#include "cuda/clutter_rng.cuh"
#include "core/types.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace radar::cuda
{
    /// @brief 检查 CUDA 海杂波生成器是否可用
    ///
    /// @param error 可选，失败时写入错误信息
    /// @return true 如果至少有一个 CUDA 设备可用
    bool is_clutter_gpu_available(std::string *error = nullptr);

    /// @brief FIR 滤波器最大长度，受 GPU 常量内存大小限制
    constexpr int kClutterMaxFirLength = 4096;

    /// @brief Counter RNG 中距离和方位索引各占 16 bit。
    constexpr int kClutterMaxSpatialCells = 65536;

    /// @brief 活跃方位补丁描述符
    ///
    /// 每个补丁代表一个特定的方位角方向，包含方位索引和对应的幅度权重。
    /// 在每 PRT 生成时，根据当前波束指向筛选出活跃的方位补丁。
    struct ClutterActivePatch
    {
        int az_idx;                    ///< 方位角索引
        Scalar az_amplitude_weight;    ///< 直接乘到复回波上的归一化方位幅度权重 [0, 1]
    };

    /// @brief GPU 海杂波生成的性能计时数据
    ///
    /// 记录各阶段的耗时，用于性能分析和优化。
    struct ClutterGpuTiming
    {
        double active_upload_ms = 0.0;  ///< 活跃补丁上传耗时 (ms)
        double kernel_sync_ms = 0.0;    ///< 内核执行 + 同步耗时 (ms)
        double device_to_host_ms = 0.0; ///< 结果从设备到主机拷贝耗时 (ms)
        double host_convert_ms = 0.0;   ///< 主机端格式转换耗时 (ms)
        double total_ms = 0.0;          ///< 总耗时 (ms)
    };

    /// @brief 异步双缓冲路径使用的 slot 私有资源。
    struct ClutterGpuStreamState
    {
        ClutterActivePatch *d_active_patches[2] = {nullptr, nullptr};
        DeviceComplex *d_echo[2] = {nullptr, nullptr};
        DeviceComplex *d_patch_contrib[2] = {nullptr, nullptr};
        Scalar *d_el_amplitude_weight[2] = {nullptr, nullptr};
        ClutterActivePatch *h_active_patches[2] = {nullptr, nullptr};
        Scalar *h_el_amplitude_weight[2] = {nullptr, nullptr};
        DeviceComplex *h_echo[2] = {nullptr, nullptr};
        // CUDA runtime 句柄保留为 void*，避免公共头文件依赖 cuda_runtime.h。
        void *streams[2] = {nullptr, nullptr};
        void *kernel_done_events[2] = {nullptr, nullptr};
        void *copy_done_events[2] = {nullptr, nullptr};
        bool slot_pending[2] = {false, false};
        int num_range_cells = 0;
        int active_patch_capacity = 0;
        bool initialized = false;
    };

    /// @brief GPU 海杂波生成器状态
    ///
    /// 持有设备端内存指针和配置参数。初始化后通过 generate_clutter_prt_gpu
    /// 反复调用，每次生成一个 PRT 的回波数据。使用完毕后需调用
    /// release_clutter_gpu_state 释放资源。
    struct ClutterGpuState
    {
        Scalar *d_patch_amplitude_map = nullptr;        ///< 设备端静态 patch 幅度图 [az][range]
        Scalar *d_el_amplitude_weight = nullptr;        ///< 设备端距离向俯仰幅度权重数组
        ClutterActivePatch *d_active_patches = nullptr; ///< 设备端活跃方位补丁缓冲区
        DeviceComplex *d_fir_coeffs = nullptr;          ///< FIR 系数全局内存副本，供实验 kernel 使用
        DeviceComplex *d_echo = nullptr;                ///< 设备端输出回波缓冲区
        DeviceComplex *d_patch_contrib = nullptr;       ///< 二维 range×patch 路径的中间贡献缓冲区
        Scalar *d_k_texture_state = nullptr;            ///< K 分布 AR(1) texture 高斯状态
        std::uint64_t *d_k_texture_last_pulse = nullptr; ///< K 分布 texture 最近更新脉冲
        Scalar *d_k_sqrt_tau_lut = nullptr;             ///< K 分布 z->sqrt(tau) 查找表
        int num_range_cells = 0;                ///< 距离单元数量
        int num_az_cells = 0;                   ///< 方位 patch 数量
        int active_patch_capacity = 0;          ///< 活跃方位补丁最大容量
        int fir_length = 0;                     ///< FIR 滤波器长度
        int k_lut_size = 0;                     ///< K 分布 LUT 长度
        bool initialized = false;               ///< 是否已初始化
    };

    /// @brief 单个 PRT 的海杂波生成参数
    ///
    /// 包含脉冲索引、随机种子、天线方向图参数、分布类型及参数等。
    /// 每次调用 generate_clutter_prt_gpu 时传入新的参数。
    struct ClutterPrtParams
    {
        std::uint64_t pulse_index = 0; ///< 当前脉冲索引（用于 FIR 历史和 RNG）
        std::uint64_t seed = 0;        ///< 随机种子

        // 天线方向图参数
        Scalar peak_amplitude_weight = 1.0f; ///< 直接乘到复回波上的峰值幅度权重

        // 幅度分布参数
        Scalar weibull_shape = 2.0f;     ///< Weibull 形状参数 k
        Scalar weibull_scale = 1.0f;     ///< Weibull 尺度参数 λ
        Scalar lognormal_mu = -1.0f;     ///< LogNormal 均值参数 μ
        Scalar lognormal_sigma = 1.0f;   ///< LogNormal 标准差参数 σ
        Scalar k_texture_rho = 0.0f;     ///< K 分布 texture AR(1) 单 PRT 相关系数
        Scalar k_texture_innovation_scale = 0.0f; ///< K 分布连续 PRT AR(1) 创新项系数

        int num_range_cells = 0;         ///< 距离单元数量
        int active_count = 0;            ///< 当前 PRT 的活跃方位补丁数
        int k_lut_size = 0;              ///< K 分布 LUT 长度
        ClutterDistribution distribution = ClutterDistribution::Rayleigh; ///< 幅度分布类型
    };

    /// @brief 初始化海杂波 GPU 状态
    ///
    /// 在 GPU 上分配设备内存，上传 FIR 系数到常量内存，并拷贝静态数据
    /// （二维 patch 幅度图、距离向俯仰幅度权重）到设备端。初始化后可通过 generate_clutter_prt_gpu
    /// 反复调用生成回波。
    ///
    /// @param[out] state 要初始化的 GPU 状态
    /// @param fir_coeffs FIR 滤波器系数（主机端）
    /// @param fir_length FIR 长度（≤ kClutterMaxFirLength）
    /// @param patch_amplitude_map 静态 patch 幅度图（主机端，布局为 [az][range]）
    /// @param el_amplitude_weight 距离向俯仰幅度权重数组（主机端）
    /// @param num_range_cells 距离单元数
    /// @param active_patch_capacity 活跃补丁最大容量
    /// @param num_az_cells 方位 patch 总数
    /// @param[out] error 可选，错误信息
    /// @return true 成功，false 失败
    bool initialize_clutter_gpu_state(ClutterGpuState &state,
                                      const Complex *fir_coeffs,
                                      int fir_length,
                                      const Scalar *patch_amplitude_map,
                                      const Scalar *el_amplitude_weight,
                                      int num_range_cells,
                                      int active_patch_capacity,
                                      int num_az_cells,
                                      const Scalar *k_sqrt_tau_lut = nullptr,
                                      int k_lut_size = 0,
                                      std::string *error = nullptr);

    /// @brief 波束俯仰角变化时更新距离向俯仰幅度权重
    bool update_clutter_gpu_elevation_amplitude_weight(ClutterGpuState &state,
                                                       const Scalar *el_amplitude_weight,
                                                       int num_range_cells,
                                                       std::string *error = nullptr);

    /// @brief 释放海杂波 GPU 状态中的设备内存
    ///
    /// 安全释放所有设备端资源并重置状态。可重复安全调用。
    ///
    /// @param[in,out] state 要释放的 GPU 状态
    void release_clutter_gpu_state(ClutterGpuState &state);

    /// @brief 在 GPU 上按单个 PRT 生成海杂波回波
    ///
    /// 执行一次完整的 GPU 海杂波生成流程：
    /// 1. 上传活跃方位补丁
    /// 2. 启动 GPU 内核进行 FIR 卷积 + 分布变换 + 空间加权
    /// 3. 将结果回传到主机内存
    ///
    /// @param state GPU 状态（需已初始化）
    /// @param params PRT 参数
    /// @param active_patches 活跃方位补丁数组（主机端）
    /// @param[out] out_echo 输出回波缓冲区（主机端）
    /// @param[out] timing 可选，性能计时
    /// @param[out] error 可选，错误信息
    /// @return true 成功，false 失败
    bool generate_clutter_prt_gpu(ClutterGpuState &state,
                                  const ClutterPrtParams &params,
                                  const ClutterActivePatch *active_patches,
                                  Complex *out_echo,
                                  ClutterGpuTiming *timing = nullptr,
                                  std::string *error = nullptr);

    /// @brief 二维 range×patch 并行路径，先计算每个 patch 贡献，再按距离单元归约。
    ///
    /// 该接口用于性能实验，不替换默认的一维距离并行实现。
    bool generate_clutter_prt_gpu_2d(ClutterGpuState &state,
                                     const ClutterPrtParams &params,
                                     const ClutterActivePatch *active_patches,
                                     Complex *out_echo,
                                     ClutterGpuTiming *timing = nullptr,
                                     std::string *error = nullptr);

    /// @brief 二维 range×patch 并行实验路径，包含 K 分布连续 PRT texture 快路径。
    bool generate_clutter_prt_gpu_2d_fast(ClutterGpuState &state,
                                          const ClutterPrtParams &params,
                                          const ClutterActivePatch *active_patches,
                                          Complex *out_echo,
                                          ClutterGpuTiming *timing = nullptr,
                                          std::string *error = nullptr);

    bool initialize_clutter_gpu_stream_state(ClutterGpuStreamState &stream_state,
                                             const ClutterGpuState &base_state,
                                             std::string *error = nullptr);

    void release_clutter_gpu_stream_state(ClutterGpuStreamState &stream_state);

    bool launch_clutter_prt_gpu_2d_fast_stream(ClutterGpuState &base_state,
                                               ClutterGpuStreamState &stream_state,
                                               int slot,
                                               int previous_kernel_slot,
                                               const ClutterPrtParams &params,
                                               const ClutterActivePatch *active_patches,
                                               const Scalar *el_amplitude_weight,
                                               std::string *error = nullptr);

    bool wait_clutter_gpu_stream_slot(ClutterGpuStreamState &stream_state,
                                      int slot,
                                      Complex *out_echo,
                                      ClutterGpuTiming *timing = nullptr,
                                      std::string *error = nullptr);

} // namespace radar::cuda
