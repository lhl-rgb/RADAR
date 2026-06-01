一、整体架构总览
Text
┌─────────────────────────────────────────────────────────────────────┐
│                          冷启动期（一次性）                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────────┐  │
│  │ 系统参数装载 │→│ A_static 地图 │→│ FIR 系数 h[k] 上传        │  │
│  │ Pt,λ,L_sys.. │  │ 预计算 + 上传 │  │ 天线方向图查找表上传     │  │
│  └──────────────┘  └──────────────┘  └──────────────────────────┘  │
│                                  │                                  │
│                       seed = generate_seed()                        │
│                       n = 0                                         │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        每 PRT 主循环（实时）                         │
│                                                                     │
│  ┌─── CPU 侧 (~10 μs) ────────────────────────────────────────┐    │
│  │ 1. θ_beam(n) = beam_pointing(n)                            │    │
│  │ 2. 扫天线方向图 → 生成 active_patch[]                      │    │
│  │ 3. cudaMemcpyAsync 上传 active 列表 (~128 B)               │    │
│  └────────────────────────────────────────────────────────────┘    │
│                                  │                                  │
│  ┌─── GPU 侧 (~200 μs) ───────────────────────────────────────┐    │
│  │ 4. clutter_kernel<<<...>>>  → z_clutter[N_r]               │    │
│  │ 5. thermal_kernel<<<...>>>  → 加热噪                       │    │
│  │ 6. target_kernel<<<...>>>   → 加目标                       │    │
│  │ 7. 后续信号处理（脉压、MTD、CFAR…）                        │    │
│  └────────────────────────────────────────────────────────────┘    │
│                                  │                                  │
│                            n ← n + 1                                │
│                            ↑─────────┘                              │
└─────────────────────────────────────────────────────────────────────┘
整个 PRT 之间在 GPU 上需要保留的算法状态只有 16 字节：(seed, n)。


二、冷启动期：四步初始化

步骤 1：系统参数与几何参数装载
Cpp
struct SysParams {
    float Pt;             // 发射功率 W
    float lambda;         // 波长 m
    float L_sys;          // 系统损耗（线性）
    float c_tau_2;        // 距离分辨率 = c·τ/2
    float dTheta_az;      // 方位 patch 角宽度
    float R0;             // 第一个距离单元对应的斜距
    float dR;             // 距离采样间隔
    int   N_r, N_az;      // 距离/方位 patch 数
    int   L;              // FIR 阶数
    int   PRF_Hz;
    float omega_scan;     // 天线转速 rad/s
};
CPU 读配置文件，填好这个结构。

步骤 2：A_static 静态幅度图预计算
把雷达方程里所有"与瞬时波束指向无关"的因子全部打包：
Cpp
void build_A_static(const SysParams& p,
                    const float*    sigma0_map,    // 输入：地表后向散射系数 [N_r×N_az]
                    float*          A_static_out)  // 输出：电压幅度图 [N_r×N_az]
{
    const float K_const = p.Pt * p.lambda * p.lambda
                        / (powf(4.f * M_PI, 3.f) * p.L_sys);

    for (int r = 0; r < p.N_r; ++r) {
        float R_r    = p.R0 + r * p.dR;
        float A_cell = R_r * p.dTheta_az * p.c_tau_2;       // 杂波单元面积
        float K_r    = K_const * A_cell / powf(R_r, 4.f);    // r 行公共因子

        for (int t = 0; t < p.N_az; ++t) {
            float P_base = K_r * sigma0_map[r * p.N_az + t];
            A_static_out[r * p.N_az + t] = sqrtf(P_base);    // ← 电压
        }
    }
}
完成后上传 GPU：
Cpp
cudaMalloc(&d_A_static, N_r * N_az * sizeof(float));
cudaMemcpy(d_A_static, A_static_host, ..., cudaMemcpyHostToDevice);
显存占用：N_r × N_az × 4 B ≈ 8 MB。

步骤 3：FIR 系数与天线方向图
FIR 系数 h[k]（决定杂波多普勒谱形状）：
Cpp
// 由杂波 PSD 设计 FIR：σ_f 谱宽决定 h 形状
design_fir_coeffs(sigma_f, fc_doppler, L, h_fir);
normalize_unit_power(h_fir, L);   // 归一：Σ|h[k]|² = 1

cudaMemcpyToSymbol(c_h_fir, h_fir, L * sizeof(float2));   // constant memory
双程功率方向图查找表：
Cpp
// 把 G²(Δθ) 离散化成 LUT，分辨率约 0.01°
const int   N_LUT = 36000;
const float dTheta_LUT = 2*M_PI / N_LUT;
float G2_power_LUT[N_LUT];

for (int i = 0; i < N_LUT; ++i) {
    float dt = (i - N_LUT/2) * dTheta_LUT;
    G2_power_LUT[i] = antenna_pattern_2way(dt);   // sinc², Taylor, 实测...
}
LUT 留在 CPU（每 PRT 只查约 16 个 patch，毋需上 GPU）。

步骤 4：随机种子与 PRT 计数器
Cpp
uint64_t seed = generate_seed();   // 来自 OS、用户配置或固定值（用于复现）
uint64_t n    = 0;                  // PRT 序号
整个仿真器需要在 PRT 间持久化的状态就是这两个 64-bit 整数。


三、每 PRT 主循环：CPU 侧逻辑

3.1 计算波束指向
Cpp
float theta_beam = fmodf(p.omega_scan * (n / (float)p.PRF_Hz), 2.f * M_PI);
机扫雷达直接按转速；电扫按调度表。

3.2 构建 active patch 列表
Cpp
int build_active_list(
    float          theta_beam,
    const float*   G2_power_LUT,
    int            N_LUT,
    float          dTheta_LUT,
    int            N_az,
    float          dTheta_az,
    float          gain_thresh_dB,    // 例如 -40 dB
    ActivePatch*   out)               // 上限 MAX_ACTIVE = 64
{
    float thresh_lin = powf(10.f, gain_thresh_dB / 10.f);
    int   count = 0;

    for (int t = 0; t < N_az; ++t) {
        float theta_p = t * dTheta_az;
        float dtheta  = wrap_to_pi(theta_p - theta_beam);

        int   idx    = (int)((dtheta + M_PI) / dTheta_LUT);
        float G2_pow = G2_power_LUT[idx];

        if (G2_pow < thresh_lin) continue;

        out[count].theta_idx    = t;
        out[count].gain_voltage = sqrtf(G2_pow);    // 电压方向图
        count++;
    }
    return count;
}
主瓣 3 dB 宽度约 1°，加近副瓣到 −40 dB → N_active ≈ 16。

3.3 上传 active 列表（异步）
Cpp
struct ActivePatch {
    int   theta_idx;       // 4 B
    float gain_voltage;    // 4 B
};   // 总 8 B

cudaMemcpyAsync(d_active, h_active,
                N_active * sizeof(ActivePatch),
                cudaMemcpyHostToDevice, stream);
数据量约 16 × 8 = 128 字节，PCIe 传输忽略不计。


四、每 PRT 主循环：GPU 侧 Kernel

4.1 Kernel 启动参数
Cpp
const int  THREADS = 256;
const int  blocks  = (N_r + THREADS - 1) / THREADS;
const size_t shm   = N_active * sizeof(ActivePatch);

clutter_kernel<<<blocks, THREADS, shm, stream>>>(
    seed, n, L, N_r, N_az,
    d_active, N_active,
    d_A_static, d_z_out);
线程映射：每个 thread 算一个距离单元 r。block 内 thread 共享 active 列表（shared memory）。

4.2 完整 Kernel 代码
Cpp
__constant__ float2 c_h_fir[L_MAX];   // FIR 系数（复数）

__global__ void clutter_kernel(
    uint64_t        seed,
    uint64_t        n,
    int             L,
    int             N_r,
    int             N_az,
    const ActivePatch* __restrict__ active,
    int             N_active,
    const float*    __restrict__ A_static,   // 静态电压幅度图
    float2*         __restrict__ z_out)      // 输出杂波回波
{
    int r = blockIdx.x * blockDim.x + threadIdx.x;
    if (r >= N_r) return;

    // ─── A. active 列表搬到 shared memory ───
    extern __shared__ ActivePatch s_active[];
    for (int i = threadIdx.x; i < N_active; i += blockDim.x)
        s_active[i] = active[i];
    __syncthreads();

    // ─── B. 主循环：对每个 active patch 做 FIR ───
    float2 z_acc = make_float2(0.f, 0.f);

    for (int p = 0; p < N_active; ++p) {
        const int   theta_p = s_active[p].theta_idx;
        const float G_v     = s_active[p].gain_voltage;       // 电压方向图
        const float A_amp   = A_static[r * N_az + theta_p];   // 静态幅度

        // 预组装 counter 的 (r, theta_p) 部分（外层不变）
        const uint32_t ctr_xy = ((uint32_t)theta_p << 16) | (uint32_t)r;

        // —— FIR 卷积，c_norm 是单位功率复白高斯滤波输出 ——
        float2 c_norm = make_float2(0.f, 0.f);

        // 优化：每次 Philox 调用产 4 个 uint32 = 2 个复样本，分给两个 tap
        for (int k = 0; k < L; k += 2) {
            int64_t nk0 = (int64_t)n - k;
            int64_t nk1 = nk0 - 1;

            uint4 rnd = philox4x32_10_packed(seed, ctr_xy, nk0);
            float2 w0 = uint2_to_complex_gauss(rnd.x, rnd.y);
            float2 w1 = uint2_to_complex_gauss(rnd.z, rnd.w);

            float2 h0 = c_h_fir[k];
            float2 h1 = c_h_fir[k + 1];

            // c_norm += h0 * w0 + h1 * w1
            c_norm.x += h0.x*w0.x - h0.y*w0.y + h1.x*w1.x - h1.y*w1.y;
            c_norm.y += h0.x*w0.y + h0.y*w0.x + h1.x*w1.y + h1.y*w1.x;
        }

        // ─── C. 把雷达方程的物理因子乘进去 ───
        const float scale = G_v * A_amp;     // = √P_c 的电压幅度
        z_acc.x += scale * c_norm.x;
        z_acc.y += scale * c_norm.y;
    }

    z_out[r] = z_acc;
}

4.3 Philox 包装（device 函数）
Cpp
__device__ __forceinline__
uint4 philox4x32_10_packed(uint64_t seed, uint32_t ctr_xy, int64_t nk)
{
    uint4 ctr;
    ctr.x = ctr_xy;
    ctr.y = (uint32_t)(nk & 0xFFFFFFFF);
    ctr.z = (uint32_t)((uint64_t)nk >> 32);
    ctr.w = STREAM_CLUTTER;            // 流标识

    uint2 key;
    key.x = (uint32_t)(seed & 0xFFFFFFFF);
    key.y = (uint32_t)(seed >> 32);

    return philox4x32_10(ctr, key);    // CUDA 内置或 ~30 行手写
}

__device__ __forceinline__
float2 uint2_to_complex_gauss(uint32_t a, uint32_t b)
{
    // Box-Muller，输出实/虚各 N(0, 1/2)，使 |w|² 期望 = 1
    float u1 = __uint_as_float((a >> 9) | 0x3F800000) - 1.f;   // [0,1)
    if (u1 < 1e-38f) u1 = 1e-38f;
    float u2 = __uint_as_float((b >> 9) | 0x3F800000) - 1.f;

    float r  = sqrtf(-logf(u1));      // 注意：少一个 sqrt(2)，配合下面 inv_sqrt2
    float s, c;
    __sincosf(2.f * M_PI * u2, &s, &c);
    return make_float2(r * c, r * s);
}


五、物理量流向梳理（最重要的一张表）
把每一个物理因子放在哪一层，标得清清楚楚：











































































物理量何时算存哪里大小kernel 内代价Pt, λ², (4π)³, L_sys启动标量 K_const4 B已并入 A_staticσ⁰(r, θ_p) 散射图启动sigma0_map8 MB已并入 A_staticA_cell(r) = R·Δθ·cτ/2启动per-r 行因子已并入已并入 A_staticR(r)⁴ 距离衰减启动per-r 行因子已并入已并入 A_staticA_static(r,θ_p)启动d_A_static8 MB1 次访存G²(Δθ) 双程方向图每 PRTactive_patch.gain_voltage128 B1 次访存h[k] FIR 系数启动c_h_fir (constant)2 KBL 次访存（缓存）w(r,θ_p,n−k) 白噪现算不存0L 次 Philoxseed, n持久标量16 Bkernel 参数
关键点：

静态物理因子全部摊到启动期，一次乘法解决
动态部分只有 G²(Δθ)，每 PRT 仅算 16 个值
白噪声完全不存，FIR 现算



六、性能分析
参数：N_r = 2000，N_active = 16，L = 256，PRF = 2000 Hz。

单 PRT 计算量
Text
RNG 调用次数 = N_r × N_active × (L/2) = 2000 × 16 × 128 = 4.1 × 10⁶
复 FMA 次数  = N_r × N_active × L     = 8.2 × 10⁶
访存量       ≈ N_r × N_active × 4 B   = 128 KB（A_static 那部分）

时间估算（A100 / RTX 3080 量级）





























项目时间Philox（4×10⁶ 次）~40 μs复 FMA + 累加~80 μsA_static 访存~30 μs启动开销 + sync~20 μs总计~170 μs
PRT 间隔 1/2000 = 500 μs。
实时余量 ≈ 65%。够装下后续脉压、MTD、CFAR、显示渲染。


七、数据流时序图
Text
PRT n:
  CPU:  ├─ θ_beam ─┤
        ├─ active list ─┤
                        ├─ memcpy ─┤
  GPU:                              ├──── clutter_kernel ────┤
                                                              ├─ thermal ─┤
                                                                          ├─ targets ─┤
                                                                                      ├─ pulse_compress ─┤

PRT n+1:
  CPU:  ├─ θ_beam ─┤
        ├─ active list ─┤
                        ├─ memcpy ─┤
  GPU:                                              ├──── clutter_kernel ────┤
                                                                              ├─ ...
CPU 准备和 GPU kernel 在不同 stream 上完全重叠，CPU 侧 10 μs 的工作完全藏在 GPU 计算时间里。


八、扩展性与边界条件

多源信号（杂波 / 热噪 / 目标）
同一个 Philox 调用框架，靠 stream_id 字段区分：
Cpp
#define STREAM_CLUTTER  0xC1077E00u
#define STREAM_THERMAL  0x7E110000u
#define STREAM_TARGET   0x7A767E70u
热噪声 kernel 只用 (r, n)，目标 kernel 用 (target_id, n)。所有源共用一个 seed，统计上独立。

大场景（N_r = 8000，N_az = 3600）
A_static = 8000 × 3600 × 4 = 115 MB，仍轻松放下单卡。
kernel 时间线性增长，A100 上估约 700 μs，单卡接近极限。多 stream + 双卡可拓展至更大场景。

副瓣 / 远副瓣杂波研究
把 gain_thresh_dB 从 −40 dB 调到 −60 dB，N_active 涨到约 100~200。kernel 时间约 1 ms，仍可单卡跑（PRF 降到 1 kHz）。算法零修改。

断点续算 / 单帧重现
Cpp
// 保存：
checkpoint = { seed, n };   // 16 B
// 恢复：
seed = checkpoint.seed; n = checkpoint.n;
// 直接从这个 PRT 开始跑，回波逐 bit 一致

多 GPU / 多机一致性
主控广播 (seed, n)，各节点独立计算自己负责的距离段或方位段，结果天然一致，无需任何同步。


九、CPU 主循环骨架
Cpp
void run_simulation(const SysParams& p, uint64_t seed) {
    // ── 启动期 ──
    float* d_A_static; build_and_upload_A_static(p, sigma0_map, &d_A_static);
    upload_fir_coeffs(h_fir, p.L);
    build_antenna_lut(G2_power_LUT);

    cudaStream_t stream;
    cudaStreamCreate(&stream);

    ActivePatch* d_active; cudaMalloc(&d_active, MAX_ACTIVE * sizeof(ActivePatch));
    float2*      d_z_out;  cudaMalloc(&d_z_out,  p.N_r * sizeof(float2));

    ActivePatch h_active[MAX_ACTIVE];

    // ── 主循环 ──
    for (uint64_t n = 0; n < N_TOTAL; ++n) {

        // CPU
        float theta_beam = compute_beam_pointing(n, p);
        int   N_active   = build_active_list(theta_beam, G2_power_LUT, ...,
                                              h_active);

        cudaMemcpyAsync(d_active, h_active,
                        N_active * sizeof(ActivePatch),
                        cudaMemcpyHostToDevice, stream);

        // GPU
        int blocks = (p.N_r + 255) / 256;
        size_t shm = N_active * sizeof(ActivePatch);

        clutter_kernel<<<blocks, 256, shm, stream>>>(
            seed, n, p.L, p.N_r, p.N_az,
            d_active, N_active, d_A_static, d_z_out);

        thermal_kernel<<<blocks, 256, 0, stream>>>(seed, n, p.N_r, d_z_out);
        target_kernel <<<...,  stream>>>(seed, n, d_targets, N_tgt, d_z_out);

        // 后续处理...
        pulse_compress<<<..., stream>>>(d_z_out, d_z_pc);
        // mti / mtd / cfar / display ...
    }

    cudaStreamDestroy(stream);
    // 释放...
}


十、流程总结（一张图记住）
Text
冷启动一次:  Pt,λ,L_sys,σ⁰,A_cell,R⁴ ──┐
                                       ├──→ A_static[N_r×N_az]  (8 MB, GPU)
             σ⁰_map ───────────────────┘
             h[k] (Σ|h|²=1) ──────────────→ c_h_fir[L]  (2 KB, constant)
             G²(Δθ) 解析/实测 ─────────────→ G2_power_LUT  (CPU)
             seed, n=0

每 PRT:
  CPU:  n, ω_scan ──→ θ_beam(n) ──→ 扫 LUT ──→ active_patch[~16]  ──→ memcpyAsync
                                              (theta_idx, G_v=√G²)

  GPU kernel (per thread = per r):
        for p in active:
            G_v   ← active[p].gain_voltage
            A_amp ← A_static[r, theta_p]
            c_norm = Σ_{k=0..L-1} h[k] · philox_gauss(seed, r, theta_p, n-k)
            z[r] += (G_v · A_amp) · c_norm        ← 雷达方程在此落地

        out: z_clutter[r] = z[r]

  n ← n+1

一句话总结整套流程：

把雷达方程的所有静态因子在启动期吸收成一张 8 MB 的电压幅度图 A_static，把瞬时变化的双程方向图 G²(Δθ) 在每 PRT 缩成约 16 个 active patch 的 128 字节列表，把白噪历史样本完全用 Philox counter-based RNG 现算消除，kernel 内每个距离单元的运算就是「16 patch × 256 tap 的复 FMA + 一次 G·A·c_norm 缩放」，单 PRT GPU 时间 ≈ 200 μs，实时余量 60%+，状态量永远只有 (seed, n) 这 16 字节。