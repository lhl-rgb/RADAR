/**
 * @file sea_clutter_config.cpp
 * @brief 海杂波配置参数验证实现
 *
 * 实现 SeaClutterConfig::validate() 方法，对以下字段进行合法性校验：
 * - 后端类型（仅 GPU 可用）
 * - 海况等级范围 [1, 5]
 * - 多普勒谱宽 > 0
 * - 方位 patch 步长 (0, 360]
 * - 活动窗口门限 < 0 dB
 * - 幅度分布类型及对应参数
 * - FIR 长度 ≥ 0
 * - 距离范围（若指定则 min < max）
 */

#include "clutter/sea_clutter_config.h"

namespace radar::clutter
{

    bool SeaClutterConfig::validate(std::string &error) const
    {
        // ---- 后端类型：当前仅支持 GPU ----
        if (backend != SeaClutterBackend::GPU)
        {
            error = "SeaClutterEngine is GPU-only; CPU backend is not implemented";
            return false;
        }

        // ---- 海况等级：Morchin 模型要求 1-5 ----
        if (sea_state < 1.0 || sea_state > 5.0)
        {
            error = "sea_state must be in [1, 5]";
            return false;
        }

        // ---- 多普勒谱宽：必须为正数 ----
        if (doppler_sigma_hz <= 0.0)
        {
            error = "doppler_sigma_hz must be > 0";
            return false;
        }

        // ---- 方位 patch 网格步长：固定场景积分网格 ----
        if (az_patch_step_deg <= 0.0 || az_patch_step_deg > 360.0)
        {
            error = "az_patch_step_deg must be in (0, 360]";
            return false;
        }

        // ---- 活动窗口门限：低于峰值（dB 值必须为负） ----
        if (active_gain_floor_db >= 0.0)
        {
            error = "active_gain_floor_db must be < 0";
            return false;
        }

        // ---- 幅度分布类型 ----
        if (distribution != ClutterDistribution::Rayleigh &&
            distribution != ClutterDistribution::Weibull &&
            distribution != ClutterDistribution::LogNormal &&
            distribution != ClutterDistribution::K)
        {
            error = "unsupported clutter distribution";
            return false;
        }

        // ---- Weibull 参数 ----
        if (weibull_shape <= 0.0)
        {
            error = "weibull_shape must be > 0";
            return false;
        }

        if (weibull_scale <= 0.0)
        {
            error = "weibull_scale must be > 0";
            return false;
        }

        // ---- LogNormal 参数 ----
        if (lognormal_sigma < 0.0)
        {
            error = "lognormal_sigma must be >= 0";
            return false;
        }

        // ---- K 分布参数 ----
        if (k_shape_nu <= 0.0)
        {
            error = "k_shape_nu must be > 0";
            return false;
        }

        if (k_texture_bandwidth_hz <= 0.0)
        {
            error = "k_texture_bandwidth_hz must be > 0";
            return false;
        }

        if (k_lut_size < 16)
        {
            error = "k_lut_size must be >= 16";
            return false;
        }

        // ---- FIR 长度：0 表示自动推导，负数为非法 ----
        if (fir_length < 0)
        {
            error = "fir_length must be >= 0";
            return false;
        }

        // ---- 距离范围：若同时指定则需满足 min < max ----
        if (ground_range_min_m >= 0.0 && ground_range_max_m >= 0.0)
        {
            if (ground_range_min_m >= ground_range_max_m)
            {
                error = "ground_range_min_m must be < ground_range_max_m";
                return false;
            }
        }

        return true;
    }

} // namespace radar::clutter
