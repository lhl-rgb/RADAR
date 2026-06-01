/**
 * @file target_config.hpp
 * @brief 目标模块配置参数
 */

#pragma once

#include "core/types.h"
#include <cstdint>
#include <string>

namespace radar::target {

/**
 * @brief 目标配置
 * @details 包含目标回波生成的行为选项和物理参数
 */
struct TargetConfig {
    // 行为选项
    bool enabled = true;                    ///< 是否启用目标回波生成
    bool enable_swerling = true;            ///< 是否启用 Swerling 起伏
    bool skip_out_of_beam_targets = false;  ///< 是否跳过波束外的目标
    uint64_t seed = 20260330ULL;            ///< 随机种子

    // 物理参数
    bool enable_beam_gain = true;                   ///< 是否启用天线方向增益
    bool enable_two_way_propagation_loss = true;    ///< 是否启用双程传播损耗
    bool enable_phase = true;                       ///< 是否启用传播相位
    Scalar beam_gate_threshold_db = -20.0;          ///< 波束裁剪阈值（dB，相对峰值）

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

}  // namespace radar::target
