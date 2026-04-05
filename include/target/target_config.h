/**
 * @file target_config.hpp
 * @brief 目标模块配置参数
 */

#pragma once

#include "core/types.h"
#include <cstdint>
#include <string>
#include <nlohmann/json.hpp>

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

// JSON 序列化支持
inline void from_json(const nlohmann::json& j, TargetConfig& cfg) {
    // 支持扁平化格式（向后兼容）
    if (j.contains("enabled")) j.at("enabled").get_to(cfg.enabled);
    if (j.contains("enable_swerling")) j.at("enable_swerling").get_to(cfg.enable_swerling);
    if (j.contains("seed")) j.at("seed").get_to(cfg.seed);
    if (j.contains("enable_beam_gain")) j.at("enable_beam_gain").get_to(cfg.enable_beam_gain);
    if (j.contains("enable_two_way_propagation_loss")) j.at("enable_two_way_propagation_loss").get_to(cfg.enable_two_way_propagation_loss);
    if (j.contains("enable_phase")) j.at("enable_phase").get_to(cfg.enable_phase);
    if (j.contains("beam_gate_threshold_db")) j.at("beam_gate_threshold_db").get_to(cfg.beam_gate_threshold_db);

    // 支持嵌套格式（新格式）
    if (j.contains("options")) {
        const auto& opts = j.at("options");
        if (opts.contains("enabled")) opts.at("enabled").get_to(cfg.enabled);
        if (opts.contains("enable_swerling")) opts.at("enable_swerling").get_to(cfg.enable_swerling);
        if (opts.contains("skip_out_of_beam_targets")) opts.at("skip_out_of_beam_targets").get_to(cfg.skip_out_of_beam_targets);
        if (opts.contains("seed")) opts.at("seed").get_to(cfg.seed);
    }

    if (j.contains("params")) {
        const auto& params = j.at("params");
        if (params.contains("enable_beam_gain")) params.at("enable_beam_gain").get_to(cfg.enable_beam_gain);
        if (params.contains("enable_two_way_propagation_loss")) params.at("enable_two_way_propagation_loss").get_to(cfg.enable_two_way_propagation_loss);
        if (params.contains("enable_phase")) params.at("enable_phase").get_to(cfg.enable_phase);
        if (params.contains("beam_gate_threshold_db")) params.at("beam_gate_threshold_db").get_to(cfg.beam_gate_threshold_db);
    }
}

inline void to_json(nlohmann::json& j, const TargetConfig& cfg) {
    j = nlohmann::json{
        {"enabled", cfg.enabled},
        {"enable_swerling", cfg.enable_swerling},
        {"skip_out_of_beam_targets", cfg.skip_out_of_beam_targets},
        {"seed", cfg.seed},
        {"enable_beam_gain", cfg.enable_beam_gain},
        {"enable_two_way_propagation_loss", cfg.enable_two_way_propagation_loss},
        {"enable_phase", cfg.enable_phase},
        {"beam_gate_threshold_db", cfg.beam_gate_threshold_db}
    };
}

}  // namespace radar::target
