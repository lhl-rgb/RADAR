/**
 * @file target_engine.h
 * @brief 点目标回波生成引擎（编排层）
 */

#pragma once

#include "antenna/antenna_model.h"
#include "core/radar_system_params.h"
#include "core/types.h"
#include "target/target_config.h"
#include "target/target_echo_synthesizer.h"
#include "target/target_kinematics.h"
#include "target/target_manager.h"

#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace radar::target {

/**
 * @brief 点目标回波生成引擎（统一管理器）
 */
class TargetEngine {
public:
    explicit TargetEngine() = default;
    ~TargetEngine() = default;

    TargetEngine(const TargetEngine&) = delete;
    TargetEngine& operator=(const TargetEngine&) = delete;

    void set_config(const TargetConfig& cfg) { cfg_ = cfg; }

    bool initialize();
    bool is_initialized() const { return initialized_; }
    const std::string& last_error() const { return last_error_; }

    void set_initial_targets(const TargetList& targets);
    void update_targets(Scalar current_time_s);
    const TargetList& get_current_targets() const;
    void clear_targets();
    std::size_t target_count() const;

    bool is_enabled() const { return cfg_.enabled; }

    /**
     * @brief 生成单脉冲目标回波（逐PRT接口）
     */
    bool generate_pulse(const BeamView& beam,
                        const PulseContext& pulse,
                        const RadarSystemParams& system,
                        const ComplexVec& tx_waveform,
                        PulseEcho& out_echo);

    const TargetConfig& config() const { return cfg_; }
    const TargetList& initial_targets() const { return initial_targets_; }

private:
    struct TargetFluctuationState {
        Scalar current_rcs_m2 = 0.0;
        bool has_current_rcs = false;
        bool was_illuminated = false;
        uint64_t visit_count = 0;
        std::mt19937_64 rng{};
        bool rng_seeded = false;
    };

    void reset_fluctuation_states();
    TargetFluctuationState& ensure_fluctuation_state(const TargetState& target);
    bool is_target_illuminated(const TargetState& target,
                               const BeamView& beam) const;
    Scalar resolve_rcs_for_pulse(const TargetState& target,
                                 const BeamView& beam);
    TargetList generate_default_targets() const;

    TargetConfig cfg_;
    TargetList initial_targets_;
    bool initialized_ = false;
    std::string last_error_;
    TargetManager target_manager_;
    std::unordered_map<uint64_t, TargetFluctuationState> fluctuation_states_;
};

} // namespace radar::target
