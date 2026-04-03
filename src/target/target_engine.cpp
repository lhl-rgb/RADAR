/**
 * @file target_engine.cpp
 * @brief 点目标回波生成引擎实现
 */

#include "target/target_engine.h"
#include "core/math_utils.h"

#include <stdexcept>

namespace radar::target {

// ============================================================================
// 初始化
// ============================================================================

bool TargetEngine::initialize() {
    // 如果没有设置初始目标，生成默认目标
    if (initial_targets_.empty()) {
        initial_targets_ = generate_default_targets();
    }

    // 设置目标到管理器
    target_manager_.set_targets(initial_targets_);

    initialized_ = true;
    return true;
}

// ============================================================================
// 目标状态管理（委托给 TargetManager）
// ============================================================================

void TargetEngine::set_initial_targets(const TargetList& targets) {
    initial_targets_ = targets;
    if (initialized_) {
        target_manager_.set_targets(targets);
    }
}

void TargetEngine::update_targets(Scalar current_time_s) {
    target_manager_.update_to_time(current_time_s);
}

const TargetList& TargetEngine::get_current_targets() const {
    return target_manager_.get_current_targets();
}

void TargetEngine::clear_targets() {
    target_manager_.clear();
}

std::size_t TargetEngine::target_count() const {
    return target_manager_.size();
}

// ============================================================================
// 回波生成
// ============================================================================

bool TargetEngine::init_output_echo(const BeamView& beam,
                                    const RadarSystemParams& system,
                                    CpiEcho& out_echo,
                                    std::string& error) const {
    error.clear();

    if (system.pulses_per_cpi <= 0 || system.samples_per_pulse <= 0) {
        error = "RadarSystemParams invalid: pulses_per_cpi and samples_per_pulse must be positive.";
        return false;
    }

    out_echo.beam_index = beam.beam_index;
    out_echo.azimuth_deg = beam.pointing.azimuth_deg;
    out_echo.elevation_deg = beam.pointing.elevation_deg;
    out_echo.pulses.assign(static_cast<std::size_t>(system.pulses_per_cpi),
                           PulseEcho(static_cast<std::size_t>(system.samples_per_pulse), Complex(0.0, 0.0)));
    return true;
}

bool TargetEngine::generate(const BeamView& beam,
                            const RadarSystemParams& system,
                            const ComplexVec& tx_waveform,
                            CpiEcho& out_echo) {
    last_error_.clear();

    if (tx_waveform.empty()) {
        last_error_ = "TargetEngine invalid input: tx_waveform must not be empty.";
        return false;
    }

    if (!cfg_.enabled) {
        return true;
    }

    // 获取当前目标状态
    const TargetList& active_targets = get_current_targets();

    std::string error;
    if (!init_output_echo(beam, system, out_echo, error)) {
        last_error_ = error;
        return false;
    }

    TrajectoryBatch trajectories;
    if (!TargetKinematics::generate_trajectories(active_targets, beam, system, cfg_, trajectories, error)) {
        last_error_ = error;
        return false;
    }

    if (!TargetEchoSynthesizer::generate_target_echoes(trajectories, system, cfg_,
                                                       tx_waveform, out_echo, error)) {
        last_error_ = error;
        return false;
    }

    return true;
}

CpiEcho TargetEngine::generate_or_throw(const BeamView& beam,
                                        const RadarSystemParams& system,
                                        const ComplexVec& tx_waveform) {
    CpiEcho echo;
    if (!generate(beam, system, tx_waveform, echo)) {
        throw std::runtime_error(last_error_);
    }
    return echo;
}

// ============================================================================
// 内部实现
// ============================================================================

TargetList TargetEngine::generate_default_targets() const {
    return {
        {1, Vec3(50000.0, 0.0, 5000.0), Vec3::Zero(), Vec3::Zero(),
         MotionModel::Stationary, 10.0, SwerlingType::Swerling1, true},
        {2, Vec3(30000.0, 10000.0, 3000.0), Vec3(100.0, 50.0, 0.0), Vec3::Zero(),
         MotionModel::ConstantVelocity, 5.0, SwerlingType::Swerling1, true}
    };
}

}  // namespace radar::target
