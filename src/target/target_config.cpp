/**
 * @file target_config.cpp
 * @brief 目标配置参数实现
 */

#include "target/target_config.h"
#include "core/math_utils.h"

namespace radar::target {

bool TargetConfig::validate(std::string& error) const {
    if (!math::is_finite(beam_gate_threshold_db)) {
        error = "beam_gate_threshold_db must be finite";
        return false;
    }
    return true;
}

}  // namespace radar::target