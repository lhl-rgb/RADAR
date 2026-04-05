/**
 * @file antenna_config.cpp
 * @brief 天线模块配置参数实现
 */

#include "antenna/antenna_config.h"

namespace radar::antenna {

bool AntennaConfig::validate(std::string& error) const {
    if (num_elements_az <= 0) {
        error = "num_elements_az must be positive";
        return false;
    }
    if (spacing_az_lambda <= 0.0) {
        error = "spacing_az_lambda must be positive";
        return false;
    }
    if (peak_gain_db < 0.0) {
        error = "peak_gain_db cannot be negative";
        return false;
    }

    if (model_type == PhasedArrayModelType::UPA_2D) {
        if (num_elements_el <= 0) {
            error = "num_elements_el must be positive for UPA_2D model";
            return false;
        }
        if (spacing_el_lambda <= 0.0) {
            error = "spacing_el_lambda must be positive for UPA_2D model";
            return false;
        }
    }

    return true;
}

}  // namespace radar::antenna
