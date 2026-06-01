#pragma once

#include "core/types.h"

#include <cstdint>

namespace radar::core
{
    struct EchoFrame
    {
        std::uint64_t pulse_index = 0;
        std::uint32_t scan_index = 0;
        Scalar timestamp_s = 0.0f;
        Scalar beam_az_deg = 0.0f;
        Scalar beam_el_deg = 0.0f;
        Scalar range_bin_size_m = 0.0f;
        PulseEcho iq;
    };
} // namespace radar::core
