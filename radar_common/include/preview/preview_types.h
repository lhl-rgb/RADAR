#pragma once

#include <cstdint>
#include <vector>

namespace radar::preview
{
    struct RangeProfilePreview
    {
        std::uint64_t pulse_index = 0;
        float beam_az_deg = 0.0f;
        std::vector<float> power_db;
    };

    struct PpiPreviewFrame
    {
        std::uint32_t scan_index = 0;
        int range_bins = 0;
        int az_bins = 0;
        float dynamic_range_db = 50.0f;
        std::vector<std::uint8_t> image;
    };
} // namespace radar::preview
