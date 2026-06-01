#pragma once

#include <string>

namespace radar::preview
{
    struct PreviewConfig
    {
        bool enabled = true;
        int range_profile_stride_pulses = 50;
        int ppi_range_bins = 512;
        int ppi_az_bins = 720;
        float dynamic_range_db = 50.0f;

        bool validate(std::string &error) const
        {
            if (!enabled)
            {
                return true;
            }
            if (range_profile_stride_pulses <= 0)
            {
                error = "preview.range_profile_stride_pulses must be positive";
                return false;
            }
            if (ppi_range_bins <= 0 || ppi_az_bins <= 0)
            {
                error = "preview PPI dimensions must be positive";
                return false;
            }
            if (dynamic_range_db <= 0.0f)
            {
                error = "preview.dynamic_range_db must be positive";
                return false;
            }
            return true;
        }
    };
} // namespace radar::preview
