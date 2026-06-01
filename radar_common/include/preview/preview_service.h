#pragma once

#include "core/echo_frame.h"
#include "preview/preview_config.h"
#include "preview/preview_types.h"

#include <cstdint>
#include <optional>
#include <string>

namespace radar::preview
{
    class PreviewService
    {
    public:
        void set_config(const PreviewConfig &config) { config_ = config; }
        const PreviewConfig &config() const { return config_; }

        bool initialize(int source_range_bins);
        void reset_scan(std::uint32_t scan_index);
        void consume_frame(const core::EchoFrame &frame);

        bool has_range_profile() const { return latest_range_profile_.has_value(); }
        bool has_ppi_frame() const { return latest_ppi_.has_value(); }
        const RangeProfilePreview *latest_range_profile() const;
        const PpiPreviewFrame *latest_ppi_frame() const;
        std::uint64_t dropped_frames() const { return dropped_frames_; }
        const std::string &last_error() const { return last_error_; }

    private:
        float sample_power_db(const radar::Complex &sample) const;

        PreviewConfig config_;
        int source_range_bins_ = 0;
        std::uint32_t current_scan_index_ = 0;
        std::optional<RangeProfilePreview> latest_range_profile_;
        std::optional<PpiPreviewFrame> latest_ppi_;
        std::uint64_t dropped_frames_ = 0;
        std::string last_error_;
    };
} // namespace radar::preview
