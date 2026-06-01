#include "preview/preview_service.h"

#include <algorithm>
#include <cmath>

namespace radar::preview
{
    bool PreviewService::initialize(int source_range_bins)
    {
        std::string error;
        if (!config_.validate(error))
        {
            last_error_ = error;
            return false;
        }

        source_range_bins_ = source_range_bins;
        if (source_range_bins_ <= 0)
        {
            last_error_ = "PreviewService source range bins must be positive";
            return false;
        }

        reset_scan(0);
        last_error_.clear();
        return true;
    }

    void PreviewService::reset_scan(std::uint32_t scan_index)
    {
        current_scan_index_ = scan_index;
        latest_range_profile_.reset();
        latest_ppi_ = PpiPreviewFrame{};
        latest_ppi_->scan_index = scan_index;
        latest_ppi_->range_bins = config_.ppi_range_bins;
        latest_ppi_->az_bins = config_.ppi_az_bins;
        latest_ppi_->dynamic_range_db = config_.dynamic_range_db;
        latest_ppi_->image.assign(static_cast<std::size_t>(config_.ppi_range_bins * config_.ppi_az_bins), 0U);
    }

    void PreviewService::consume_frame(const core::EchoFrame &frame)
    {
        if (!config_.enabled)
        {
            return;
        }
        if (frame.iq.empty() || source_range_bins_ <= 0)
        {
            ++dropped_frames_;
            return;
        }

        if (frame.scan_index != current_scan_index_)
        {
            reset_scan(frame.scan_index);
        }

        if (frame.pulse_index % static_cast<std::uint64_t>(config_.range_profile_stride_pulses) == 0ULL)
        {
            RangeProfilePreview profile;
            profile.pulse_index = frame.pulse_index;
            profile.beam_az_deg = frame.beam_az_deg;
            profile.power_db.resize(frame.iq.size());
            for (std::size_t i = 0; i < frame.iq.size(); ++i)
            {
                profile.power_db[i] = sample_power_db(frame.iq[i]);
            }
            latest_range_profile_ = std::move(profile);
        }

        if (!latest_ppi_.has_value() || config_.ppi_range_bins <= 0 || config_.ppi_az_bins <= 0)
        {
            return;
        }

        int az_bin = static_cast<int>(std::floor(frame.beam_az_deg / 360.0f * config_.ppi_az_bins));
        az_bin = std::clamp(az_bin, 0, config_.ppi_az_bins - 1);

        float peak_db = -240.0f;
        for (const auto &sample : frame.iq)
        {
            peak_db = std::max(peak_db, sample_power_db(sample));
        }
        const float floor_db = peak_db - config_.dynamic_range_db;

        for (int out_r = 0; out_r < config_.ppi_range_bins; ++out_r)
        {
            const int src_r = std::min(
                static_cast<int>((static_cast<long long>(out_r) * static_cast<long long>(frame.iq.size())) /
                                 static_cast<long long>(config_.ppi_range_bins)),
                static_cast<int>(frame.iq.size()) - 1);
            const float db = sample_power_db(frame.iq[static_cast<std::size_t>(src_r)]);
            const float normalized = std::clamp((db - floor_db) / config_.dynamic_range_db, 0.0f, 1.0f);
            const auto pixel = static_cast<std::uint8_t>(std::lround(normalized * 255.0f));
            latest_ppi_->image[static_cast<std::size_t>(az_bin * config_.ppi_range_bins + out_r)] = pixel;
        }
    }

    const RangeProfilePreview *PreviewService::latest_range_profile() const
    {
        return latest_range_profile_.has_value() ? &(*latest_range_profile_) : nullptr;
    }

    const PpiPreviewFrame *PreviewService::latest_ppi_frame() const
    {
        return latest_ppi_.has_value() ? &(*latest_ppi_) : nullptr;
    }

    float PreviewService::sample_power_db(const radar::Complex &sample) const
    {
        const float power = std::max(static_cast<float>(std::norm(sample)), 1.0e-30f);
        return 10.0f * std::log10(power);
    }
} // namespace radar::preview
