#include "core/metrics_collector.h"

#include <algorithm>

namespace radar::core
{
    void MetricsCollector::reset(std::uint64_t total_pulses, float prf_hz)
    {
        metrics_ = SimulationMetrics{};
        metrics_.total_pulses = total_pulses;
        prf_hz_ = prf_hz;
        total_processing_us_ = 0.0;
    }

    void MetricsCollector::record_pulse(std::uint32_t scan,
                                        std::uint64_t pulse,
                                        float target_us,
                                        float clutter_us,
                                        float noise_us,
                                        float compose_us,
                                        float output_us,
                                        float preview_us,
                                        std::uint64_t frames_output,
                                        std::uint64_t dropped_preview_frames)
    {
        metrics_.current_scan = scan;
        metrics_.current_pulse = pulse;
        metrics_.target_us = target_us;
        metrics_.clutter_us = clutter_us;
        metrics_.noise_us = noise_us;
        metrics_.compose_us = compose_us;
        metrics_.output_us = output_us;
        metrics_.preview_us = preview_us;
        metrics_.frames_output = frames_output;
        metrics_.dropped_preview_frames = dropped_preview_frames;

        const double pulse_us = static_cast<double>(target_us) +
                                static_cast<double>(clutter_us) +
                                static_cast<double>(noise_us) +
                                static_cast<double>(compose_us) +
                                static_cast<double>(output_us) +
                                static_cast<double>(preview_us);
        total_processing_us_ += pulse_us;
        const double completed = static_cast<double>(pulse + 1U);
        metrics_.avg_us_per_prt = static_cast<float>(total_processing_us_ / std::max(completed, 1.0));
        metrics_.progress_percent = metrics_.total_pulses == 0
                                        ? 0.0f
                                        : static_cast<float>(100.0 * completed /
                                                             static_cast<double>(metrics_.total_pulses));
        metrics_.realtime_ratio = (metrics_.avg_us_per_prt > 0.0f && prf_hz_ > 0.0f)
                                      ? (1.0e6f / prf_hz_) / metrics_.avg_us_per_prt
                                      : 0.0f;
    }
} // namespace radar::core
