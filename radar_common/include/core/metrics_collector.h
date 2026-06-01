#pragma once

#include <cstdint>

namespace radar::core
{
    struct SimulationMetrics
    {
        std::uint32_t current_scan = 0;
        std::uint64_t current_pulse = 0;
        std::uint64_t total_pulses = 0;
        float progress_percent = 0.0f;
        float avg_us_per_prt = 0.0f;
        float realtime_ratio = 0.0f;
        float target_us = 0.0f;
        float clutter_us = 0.0f;
        float noise_us = 0.0f;
        float compose_us = 0.0f;
        float output_us = 0.0f;
        float preview_us = 0.0f;
        std::uint64_t frames_output = 0;
        std::uint64_t dropped_preview_frames = 0;
    };

    class MetricsCollector
    {
    public:
        void reset(std::uint64_t total_pulses, float prf_hz);
        void record_pulse(std::uint32_t scan,
                          std::uint64_t pulse,
                          float target_us,
                          float clutter_us,
                          float noise_us,
                          float compose_us,
                          float output_us,
                          float preview_us,
                          std::uint64_t frames_output,
                          std::uint64_t dropped_preview_frames);
        const SimulationMetrics &metrics() const { return metrics_; }

    private:
        SimulationMetrics metrics_;
        float prf_hz_ = 0.0f;
        double total_processing_us_ = 0.0;
    };
} // namespace radar::core
