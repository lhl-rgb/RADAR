#pragma once

#include "output/echo_sink.h"

#include <fstream>

namespace radar::output
{
    class FileEchoSink final : public EchoSink
    {
    public:
        bool open(const OutputConfig &config) override;
        bool write_frame(const core::EchoFrame &frame) override;
        void close() override;
        const std::string &last_error() const override { return last_error_; }
        std::uint64_t frames_written() const override { return frames_written_; }

    private:
        bool write_metadata(const OutputConfig &config);

        OutputConfig config_;
        std::ofstream data_file_;
        std::string last_error_;
        std::uint64_t frames_written_ = 0;
        std::uint32_t samples_per_frame_ = 0;
        bool opened_ = false;
    };
} // namespace radar::output
