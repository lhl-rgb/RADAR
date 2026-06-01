#pragma once

#include "output/echo_sink.h"

namespace radar::output
{
    class NullEchoSink final : public EchoSink
    {
    public:
        bool open(const OutputConfig &config) override;
        bool write_frame(const core::EchoFrame &frame) override;
        void close() override;
        const std::string &last_error() const override { return last_error_; }
        std::uint64_t frames_written() const override { return frames_written_; }

    private:
        std::string last_error_;
        std::uint64_t frames_written_ = 0;
        bool opened_ = false;
    };
} // namespace radar::output
