#pragma once

#include "core/echo_frame.h"
#include "output/output_config.h"

#include <cstdint>
#include <memory>
#include <string>

namespace radar::output
{
    class EchoSink
    {
    public:
        virtual ~EchoSink() = default;
        virtual bool open(const OutputConfig &config) = 0;
        virtual bool write_frame(const core::EchoFrame &frame) = 0;
        virtual void close() = 0;
        virtual const std::string &last_error() const = 0;
        virtual std::uint64_t frames_written() const = 0;
    };

    std::unique_ptr<EchoSink> create_echo_sink(const OutputConfig &config);
} // namespace radar::output
