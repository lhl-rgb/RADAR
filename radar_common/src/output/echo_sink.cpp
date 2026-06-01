#include "output/echo_sink.h"

#include "output/file_echo_sink.h"
#include "output/null_echo_sink.h"
#include "output/udp_echo_sink.h"

namespace radar::output
{
    std::unique_ptr<EchoSink> create_echo_sink(const OutputConfig &config)
    {
        if (!config.enabled || config.sink_type == OutputSinkType::Null)
        {
            return std::make_unique<NullEchoSink>();
        }

        if (config.sink_type == OutputSinkType::File)
        {
            return std::make_unique<FileEchoSink>();
        }

        if (config.sink_type == OutputSinkType::Udp)
        {
            return std::make_unique<UdpEchoSink>();
        }

        return std::make_unique<NullEchoSink>();
    }
} // namespace radar::output
