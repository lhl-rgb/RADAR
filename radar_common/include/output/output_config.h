#pragma once

#include "core/tools/udp_config.h"

#include <string>

namespace radar::output
{
    enum class OutputSinkType
    {
        Null = 0,
        File = 1,
        Udp = 2
    };

    struct OutputConfig
    {
        bool enabled = false;
        OutputSinkType sink_type = OutputSinkType::Null;
        std::string output_dir = "output";
        std::string file_prefix = "echo_stream";
        radar::UdpConfig udp;

        bool validate(std::string &error) const
        {
            if (!enabled)
            {
                return true;
            }
            if (sink_type == OutputSinkType::File && output_dir.empty())
            {
                error = "output.output_dir cannot be empty for File sink";
                return false;
            }
            if (sink_type == OutputSinkType::Udp && !udp.validate(error))
            {
                error = "output UDP configuration invalid: " + error;
                return false;
            }
            return true;
        }
    };
} // namespace radar::output
