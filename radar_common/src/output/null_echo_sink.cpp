#include "output/null_echo_sink.h"

namespace radar::output
{
    bool NullEchoSink::open(const OutputConfig &)
    {
        frames_written_ = 0;
        opened_ = true;
        last_error_.clear();
        return true;
    }

    bool NullEchoSink::write_frame(const core::EchoFrame &)
    {
        if (!opened_)
        {
            last_error_ = "NullEchoSink is not open";
            return false;
        }
        ++frames_written_;
        return true;
    }

    void NullEchoSink::close()
    {
        opened_ = false;
    }
} // namespace radar::output
