#pragma once

#include "output/echo_sink.h"

#include <cstddef>
#include <cstdint>
#include <netinet/in.h>
#include <string>

namespace radar::output
{
#pragma pack(push, 1)
    struct UdpEchoPacketHeader
    {
        std::uint32_t magic = 0x45434855U; // ECHU
        std::uint16_t version = 1U;
        std::uint16_t header_size = sizeof(UdpEchoPacketHeader);
        std::uint64_t sequence_id = 0U;
        std::uint64_t pulse_index = 0U;
        std::uint32_t scan_index = 0U;
        std::uint16_t segment_index = 0U;
        std::uint16_t segment_count = 0U;
        std::uint32_t sample_offset = 0U;
        std::uint32_t sample_count = 0U;
        float timestamp_s = 0.0f;
        float beam_az_deg = 0.0f;
        float beam_el_deg = 0.0f;
        float range_bin_size_m = 0.0f;
        std::uint32_t payload_bytes = 0U;
        std::uint32_t checksum = 0U;
    };
#pragma pack(pop)

    class UdpEchoSink final : public EchoSink
    {
    public:
        UdpEchoSink() = default;
        ~UdpEchoSink() override;

        bool open(const OutputConfig &config) override;
        bool write_frame(const core::EchoFrame &frame) override;
        void close() override;
        const std::string &last_error() const override { return last_error_; }
        std::uint64_t frames_written() const override { return frames_written_; }

    private:
        std::uint32_t compute_checksum(const std::uint8_t *data, std::size_t size) const;

        OutputConfig config_;
        std::string last_error_;
        int socket_ = -1;
        sockaddr_in target_addr_{};
        std::uint64_t sequence_id_ = 0U;
        std::uint64_t frames_written_ = 0U;
        bool opened_ = false;
    };
} // namespace radar::output
