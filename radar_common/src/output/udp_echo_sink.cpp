#include "output/udp_echo_sink.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <limits>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <vector>

namespace radar::output
{
    namespace
    {
        constexpr std::size_t kMaxDatagramBytes = 60000U;
        constexpr std::size_t kBytesPerComplexFloat = sizeof(float) * 2U;
        constexpr std::uint32_t kFnvOffset = 2166136261U;
        constexpr std::uint32_t kFnvPrime = 16777619U;
    } // namespace

    UdpEchoSink::~UdpEchoSink()
    {
        close();
    }

    bool UdpEchoSink::open(const OutputConfig &config)
    {
        close();
        config_ = config;
        last_error_.clear();
        sequence_id_ = 0U;
        frames_written_ = 0U;

        std::string error;
        if (!config_.udp.validate(error))
        {
            last_error_ = "Invalid UDP config: " + error;
            return false;
        }

        socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (socket_ < 0)
        {
            last_error_ = "Failed to create UDP socket: " + std::string(std::strerror(errno));
            return false;
        }

        target_addr_ = {};
        target_addr_.sin_family = AF_INET;
        target_addr_.sin_port = htons(static_cast<std::uint16_t>(config_.udp.target_port));
        if (::inet_pton(AF_INET, config_.udp.target_ip.c_str(), &target_addr_.sin_addr) <= 0)
        {
            last_error_ = "Invalid target IP: " + config_.udp.target_ip;
            close();
            return false;
        }

        opened_ = true;
        return true;
    }

    bool UdpEchoSink::write_frame(const core::EchoFrame &frame)
    {
        if (!opened_ || socket_ < 0)
        {
            last_error_ = "UdpEchoSink is not open";
            return false;
        }
        if (frame.iq.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
        {
            last_error_ = "EchoFrame has too many samples for UDP header";
            return false;
        }

        const std::size_t payload_capacity = kMaxDatagramBytes - sizeof(UdpEchoPacketHeader);
        const std::size_t samples_per_segment = payload_capacity / kBytesPerComplexFloat;
        if (samples_per_segment == 0U)
        {
            last_error_ = "UDP payload capacity is too small";
            return false;
        }

        const std::size_t total_samples = frame.iq.size();
        const std::size_t segment_count =
            std::max<std::size_t>(1U, (total_samples + samples_per_segment - 1U) / samples_per_segment);
        if (segment_count > static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max()))
        {
            last_error_ = "EchoFrame requires too many UDP segments";
            return false;
        }

        for (std::size_t segment = 0U; segment < segment_count; ++segment)
        {
            const std::size_t sample_offset = segment * samples_per_segment;
            const std::size_t sample_count =
                std::min(samples_per_segment, total_samples - sample_offset);
            const std::size_t payload_bytes = sample_count * kBytesPerComplexFloat;
            std::vector<std::uint8_t> packet(sizeof(UdpEchoPacketHeader) + payload_bytes, 0U);

            UdpEchoPacketHeader header;
            header.sequence_id = sequence_id_++;
            header.pulse_index = frame.pulse_index;
            header.scan_index = frame.scan_index;
            header.segment_index = static_cast<std::uint16_t>(segment);
            header.segment_count = static_cast<std::uint16_t>(segment_count);
            header.sample_offset = static_cast<std::uint32_t>(sample_offset);
            header.sample_count = static_cast<std::uint32_t>(sample_count);
            header.timestamp_s = static_cast<float>(frame.timestamp_s);
            header.beam_az_deg = static_cast<float>(frame.beam_az_deg);
            header.beam_el_deg = static_cast<float>(frame.beam_el_deg);
            header.range_bin_size_m = static_cast<float>(frame.range_bin_size_m);
            header.payload_bytes = static_cast<std::uint32_t>(payload_bytes);

            std::memcpy(packet.data(), &header, sizeof(header));
            std::size_t payload_pos = sizeof(UdpEchoPacketHeader);
            for (std::size_t i = 0U; i < sample_count; ++i)
            {
                const auto &sample = frame.iq[sample_offset + i];
                const float real = static_cast<float>(sample.real());
                const float imag = static_cast<float>(sample.imag());
                std::memcpy(packet.data() + payload_pos, &real, sizeof(real));
                payload_pos += sizeof(real);
                std::memcpy(packet.data() + payload_pos, &imag, sizeof(imag));
                payload_pos += sizeof(imag);
            }

            header.checksum = compute_checksum(packet.data(), packet.size());
            std::memcpy(packet.data(), &header, sizeof(header));

            const ssize_t sent = ::sendto(socket_,
                                          packet.data(),
                                          packet.size(),
                                          0,
                                          reinterpret_cast<const sockaddr *>(&target_addr_),
                                          sizeof(target_addr_));
            if (sent < 0)
            {
                last_error_ = "Failed to send UDP EchoFrame: " + std::string(std::strerror(errno));
                return false;
            }
            if (static_cast<std::size_t>(sent) != packet.size())
            {
                last_error_ = "Incomplete UDP send";
                return false;
            }
        }

        ++frames_written_;
        return true;
    }

    void UdpEchoSink::close()
    {
        if (socket_ >= 0)
        {
            ::close(socket_);
            socket_ = -1;
        }
        opened_ = false;
    }

    std::uint32_t UdpEchoSink::compute_checksum(const std::uint8_t *data, std::size_t size) const
    {
        std::uint32_t hash = kFnvOffset;
        for (std::size_t i = 0U; i < size; ++i)
        {
            hash ^= data[i];
            hash *= kFnvPrime;
        }
        return hash;
    }
} // namespace radar::output
