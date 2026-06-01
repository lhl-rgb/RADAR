/**
 * @file udp_sender.h
 * @brief UDP 数据发送器 - 将仿真回波数据发送到信号处理机
 */

#pragma once

#include "core/radar_system_params.h"
#include "core/tools/udp_config.h"
#include "core/types.h"

#include <arpa/inet.h>
#include <cstdint>
#include <string>

namespace radar::core {

#pragma pack(push, 1)
struct UdpPacketHeader {
    uint32_t magic = 0x52414441;
    uint32_t version = 1;
    uint32_t sequence_id;
    uint32_t scan_index;
    uint32_t pulse_index;
    uint32_t beam_index;
    uint32_t pulse_count;
    uint32_t sample_count;
    int64_t timestamp_us;
    uint32_t checksum;
    uint32_t reserved = 0;
};
#pragma pack(pop)

constexpr size_t UDP_HEADER_SIZE = sizeof(UdpPacketHeader);

class UdpSender {
public:
    UdpSender() = default;
    explicit UdpSender(const UdpConfig& config);
    ~UdpSender();

    void set_config(const UdpConfig& config);
    const UdpConfig& config() const { return config_; }

    bool initialize();
    void shutdown();
    bool is_initialized() const { return initialized_; }
    const std::string& last_error() const { return last_error_; }
    uint32_t packets_sent() const { return sequence_counter_; }

private:
    bool send_packet(const uint8_t* data, size_t size);
    uint32_t compute_checksum(const uint8_t* data, size_t size) const;
    uint32_t compute_checksum(const uint8_t* data, size_t size, uint32_t initial_crc) const;

    UdpConfig config_;
    std::string last_error_;
    int socket_ = -1;
    struct sockaddr_in server_addr_{};
    uint32_t sequence_counter_ = 0;
    bool initialized_ = false;
};

}  // namespace radar::core
