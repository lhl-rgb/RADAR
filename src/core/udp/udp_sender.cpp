/**
 * @file udp_sender.cpp
 * @brief UDP 数据发送器实现
 */

#include "core/udp/udp_sender.h"

#include <arpa/inet.h>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>

namespace radar::core {

// Using declarations for types from parent namespace
using radar::CpiEcho;
using radar::RadarSystemParams;
using radar::ScalarVector;

// ============================================================================
// CRC32 查找表（预计算）
// ============================================================================

namespace {

/**
 * @brief CRC32 查找表（IEEE 802.3 标准）
 */
uint32_t crc32_table[256];
bool crc32_table_initialized = false;

/**
 * @brief 初始化 CRC32 查找表
 */
void init_crc32_table() {
    if (crc32_table_initialized) return;

    const uint32_t polynomial = 0xEDB88320;
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t crc = i;
        for (int j = 0; j < 8; ++j) {
            crc = (crc >> 1) ^ ((crc & 1) ? polynomial : 0);
        }
        crc32_table[i] = crc;
    }
    crc32_table_initialized = true;
}

}  // namespace

// ============================================================================
// UdpSender 实现
// ============================================================================

UdpSender::UdpSender(const UdpConfig& config) : config_(config) {
    init_crc32_table();
}

UdpSender::~UdpSender() {
    shutdown();
}

void UdpSender::set_config(const UdpConfig& config) {
    config_ = config;
}

bool UdpSender::initialize() {
    if (initialized_) {
        return true;
    }

    // 验证配置
    std::string error;
    if (!config_.validate(error)) {
        last_error_ = "Invalid UDP config: " + error;
        return false;
    }

    // 创建 UDP socket
    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ < 0) {
        last_error_ = "Failed to create socket: " + std::string(strerror(errno));
        return false;
    }

    // 设置目标地址
    std::memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(static_cast<uint16_t>(config_.target_port));

    // 解析 IP 地址
    if (inet_pton(AF_INET, config_.target_ip.c_str(), &server_addr_.sin_addr) <= 0) {
        last_error_ = "Invalid target IP: " + config_.target_ip;
        ::close(socket_);
        socket_ = -1;
        return false;
    }

    initialized_ = true;
    sequence_counter_ = 0;
    return true;
}

void UdpSender::shutdown() {
    if (socket_ >= 0) {
        ::close(socket_);
        socket_ = -1;
    }
    initialized_ = false;
}

bool UdpSender::send_cpi(const CpiEcho& cpi_echo,
                         int scan_index,
                         int cpi_index,
                         const RadarSystemParams& system) {
    if (!initialized_) {
        last_error_ = "UdpSender not initialized";
        return false;
    }

    if (cpi_echo.pulses.empty()) {
        last_error_ = "Empty CPI echo";
        return false;
    }

    uint32_t pulse_count = static_cast<uint32_t>(cpi_echo.pulses.size());
    uint32_t sample_count = static_cast<uint32_t>(cpi_echo.pulses.empty() ? 0 : cpi_echo.pulses[0].size());
    // 准备头部
    UdpPacketHeader header;
    header.sequence_id = htonl(sequence_counter_);
    header.scan_index = htonl(static_cast<uint32_t>(scan_index));
    header.cpi_index = htonl(static_cast<uint32_t>(cpi_index));
    header.beam_index = htonl(static_cast<uint32_t>(cpi_echo.beam_index));
    header.pulse_count = htonl(pulse_count);
    header.sample_count = htonl(sample_count);

    // 计算时间戳 (跳过实际时间获取，使用 0 或者模拟时间戳)
    header.timestamp_us = 0;

    header.checksum = 0;  // 先置 0

    // 序列化 IQ 数据（float32 实部 + 虚部交替）
    ScalarVector iq_data;
    const size_t total_samples = pulse_count * sample_count;
    iq_data.reserve(total_samples * sizeof(Scalar)); 

    for (const auto& pulse : cpi_echo.pulses) {
        for (const auto& sample : pulse) {
            iq_data.push_back(static_cast<Scalar>(sample.real()));
            iq_data.push_back(static_cast<Scalar>(sample.imag()));
        }
    }

    // 计算头部校验和
    const size_t header_size = sizeof(UdpPacketHeader);
    std::vector<uint8_t> packet_buffer(header_size + iq_data.size() * sizeof(float));

    // 计算头部校验和
    uint32_t checksum = compute_checksum(packet_buffer.data(), header_size);

    // 填入校验和（主机序转网络序）
    header.checksum = htonl(checksum);
    std::memcpy(packet_buffer.data(), &header, header_size);

    // 复制 IQ 数据
    if (!iq_data.empty()) {
        std::memcpy(packet_buffer.data() + header_size, iq_data.data(),
                    iq_data.size() * sizeof(float));
    }

    // 发送数据包
    if (!send_packet(packet_buffer.data(), packet_buffer.size())) {
        return false;
    }

    sequence_counter_++;
    return true;
}

bool UdpSender::send_packet(const uint8_t* data, size_t size) {
    ssize_t sent = ::sendto(socket_, data, size, 0,
                            reinterpret_cast<const sockaddr*>(&server_addr_),
                            sizeof(server_addr_));

    if (sent < 0) {
        last_error_ = "Failed to send packet: " + std::string(strerror(errno));
        return false;
    }

    if (static_cast<size_t>(sent) != size) {
        last_error_ = "Incomplete send: sent " + std::to_string(sent) +
                      " / " + std::to_string(size) + " bytes";
        return false;
    }

    return true;
}

uint32_t UdpSender::compute_checksum(const uint8_t* data, size_t size, uint32_t initial_crc) const {
    uint32_t crc = initial_crc ^ 0xFFFFFFFF;

    for (size_t i = 0; i < size; ++i) {
        crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
    }

    return crc ^ 0xFFFFFFFF;
}

uint32_t UdpSender::compute_checksum(const uint8_t* data, size_t size) const {
    return compute_checksum(data, size, 0);
}

}  // namespace radar::core
