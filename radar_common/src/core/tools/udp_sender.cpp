/**
 * @file udp_sender.cpp
 * @brief UDP 数据发送器实现
 */

#include "core/tools/udp_sender.h"

#include <arpa/inet.h>
#include <cstring>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>

namespace radar::core {

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
