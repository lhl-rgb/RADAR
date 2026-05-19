/**
 * @file udp_sender.h
 * @brief UDP 数据发送器 - 将仿真回波数据发送到信号处理机
 */

#pragma once

#include "core/tools/udp_config.h"
#include "core/types.h"
#include "core/radar_system_params.h"

#include <cstdint>
#include <string>
#include <arpa/inet.h>

namespace radar::core {

/**
 * @brief UDP 数据包头部结构
 * @details 网络字节序（大端），使用#pragma pack 避免内存对齐问题
 */
#pragma pack(push, 1)
struct UdpPacketHeader {
    uint32_t magic = 0x52414441;        ///< "RADAR" 魔数
    uint32_t version = 1;               ///< 协议版本
    uint32_t sequence_id;               ///< 包序号
    uint32_t scan_index;                ///< 扫描索引
    uint32_t cpi_index;                 ///< CPI 索引
    uint32_t beam_index;                ///< 波位索引
    uint32_t pulse_count;               ///< 本包脉冲数
    uint32_t sample_count;              ///< 每脉冲采样点数
    int64_t  timestamp_us;              ///< 时间戳（微秒）
    uint32_t checksum;                  ///< CRC32 头部校验和
    uint32_t reserved = 0;              ///< 保留字段
};
#pragma pack(pop)

/**
 * @brief UDP 头部大小常量
 */
constexpr size_t UDP_HEADER_SIZE = sizeof(UdpPacketHeader);

/**
 * @brief UDP 数据发送器
 * @details 将 CPI 回波数据通过 UDP 发送到信号处理机
 */
class UdpSender {
public:
    /**
     * @brief 默认构造函数
     */
    UdpSender() = default;

    /**
     * @brief 使用配置构造
     * @param config UDP 配置参数
     */
    explicit UdpSender(const UdpConfig& config);

    /**
     * @brief 析构函数（自动关闭 socket）
     */
    ~UdpSender();

    /**
     * @brief 设置配置参数
     * @param config UDP 配置参数
     */
    void set_config(const UdpConfig& config);

    /**
     * @brief 获取当前配置
     */
    const UdpConfig& config() const { return config_; }

    /**
     * @brief 初始化 UDP 发送器（创建 socket）
     * @return 成功返回 true
     */
    bool initialize();

    /**
     * @brief 关闭 UDP 发送器（关闭 socket）
     */
    void shutdown();

    /**
     * @brief 发送一个 CPI 的回波数据
     * @param cpi_echo CPI 回波数据
     * @param scan_index 扫描索引
     * @param cpi_index CPI 索引
     * @param system 雷达系统参数
     * @return 成功返回 true
     */
    bool send_cpi(const CpiEcho& cpi_echo,
                  int scan_index,
                  int cpi_index,
                  const RadarSystemParams& system);

    /**
     * @brief 检查是否已初始化
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief 获取最近一次错误信息
     */
    const std::string& last_error() const { return last_error_; }

    /**
     * @brief 获取已发送包数量统计
     */
    uint32_t packets_sent() const { return sequence_counter_; }

private:
    /**
     * @brief 发送原始数据包
     * @param data 数据缓冲区
     * @param size 数据大小
     * @return 成功返回 true
     */
    bool send_packet(const uint8_t* data, size_t size);

    /**
     * @brief 计算 CRC32 校验和
     * @param data 数据缓冲区
     * @param size 数据大小
     * @return CRC32 值
     */
    uint32_t compute_checksum(const uint8_t* data, size_t size) const;

    /**
     * @brief 计算 CRC32 校验和（带初始值）
     * @param data 数据缓冲区
     * @param size 数据大小
     * @param initial_crc 初始 CRC 值
     * @return CRC32 值
     */
    uint32_t compute_checksum(const uint8_t* data, size_t size, uint32_t initial_crc) const;

    UdpConfig config_;               ///< UDP 配置
    std::string last_error_;         ///< 最近错误信息
    int socket_ = -1;                ///< Socket 文件描述符
    struct sockaddr_in server_addr_{}; ///< 目标地址结构
    uint32_t sequence_counter_ = 0;  ///< 序列号计数器
    bool initialized_ = false;       ///< 是否已初始化
};

}  // namespace radar::core
