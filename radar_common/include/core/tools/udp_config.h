/**
 * @file udp_config.h
 * @brief UDP 输出配置
 */

#pragma once

#include <string>

namespace radar {

/**
 * @brief UDP 输出配置
 * @details 用于配置仿真数据通过 UDP 发送到信号处理机
 */
struct UdpConfig {
    bool enabled = false;           ///< 是否启用 UDP 输出
    std::string target_ip = "127.0.0.1";  ///< 目标 IP 地址
    int target_port = 9000;         ///< 目标端口号

    /**
     * @brief 验证配置参数
     * @param error 错误信息输出
     * @return 有效返回 true
     */
    bool validate(std::string& error) const;
};

inline bool UdpConfig::validate(std::string& error) const {
    if (target_port <= 0 || target_port > 65535) {
        error = "target_port must be in range [1, 65535]";
        return false;
    }
    if (target_ip.empty()) {
        error = "target_ip cannot be empty";
        return false;
    }
    return true;
}

}  // namespace radar
