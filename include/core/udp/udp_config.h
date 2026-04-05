/**
 * @file udp_config.h
 * @brief UDP 输出配置
 */

#pragma once

#include <string>
#include <nlohmann/json.hpp>

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

// JSON 序列化
inline void from_json(const nlohmann::json& j, UdpConfig& cfg) {
    if (j.contains("enabled")) j.at("enabled").get_to(cfg.enabled);
    if (j.contains("target_ip")) j.at("target_ip").get_to(cfg.target_ip);
    if (j.contains("target_port")) j.at("target_port").get_to(cfg.target_port);
}

inline void to_json(nlohmann::json& j, const UdpConfig& cfg) {
    j = nlohmann::json{
        {"enabled", cfg.enabled},
        {"target_ip", cfg.target_ip},
        {"target_port", cfg.target_port}
    };
}

}  // namespace radar
