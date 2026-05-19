/**
 * @file data_exporter.h
 * @brief 仿真数据导出器
 * @details
 * 负责将每圈（Scan）的仿真数据导出到文件，供 MATLAB 验证脚本使用。
 * 导出内容包括：
 * - 目标配置参数（设定值）
 * - 每 CPI 的回波数据
 * - 目标轨迹快照
 */

#pragma once

#include "core/types.h"
#include "core/radar_system_params.h"
#include "target/target_config.h"
#include "target/target_kinematics.h"

#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>

namespace radar::core {

/**
 * @brief 数据导出配置
 */
struct ExportConfig {
    bool enabled = true;                        ///< 是否启用数据导出
    std::string output_dir = "output";          ///< 输出目录
    bool export_raw_echo_iq = true;             ///< 是否导出原始 IQ 数据 (dat 二进制格式)
    bool export_target_snapshots = false;        ///< 是否导出目标快照
    bool export_config_params = true;           ///< 是否导出配置参数
};

// JSON 序列化
inline void from_json(const nlohmann::json& j, ExportConfig& cfg) {
    if (j.contains("enabled")) j.at("enabled").get_to(cfg.enabled);
    if (j.contains("output_dir")) j.at("output_dir").get_to(cfg.output_dir);
    if (j.contains("export_raw_echo_iq")) j.at("export_raw_echo_iq").get_to(cfg.export_raw_echo_iq);
    if (j.contains("export_target_snapshots")) j.at("export_target_snapshots").get_to(cfg.export_target_snapshots);
    if (j.contains("export_config_params")) j.at("export_config_params").get_to(cfg.export_config_params);
}

inline void to_json(nlohmann::json& j, const ExportConfig& cfg) {
    j = nlohmann::json{
        {"enabled", cfg.enabled},
        {"output_dir", cfg.output_dir},
        {"export_raw_echo_iq", cfg.export_raw_echo_iq},
        {"export_target_snapshots", cfg.export_target_snapshots},
        {"export_config_params", cfg.export_config_params}
    };
}

/**
 * @brief 单圈扫描数据
 */
struct ScanData {
    int scan_index = 0;                                     ///< 扫描编号
    std::vector<CpiEcho> cpi_echos;                         ///< CPI 回波数据
    std::vector<TargetList> target_snapshots_per_cpi;       ///< 每个 CPI 的目标快照
    RadarSystemParams system_params;                        ///< 系统参数快照
    target::TargetConfig target_config;                     ///< 目标配置快照
    TargetList initial_targets;                             ///< 初始目标配置
};

/**
 * @brief 数据导出器
 */
class DataExporter {
public:
    DataExporter() = default;
    explicit DataExporter(const ExportConfig& config);

    /**
     * @brief 设置导出配置
     */
    void set_config(const ExportConfig& config);

    /**
     * @brief 初始化（创建输出目录）
     */
    bool initialize();

    /**
     * @brief 导出一圈的数据
     * @param scan_data 扫描数据
     * @return 成功返回 true
     */
    bool export_scan(const ScanData& scan_data);

    /**
     * @brief 导出配置参数到 JSON 文件
     */
    bool export_config_params(const RadarSystemParams& system,
                               const target::TargetConfig& target_config,
                               const TargetList& initial_targets);

    /**
     * @brief 导出 IQ 数据到 DAT 文件（二进制格式）
     */
    bool export_echo_iq_dat(const std::vector<CpiEcho>& cpi_echos,
                             int scan_index);


    bool export_complex_csv(const std::string& filepath,
                        const ComplexVec& data,
                        const std::string& var_name = "signal");

    bool export_scalar_csv(const std::string& filepath,
                       const std::vector<Scalar>& data,
                       const std::string& var_name = "data");                    
    /**
     * @brief 获取最后错误信息
     */
    const std::string& last_error() const { return last_error_; }

private:
    ExportConfig config_;
    std::string last_error_;

    /**
     * @brief 确保目录存在
     */
    bool ensure_directory_exists(const std::string& path);

    /**
     * @brief 写入 CSV 文件
     */
    bool write_csv_file(const std::string& filepath, const std::vector<std::string>& headers,
                        const std::vector<std::vector<std::string>>& rows);
};

}  // namespace radar::core
