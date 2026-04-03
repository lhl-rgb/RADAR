/**
 * @file configuration_manager.h
 * @brief 配置管理器 - 支持动态更新和观察者通知
 */

#pragma once

#include "radar_config.h"
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace radar {

/**
 * @brief 配置变更观察者接口
 */
class IConfigObserver {
public:
    virtual ~IConfigObserver() = default;

    /// 整体配置变更通知
    virtual void on_config_changed(const RadarConfig& new_config) = 0;

    /// 部分配置变更通知（可选实现）
    virtual void on_system_changed(const RadarSystemParams& system) {}
    virtual void on_waveform_changed(const waveform::WaveformConfig& waveform) {}
    virtual void on_antenna_changed(const antenna::AntennaConfig& antenna) {}
    virtual void on_beam_table_changed(const antenna::BeamTableConfig& beam_table) {}
    virtual void on_noise_changed(const noise::NoiseConfig& noise) {}
    virtual void on_clutter_changed(const clutter::SeaClutterConfig& clutter) {}
    virtual void on_target_changed(const target::TargetConfig& target) {}
};

/**
 * @brief 配置管理器
 * @details
 * - 加载/保存 JSON 配置文件
 * - 管理配置生命周期
 * - 支持部分更新和观察者通知
 */
class ConfigurationManager {
public:
    ConfigurationManager() = default;

    /// 从 JSON 文件加载配置
    bool load_from_json(const std::string& filepath);

    /// 保存配置到 JSON 文件
    bool save_to_json(const std::string& filepath) const;

    /// 获取当前配置（只读）
    const RadarConfig& config() const { return config_; }

    /// 获取可修改配置引用（修改后需调用 notify_all）
    RadarConfig& mutable_config() { return config_; }

    /// 部分更新 - 系统参数
    void update_system(RadarSystemParams new_system);

    /// 部分更新 - 波形配置
    void update_waveform(waveform::WaveformConfig new_waveform);

    /// 部分更新 - 天线配置
    void update_antenna(antenna::AntennaConfig new_antenna);

    /// 部分更新 - 噪声配置
    void update_noise(noise::NoiseConfig new_noise);

    /// 部分更新 - 杂波配置
    void update_clutter(clutter::SeaClutterConfig new_clutter);

    /// 部分更新 - 目标配置
    void update_target(target::TargetConfig new_target);

    /// 整体更新
    void update_config(RadarConfig new_config);

    /// 订阅特定配置类型的变更
    template<typename ConfigType>
    void subscribe(IConfigObserver* observer) {
        observers_[std::type_index(typeid(ConfigType))].push_back(observer);
    }

    /// 取消订阅
    template<typename ConfigType>
    void unsubscribe(IConfigObserver* observer) {
        auto& vec = observers_[std::type_index(typeid(ConfigType))];
        vec.erase(std::remove(vec.begin(), vec.end(), observer), vec.end());
    }

    /// 通知所有观察者
    void notify_all();

    /// 获取最近错误信息
    const std::string& last_error() const { return last_error_; }

private:
    /// 通知特定类型的观察者
    template<typename ConfigType>
    void notify_type(const ConfigType& cfg) {
        auto it = observers_.find(std::type_index(typeid(ConfigType)));
        if (it != observers_.end()) {
            for (auto* observer : it->second) {
                if constexpr (std::is_same_v<ConfigType, RadarSystemParams>) {
                    observer->on_system_changed(cfg);
                } else if constexpr (std::is_same_v<ConfigType, waveform::WaveformConfig>) {
                    observer->on_waveform_changed(cfg);
                } else if constexpr (std::is_same_v<ConfigType, antenna::AntennaConfig>) {
                    observer->on_antenna_changed(cfg);
                } else if constexpr (std::is_same_v<ConfigType, noise::NoiseConfig>) {
                    observer->on_noise_changed(cfg);
                } else if constexpr (std::is_same_v<ConfigType, clutter::SeaClutterConfig>) {
                    observer->on_clutter_changed(cfg);
                } else if constexpr (std::is_same_v<ConfigType, target::TargetConfig>) {
                    observer->on_target_changed(cfg);
                }
            }
        }
    }

    RadarConfig config_;
    std::unordered_map<std::type_index, std::vector<IConfigObserver*>> observers_;
    mutable std::string last_error_;
};

}  // namespace radar