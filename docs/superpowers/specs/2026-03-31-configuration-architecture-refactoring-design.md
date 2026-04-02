---
name: Configuration Architecture Refactoring
description: Refactor configuration system: unified types, ConfigurationManager with observer pattern, directory reorganization, eliminate legacy RadarParams
type: project
---

# Configuration Architecture Refactoring Design

## Summary

Refactor the radar simulation configuration system to:
- Unify configuration types (eliminate dual Config/Params types)
- Introduce ConfigurationManager with observer pattern for dynamic updates
- Reorganize directory structure for better modularity
- Support JSON file loading with runtime parsing
- Enable partial configuration updates with targeted notifications

## Motivation

Current issues:
1. **Dual configuration types**: `SeaClutterConfig` vs `SeaClutterParams`, `TargetConfig` vs `TargetParams`, etc.
2. **RadarParams monolith**: Contains all parameters in one struct, causing tight coupling
3. **Manual conversion code**: ~70 lines of field-by-field assignment in main.cpp
4. **Raw pointers in RadarConfig**: No lifecycle management
5. **No update mechanism**: Engines cannot respond to configuration changes

## Architecture

### Core Components

```
ConfigurationManager (owner)
    │
    ├── RadarConfig (pure data container)
    │       ├── RadarSystemParams (shared global params)
    │       ├── waveform::WaveformConfig
    │       ├── antenna::AntennaConfig
    │       ├── noise::NoiseConfig
    │       ├── clutter::SeaClutterConfig
    │       └── target::TargetConfig
    │
    └── Observer registry
            │
            └── IConfigObserver interface
                    ├── WaveformEngine
                    ├── NoiseEngine
                    ├── ClutterEngine
                    └── TargetEngine
```

### ConfigurationManager

**Why**: Separates configuration management (loading, updating, notifying) from configuration data (RadarConfig).

**How to apply**:
- Singleton or passed to Engines at construction
- Handles JSON file I/O
- Manages observer subscriptions per config type
- Notifies only relevant observers on partial updates

```cpp
class ConfigurationManager {
public:
    // File I/O
    bool load_from_json(const std::string& filepath);
    bool save_to_json(const std::string& filepath) const;

    // Access
    const RadarConfig& config() const;

    // Partial updates (notify relevant observers only)
    void update_system(RadarSystemParams new_system);
    void update_waveform(waveform::WaveformConfig new_waveform);
    void update_noise(noise::NoiseConfig new_noise);
    void update_clutter(clutter::SeaClutterConfig new_clutter);
    void update_target(target::TargetConfig new_target);
    void update_antenna(antenna::AntennaConfig new_antenna);

    // Full update (notify all observers)
    void update_config(RadarConfig new_config);

    // Subscription
    template<typename ConfigType>
    void subscribe(IConfigObserver* observer);
    void unsubscribe(IConfigObserver* observer);

private:
    RadarConfig config_;
    std::unordered_map<std::type_index, std::vector<IConfigObserver*>> observers_;
};
```

### IConfigObserver Interface

**Why**: Enables Engines to react to configuration changes without polling.

**How to apply**:
- Engines implement this interface
- Register with ConfigurationManager for specific config types
- Apply new configuration in callback methods

```cpp
class IConfigObserver {
public:
    virtual ~IConfigObserver() = default;

    // Full config change
    virtual void on_config_changed(const RadarConfig& new_config) = 0;

    // Partial config changes (optional, override as needed)
    virtual void on_system_changed(const RadarSystemParams& system) {}
    virtual void on_waveform_changed(const waveform::WaveformConfig& waveform) {}
    virtual void on_antenna_changed(const antenna::AntennaConfig& antenna) {}
    virtual void on_noise_changed(const noise::NoiseConfig& noise) {}
    virtual void on_clutter_changed(const clutter::SeaClutterConfig& clutter) {}
    virtual void on_target_changed(const target::TargetConfig& target) {}
};
```

### RadarConfig (Pure Data)

**Why**: Simple value-semantics container, no behavior, easy to serialize.

**How to apply**:
- All fields are values (no pointers)
- Passed by const reference to Engines
- Serialization via JSON

```cpp
struct RadarConfig {
    RadarSystemParams system;
    waveform::WaveformConfig waveform;
    antenna::AntennaConfig antenna;
    noise::NoiseConfig noise;
    clutter::SeaClutterConfig clutter;
    target::TargetConfig target;

    void compute_derived_params();  // delegates to system
    bool validate(std::string& error) const;
    bool validate_full(std::string& error) const;
    void print() const;
};
```

## Directory Structure

### Before → After

```
include/core/
├── antenna_set.h          → include/antenna/phased_array_antenna.h
│                          → include/antenna/antenna_scan_model.h
├── waveform_config.h      → include/waveform/waveform_config.h
├── waveform_generator.h   → include/waveform/waveform_generator.h
├── radar_params.h         → DELETE
├── types.h                → KEEP
├── radar_system_params.h  → KEEP
├── radar_config.h         → KEEP
├── math_utils.h           → KEEP
└── (new)                  → configuration_manager.h

src/core/
├── antenna_set.cpp        → src/antenna/phased_array_antenna.cpp
│                          → src/antenna/antenna_scan_model.cpp
├── waveform_config.cpp    → src/waveform/waveform_config.cpp
├── waveform_generator.cpp → src/waveform/waveform_generator.cpp
├── radar_params.cpp       → DELETE
├── radar_system_params.cpp → KEEP
├── radar_config.cpp       → KEEP
```

### Final Structure

```
include/
├── core/
│   ├── types.h
│   ├── radar_system_params.h
│   ├── radar_config.h
│   ├── configuration_manager.h  (NEW)
│   └── math_utils.h
│
├── antenna/
│   ├── antenna_config.h
│   ├── phased_array_antenna.h   (from antenna_set.h)
│   └── antenna_scan_model.h     (from antenna_set.h)
│
├── waveform/
│   ├── waveform_config.h        (from core)
│   └── waveform_generator.h     (from core)
│
├── noise/
│   ├── noise_config.h
│   └── noise_engine.h
│
├── clutter/
│   ├── sea_clutter_config.h
│   ├── clutter_engine.h
│   └── sea_clutter_model.h
│
├── target/
│   ├── target_config.hpp
│   ├── target_engine.h
│   └── target_manager.h
│

src/
├── core/
│   ├── radar_system_params.cpp
│   ├── radar_config.cpp
│   ├── configuration_manager.cpp  (NEW)
│   └── math_utils.cpp
│
├── antenna/
│   ├── phased_array_antenna.cpp   (from antenna_set.cpp)
│   ├── antenna_scan_model.cpp     (from antenna_set.cpp)
│
├── waveform/
│   ├── waveform_config.cpp        (from core)
│   ├── waveform_generator.cpp     (from core)
│
├── noise/
│   ├── noise_config.cpp
│   ├── noise_engine.cpp
│
├── clutter/
│   ├── sea_clutter_config.cpp
│   ├── clutter_engine.cpp
│   ├── sea_clutter_model.cpp
│
├── target/
│   ├── target_config.cpp
│   ├── target_engine.cpp
│   └── target_manager.cpp
│
└── app/
    └── main.cpp
```

## Type Unification

### Types to Delete

| Old Type | Location | Replacement |
|----------|----------|-------------|
| `RadarParams` | radar_params.h | `RadarConfig` + `RadarSystemParams` |
| `SeaClutterParams` | radar_params.h | `clutter::SeaClutterConfig` |
| `TargetParams` | radar_params.h | `target::TargetConfig` |
| `PhasedArrayAntennaConfig` | radar_params.h | `antenna::AntennaConfig` |
| `NoiseParams` | radar_params.h | `noise::NoiseConfig` |
| `MorchinParams` | radar_params.h | `clutter::MorchinConfig` |
| `RadarParamManager` | radar_params.h | `ConfigurationManager` |

### Engine Interface Changes

**WaveformGenerator**:
```cpp
// Before
WaveformGenerator(const RadarParams& radar);

// After
WaveformGenerator(const RadarSystemParams& system,
                  const waveform::WaveformConfig& config);
```

**NoiseEngine**:
```cpp
// Before
NoiseEngine(const RadarParams& radar);

// After
NoiseEngine(const RadarSystemParams& system,
            const noise::NoiseConfig& config);
```

**ClutterEngine**:
```cpp
// Before
bool generate_sea_clutter_cpi(const RadarParams& radar_params,
                              const PhasedArrayAntenna& antenna,
                              const AzEl& beam_pointing,
                              int beam_index,
                              const ComplexVec& tx_waveform,
                              CpiEcho& out_clutter);

// After
bool generate_sea_clutter_cpi(const RadarSystemParams& system,
                              const PhasedArrayAntenna& antenna,
                              const AzEl& beam_pointing,
                              int beam_index,
                              const ComplexVec& tx_waveform,
                              CpiEcho& out_clutter);
```

**TargetEngine**:
```cpp
// Before
bool generate(const TargetList& active_targets,
              const BeamView& beam,
              const RadarParams& radar,
              const ComplexVec& tx_waveform,
              CpiEcho& out_echo);

// After
bool generate(const TargetList& active_targets,
              const BeamView& beam,
              const RadarSystemParams& system,
              const target::TargetConfig& config,
              const ComplexVec& tx_waveform,
              CpiEcho& out_echo);
```

## JSON Configuration

### File Structure

```json
{
  "system": {
    "fc_hz": 10000000000.0,
    "prf_hz": 1600.0,
    "fs_hz": 40000000.0,
    "bw_hz": 20000000.0,
    "pulse_width_s": 2e-05,
    "peak_power_w": 5000.0,
    "pulses_per_cpi": 32,
    "min_range_m": 1000.0,
    "max_range_m": 120000.0,
    "antenna_height_m": 15.0,
    "noise_figure_db": 4.0,
    "system_loss_db": 6.0
  },
  "waveform": {
    "type": "LFM",
    "window_type": "Hamming"
  },
  "antenna": {
    "model_type": "UPA_2D",
    "num_elements_az": 16,
    "num_elements_el": 12,
    "spacing_az_lambda": 0.5,
    "spacing_el_lambda": 0.5,
    "weight_type_az": "Uniform",
    "weight_type_el": "Uniform",
    "peak_gain_db": 35.0
  },
  "noise": {
    "mode": "ComplexSigma",
    "sigma_complex": 0.001,
    "seed": 12345
  },
  "clutter": {
    "enabled": true,
    "ground_range_min_m": -1.0,
    "ground_range_max_m": -1.0,
    "range_step_m": 100.0,
    "beam_az_width_deg": 3.0,
    "az_step_deg": 0.1,
    "k_shape_nu": 0.8,
    "doppler_center_hz": 0.0,
    "doppler_sigma_hz": 20.0,
    "sequence_mode": "InTimeMode",
    "seed": 2026,
    "pool_length_factor": 32,
    "morchin": {
      "a0_db": -40.0,
      "a_g": 10.0,
      "a_f": 0.0,
      "a_s": 1.0,
      "sea_state": 3.0,
      "sin_psi_floor": 0.0001
    }
  },
  "target": {
    "enabled": true,
    "enable_beam_gain": true,
    "enable_two_way_propagation_loss": true,
    "enable_phase": true,
    "enable_swerling": true,
    "seed": 20260330,
    "beam_gate_threshold_db": -20.0,
    "skip_out_of_beam_targets": false
  }
}
```

### Serialization

Each config struct needs `from_json` / `to_json` functions using nlohmann/json library:

```cpp
// Example: noise_config.h
namespace radar::noise {

void to_json(nlohmann::json& j, const NoiseConfig& cfg);
void from_json(const nlohmann::json& j, NoiseConfig& cfg);

}  // namespace radar::noise
```

Enum types need string conversion for JSON:

```cpp
// Example
NLOHMANN_JSON_SERIALIZE_ENUM(NoiseLevelMode, {
    {NoiseLevelMode::ComplexSigma, "ComplexSigma"},
    {NoiseLevelMode::NoisePower, "NoisePower"},
    {NoiseLevelMode::ThermalKTB, "ThermalKTB"}
})
```

## Main.cpp Usage Example

```cpp
int main() {
    // 1. Load configuration
    ConfigurationManager config_manager;
    if (!config_manager.load_from_json("config/radar_config.json")) {
        std::cerr << "Failed to load config\n";
        return 1;
    }

    const RadarConfig& config = config_manager.config();
    config.compute_derived_params();

    // 2. Initialize engines (auto-subscribe to config changes)
    WaveformEngine waveform_gen(config_manager);
    NoiseEngine noise_engine(config_manager);
    ClutterEngine clutter_engine(config_manager);
    TargetEngine target_engine(config_manager);

    // 3. Initialize antenna
    PhasedArrayAntenna antenna(config.antenna);
    AntennaScanModel scan_model;
    scan_model.set_antenna(antenna);
    scan_model.set_beam_table(/* ... */);

    // 4. Simulation loop (no manual conversion needed)
    for (int cpi = 0; cpi < total_cpi; ++cpi) {
        // ... simulation logic
        waveform_gen.generate(/* params from config */);
        noise_engine.add_noise(/* ... */);
        // ...
    }

    // 5. Runtime config update example
    noise::NoiseConfig new_noise;
    new_noise.sigma_complex = 2.0e-3;
    config_manager.update_noise(new_noise);  // NoiseEngine auto-notified

    return 0;
}
```

## Implementation Order

1. Create ConfigurationManager and IConfigObserver interface
2. Add JSON serialization to all config structs
3. Update Engine interfaces to accept separated configs
4. Update Engines to implement IConfigObserver
5. Move files to new directories (antenna_set → antenna/, waveform → waveform/)
6. Delete radar_params.h and radar_params.cpp
7. Update main.cpp to use new architecture
8. Update CMakeLists.txt for new directory structure
9. Update all #include paths
10. Test compilation and functionality

## Dependencies

- nlohmann/json library for JSON parsing (header-only, add to third_party/)
- C++17 for std::type_index and template improvements

## Risk Assessment

| Risk | Mitigation |
|------|------------|
| Breaking existing code during transition | Incremental migration, keep tests passing at each step |
| Observer notification order issues | Document that order is unspecified, Engines should be independent |
| JSON parsing errors | Strict validation with error messages, fallback to defaults |
| Circular dependencies in headers | RadarConfig only contains data, ConfigurationManager manages observers |