#include <iostream>
#include <string>

#include <cuda_runtime_api.h>
#include <Eigen/Dense>
#include <gdal.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "core/types.h"
#include "core/radar_params.h"
#include "core/math_utils.h"
#include "core/antenna_set.h"
#include "core/waveform_generator.h"
#include "target/target_engine.h"
#include "target/target_manager.h"
#include "noise/noise_engine.h"
#include  "clutter/clutter_engine.h"

namespace{
using radar::AntennaScanModel;
using radar::AzEl;
using radar::ClutterEngine;
using radar::CpiEcho;
using radar::Complex;
using radar::ComplexVec;
using radar::NoiseEngine;
using radar::NoiseLevelMode;
using radar::NoiseParams;
using radar::PhaseCodeType;
using radar::PhasedArrayAntenna;
using radar::PhasedArrayAntennaConfig;
using radar::PhasedArrayModelType;
using radar::PulseEcho;
using radar::RadarParams;
using radar::Scalar;
using radar::SeaClutterModel;
using radar::SeaClutterParams;
using radar::SeaClutterSequenceMode;
using radar::SwerlingType;
using radar::MotionModel;
using radar::TargetEngine;
using radar::TargetManager;
using radar::TargetState;
using radar::BeamView;
using radar::TargetList;
using radar::WaveformGenerator;
using radar::WaveformType;
using radar::WindowType;
using radar::Vec3;
}


int main() {
  
  RadarParams params;
  WaveformGenerator waveform_gen(params);
  PhasedArrayAntenna   antenna(params.antenna_config);
  
  return 0;
}
