#include <iostream>
#include <string>

#include <cuda_runtime_api.h>
#include <Eigen/Dense>
#include <gdal.h>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include "core/types.h"
#include "core/radar_params.h"
#include "core/antenna_set.h"
using radar::RadarParams;
int main() {
  
  RadarParams params;
  params.print();
  return 0;
}
