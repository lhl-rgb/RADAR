include_guard(GLOBAL)

if(RADAR_ENABLE_CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)
find_package(GDAL REQUIRED)

add_library(radar_deps INTERFACE)
add_library(radar::deps ALIAS radar_deps)

if(TARGET GDAL::GDAL)
  set(_radar_gdal_target GDAL::GDAL)
else()
  add_library(radar_gdal_fallback INTERFACE)
  target_include_directories(radar_gdal_fallback INTERFACE ${GDAL_INCLUDE_DIRS})
  target_link_libraries(radar_gdal_fallback INTERFACE ${GDAL_LIBRARIES})
  set(_radar_gdal_target radar_gdal_fallback)
endif()

target_link_libraries(
  radar_deps
  INTERFACE
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    spdlog::spdlog
    ${_radar_gdal_target}
)

if(RADAR_ENABLE_CUDA)
  target_link_libraries(radar_deps INTERFACE CUDA::cudart)
endif()

target_include_directories(
  radar_deps
  INTERFACE
    "${PROJECT_SOURCE_DIR}/include"
)
