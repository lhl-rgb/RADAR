include_guard(GLOBAL)

if(RADAR_ENABLE_CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)
find_package(GDAL REQUIRED)
find_package(FFTW3 QUIET)

if(TARGET FFTW3::fftw3)
  set(_radar_fftw3_target FFTW3::fftw3)
elseif(FFTW3_FOUND)
  add_library(radar_fftw3_fallback INTERFACE)
  target_include_directories(radar_fftw3_fallback INTERFACE ${FFTW3_INCLUDE_DIRS})
  target_link_libraries(radar_fftw3_fallback INTERFACE ${FFTW3_LIBRARIES})
  set(_radar_fftw3_target radar_fftw3_fallback)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(FFTW3 REQUIRED IMPORTED_TARGET fftw3)
  set(_radar_fftw3_target PkgConfig::FFTW3)
endif()

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
    ${_radar_fftw3_target}
)

if(RADAR_ENABLE_CUDA)
  target_link_libraries(radar_deps INTERFACE CUDA::cudart)
endif()

target_include_directories(
  radar_deps
  INTERFACE
    "${PROJECT_SOURCE_DIR}/include"
)
