cmake_minimum_required(VERSION 3.21)

project(Radar VERSION 0.1.0 LANGUAGES CXX)

# 输出目录配置 - 所有输出到项目根目录的 out/ 目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/bin")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(RADAR_ENABLE_CUDA "Enable CUDA support" ON)
option(RADAR_BUILD_TESTS "Build test targets" OFF)

if(RADAR_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    set(RADAR_CUDA_LANGUAGE_ENABLED ON)
  else()
    set(RADAR_CUDA_LANGUAGE_ENABLED OFF)
    message(WARNING "CUDA compiler not found. CUDA source files (*.cu) will be ignored.")
  endif()
else()
  set(RADAR_CUDA_LANGUAGE_ENABLED OFF)
endif()

set(
  RADAR_CUDA_LANGUAGE_ENABLED
  "${RADAR_CUDA_LANGUAGE_ENABLED}"
  CACHE INTERNAL "Whether CUDA language is enabled for this configure run."
)

# C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 编译器警告
add_compile_options(-Wall -Wextra)

# 查找依赖
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

# 依赖库
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

if(RADAR_ENABLE_CUDA AND CMAKE_CUDA_COMPILER)
  target_link_libraries(radar_deps INTERFACE CUDA::cudart_static)
endif()

target_include_directories(
  radar_deps
  INTERFACE
    "${PROJECT_SOURCE_DIR}/include"
)

# 模块定义
set(RADAR_MODULES
  core
  antenna
  waveform
  clutter
  target
  noise
)

function(radar_collect_module_sources out_var module_name)
  set(_patterns
    "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}/*.cxx"
  )

  file(
    GLOB_RECURSE _sources
    CONFIGURE_DEPENDS
    ${_patterns}
  )
  set("${out_var}" "${_sources}" PARENT_SCOPE)
endfunction()

# 编译各模块
foreach(module_name IN LISTS RADAR_MODULES)
  radar_collect_module_sources(module_sources "${module_name}")

  set(module_target "radar_${module_name}")
  if(module_sources)
    add_library("${module_target}" STATIC ${module_sources})
  else()
    add_library("${module_target}" INTERFACE)
  endif()

  target_include_directories("${module_target}" PUBLIC "${PROJECT_SOURCE_DIR}/include")
  target_link_libraries("${module_target}" PUBLIC radar::deps)
  target_compile_features("${module_target}" PUBLIC cxx_std_17)

  add_library("radar::${module_name}" ALIAS "${module_target}")
endforeach()

# 模块集合
add_library(radar_modules INTERFACE)
add_library(radar::modules ALIAS radar_modules)
target_link_libraries(
  radar_modules
  INTERFACE
    radar::core
    radar::antenna
    radar::waveform
    radar::clutter
    radar::target
    radar::noise
)

# 主程序
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app")
  file(GLOB RADAR_APP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/app/*.cpp")
  if(RADAR_APP_SOURCES)
    add_executable(radar_app ${RADAR_APP_SOURCES})
    target_include_directories(radar_app PRIVATE "${PROJECT_SOURCE_DIR}/include")
    target_link_libraries(radar_app PRIVATE radar::modules radar::deps)
    target_compile_features(radar_app PRIVATE cxx_std_17)
    set_target_properties(radar_app PROPERTIES OUTPUT_NAME radar)
  endif()
endif()

# 测试
if(RADAR_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
  file(GLOB RADAR_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")
  if(RADAR_TEST_SOURCES)
    add_executable(radar_tests ${RADAR_TEST_SOURCES})
    target_link_libraries(radar_tests PRIVATE radar::modules radar::deps)
    enable_testing()
    add_test(NAME radar_tests COMMAND radar_tests)
  endif()
endif()
