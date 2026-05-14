cmake_minimum_required(VERSION 3.21)

project(Radar VERSION 0.1.0 LANGUAGES CXX)

# 输出目录配置 - 所有输出到项目根目录的 out/ 目录
# bin/ 存放可执行文件，lib/ 存放静态库，build/ 存放编译中间文件
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/bin")

# 构建目录由 cmake -B 参数决定，这里只配置 Debug/Release 变体
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/out/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/out/bin")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(RADAR_ENABLE_CUDA "Enable CUDA support" ON)
option(RADAR_BUILD_TESTS "Build test targets" ON)
option(RADAR_BUILD_MAIN_ONLY "Build only main applications (radar_app, matlab_export_tool)" OFF)
option(RADAR_BUILD_VALIDATE_ONLY "Build only validation tools" OFF)

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

# ============================================================
# gRPC + Protobuf
# ============================================================
find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

# ============================================================
# Qt (GUI only, optional when building standalone server)
# ============================================================
find_package(QT NAMES Qt6 Qt5 QUIET COMPONENTS Widgets)
if(QT_FOUND)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
    set(RADAR_QT_FOUND ON)
endif()

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

# 网络库（UDP socket 需要）
find_library(PTHREAD_LIBRARY pthread)
if(PTHREAD_LIBRARY)
  target_link_libraries(radar_deps INTERFACE pthread)
endif()

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
  # 判断是否编译主程序
  set(_radar_build_main OFF)
  set(_radar_build_validate OFF)

  if(RADAR_BUILD_MAIN_ONLY)
    set(_radar_build_main ON)
  elseif(RADAR_BUILD_VALIDATE_ONLY)
    set(_radar_build_validate ON)
  else()
    # 默认全部编译
    set(_radar_build_main ON)
    set(_radar_build_validate ON)
  endif()

  # 主雷达应用
  if(_radar_build_main AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/main.cpp")
    add_executable(radar_app "${CMAKE_CURRENT_SOURCE_DIR}/app/main.cpp")
    target_include_directories(radar_app PRIVATE "${PROJECT_SOURCE_DIR}/include")
    target_link_libraries(radar_app PRIVATE radar::modules radar::deps)
    target_compile_features(radar_app PRIVATE cxx_std_17)
    set_target_properties(radar_app PROPERTIES OUTPUT_NAME radar)
  endif()

  # MATLAB 导出工具
  if(_radar_build_main AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/matlab_export_tool.cpp")
    add_executable(matlab_export_tool "${CMAKE_CURRENT_SOURCE_DIR}/app/matlab_export_tool.cpp")
    target_include_directories(matlab_export_tool PRIVATE "${PROJECT_SOURCE_DIR}/include")
    target_link_libraries(matlab_export_tool PRIVATE radar::modules radar::deps)
    target_compile_features(matlab_export_tool PRIVATE cxx_std_17)
    set_target_properties(matlab_export_tool PROPERTIES OUTPUT_NAME matlab_export_tool)
  endif()

  # 波形验证工具
  if(_radar_build_validate AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/waveform_validation.cpp")
    add_executable(waveform_validation "${CMAKE_CURRENT_SOURCE_DIR}/app/waveform_validation.cpp")
    target_include_directories(waveform_validation PRIVATE "${PROJECT_SOURCE_DIR}/include")
    target_link_libraries(waveform_validation PRIVATE radar::modules radar::deps)
    target_compile_features(waveform_validation PRIVATE cxx_std_17)
    set_target_properties(waveform_validation PROPERTIES OUTPUT_NAME waveform_validation)
  endif()

  # 天线验证工具
  if(_radar_build_validate AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/antenna_validation.cpp")
    add_executable(antenna_validation "${CMAKE_CURRENT_SOURCE_DIR}/app/antenna_validation.cpp")
    target_include_directories(antenna_validation PRIVATE "${PROJECT_SOURCE_DIR}/include")
    target_link_libraries(antenna_validation PRIVATE radar::modules radar::deps)
    target_compile_features(antenna_validation PRIVATE cxx_std_17)
    set_target_properties(antenna_validation PROPERTIES OUTPUT_NAME antenna_validation)
  endif()

  # 噪声验证工具
  if(_radar_build_validate AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/noise_validation.cpp")
    add_executable(noise_validation "${CMAKE_CURRENT_SOURCE_DIR}/app/noise_validation.cpp")
    target_include_directories(noise_validation PRIVATE "${PROJECT_SOURCE_DIR}/include")
    target_link_libraries(noise_validation PRIVATE radar::modules radar::deps)
    target_compile_features(noise_validation PRIVATE cxx_std_17)
    set_target_properties(noise_validation PROPERTIES OUTPUT_NAME noise_validation)
  endif()

  # 杂波验证工具
  if(_radar_build_validate AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/clutter_validation.cpp")
    add_executable(clutter_validation "${CMAKE_CURRENT_SOURCE_DIR}/app/clutter_validation.cpp")
    target_include_directories(clutter_validation PRIVATE "${PROJECT_SOURCE_DIR}/include")
    target_link_libraries(clutter_validation PRIVATE radar::modules radar::deps)
    target_compile_features(clutter_validation PRIVATE cxx_std_17)
    set_target_properties(clutter_validation PROPERTIES OUTPUT_NAME clutter_validation)
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

# ============================================================
# gRPC service library
# ============================================================
set(RADAR_PROTO "${CMAKE_CURRENT_SOURCE_DIR}/include/grpc/radar_service.proto")

file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/grpc")

protobuf_generate(
    PROTOS ${RADAR_PROTO}
    OUT_VAR _proto_srcs
)
protobuf_generate(
    PROTOS ${RADAR_PROTO}
    LANGUAGE grpc
    OUT_VAR _grpc_srcs
    PLUGIN protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>
    GENERATE_EXTENSIONS .grpc.pb.h .grpc.pb.cc
)

add_library(radar_grpc_service STATIC
    src/grpc/radar_service.cpp
    ${_proto_srcs}
    ${_grpc_srcs}
)
target_include_directories(radar_grpc_service
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_BINARY_DIR}"
)
target_link_libraries(radar_grpc_service
    PUBLIC
        radar::modules
        gRPC::grpc++
        protobuf::libprotobuf
)

# ============================================================
# Standalone gRPC server (headless, remote mode)
# ============================================================
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/radar_server.cpp")
    add_executable(radar_server app/radar_server.cpp)
    target_link_libraries(radar_server PRIVATE radar_grpc_service radar::modules gRPC::grpc++)
endif()

# ============================================================
# Qt GUI application (with embedded gRPC server)
# ============================================================
if(RADAR_QT_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app/radar_qt.cpp")
    add_executable(radar_qt app/radar_qt.cpp)
    target_link_libraries(radar_qt PRIVATE radar_grpc_service radar::modules
                          gRPC::grpc++ Qt${QT_VERSION_MAJOR}::Widgets)
endif()
