# =============================================================================
# Radar 项目根 CMakeLists.txt
# =============================================================================
# 雷达仿真系统 - 支持 CUDA GPU 加速、gRPC 服务端、Qt 客户端
# =============================================================================

cmake_minimum_required(VERSION 3.21)

# 启用 CXX 和 CUDA 语言（CUDA 由 RADAR_ENABLE_CUDA 选项控制）
project(Radar VERSION 0.1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# 输出目录配置
# ---------------------------------------------------------------------------
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/out/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/out/bin")

# ---------------------------------------------------------------------------
# 构建选项
# ---------------------------------------------------------------------------
option(RADAR_ENABLE_CUDA  "启用 CUDA GPU 加速支持（cufft/cublas/curand）" ON)
option(RADAR_BUILD_TESTS  "构建测试目标" ON)
option(RADAR_BUILD_CLIENT "构建 Qt 客户端 (radar_client)" ON)
option(RADAR_BUILD_SERVER "构建 gRPC 服务端 (radar_server)" ON)

# ---------------------------------------------------------------------------
# C++ 标准
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra)

# ===========================================================================
# CUDA 配置
# ===========================================================================
# 当启用 CUDA 时：
#   1. 启用 CUDA 语言，设置架构和标准
#   2. 查找 CUDA Toolkit（cudart, cufft, cublas, curand）
#   3. 在 radar_common 中链接这些库
# ===========================================================================
if(RADAR_ENABLE_CUDA)
    enable_language(CUDA)

    # CUDA 架构设置（根据实际 GPU 调整）
    # 75 = Turing (T4, RTX 20xx), 80 = Ampere (A100, RTX 30xx), 86 = RTX 3070/3080, 89 = RTX 40xx
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
        set(CMAKE_CUDA_ARCHITECTURES "65;75;80;86;89")
    endif()

    # CUDA 标准（与 C++17 对齐）
    set(CMAKE_CUDA_STANDARD 17)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)

    # 查找 CUDA Toolkit 组件
    find_package(CUDAToolkit REQUIRED)

    message(STATUS "CUDA 支持已启用")
    message(STATUS "  CUDA Toolkit: ${CUDAToolkit_VERSION}")
    message(STATUS "  CUDA 架构:  ${CMAKE_CUDA_ARCHITECTURES}")
    message(STATUS "  nvcc:       ${CMAKE_CUDA_COMPILER}")
else()
    message(STATUS "CUDA 支持已禁用")
endif()

# ===========================================================================
# 第三方依赖
# ===========================================================================

# --- 数学/科学计算 ---
find_package(Eigen3 REQUIRED NO_MODULE)       # 线性代数库

# --- 数据格式 ---
find_package(nlohmann_json REQUIRED)          # JSON 序列化

# --- 日志 ---
find_package(spdlog REQUIRED)                 # 高性能日志库

# --- 地理数据 ---
find_package(GDAL REQUIRED)                   # 地理空间数据抽象库

# --- Protobuf / gRPC ---
find_package(Protobuf QUIET)
if(Protobuf_FOUND)
    if(NOT TARGET protobuf::libprotobuf)
        add_library(protobuf::libprotobuf UNKNOWN IMPORTED)
        set_target_properties(protobuf::libprotobuf PROPERTIES
            IMPORTED_LOCATION "${Protobuf_LIBRARIES}"
            INTERFACE_INCLUDE_DIRECTORIES "${Protobuf_INCLUDE_DIRS}"
        )
    endif()
else()
    find_library(PROTOBUF_LIBRARY
        NAMES protobuf libprotobuf.so libprotobuf.so.32 libprotobuf.so.23
        PATHS "$ENV{HOME}/.local/lib/x86_64-linux-gnu" /usr/lib/x86_64-linux-gnu
        REQUIRED
    )
    find_path(PROTOBUF_INCLUDE_DIR
        NAMES google/protobuf/message.h
        PATHS "$ENV{HOME}/.local/include" /usr/include
        REQUIRED
    )
    add_library(protobuf::libprotobuf UNKNOWN IMPORTED)
    set_target_properties(protobuf::libprotobuf PROPERTIES
        IMPORTED_LOCATION "${PROTOBUF_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PROTOBUF_INCLUDE_DIR}"
    )
endif()

find_library(GRPC_LIBRARY
    NAMES grpc++ libgrpc++.so libgrpc++.so.1.51
    PATHS "$ENV{HOME}/.local/lib/x86_64-linux-gnu" /usr/lib/x86_64-linux-gnu
    REQUIRED
)
find_library(GRPC_CORE_LIBRARY
    NAMES grpc libgrpc.so libgrpc.so.29
    PATHS "$ENV{HOME}/.local/lib/x86_64-linux-gnu" /usr/lib/x86_64-linux-gnu
    REQUIRED
)
find_path(GRPC_INCLUDE_DIR
    NAMES grpcpp/grpcpp.h
    PATHS "$ENV{HOME}/.local/include" /usr/include
    REQUIRED
)
add_library(gRPC::grpc++ UNKNOWN IMPORTED)
set_target_properties(gRPC::grpc++ PROPERTIES
    IMPORTED_LOCATION "${GRPC_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GRPC_INCLUDE_DIR}"
)
add_library(gRPC::grpc UNKNOWN IMPORTED)
set_target_properties(gRPC::grpc PROPERTIES
    IMPORTED_LOCATION "${GRPC_CORE_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GRPC_INCLUDE_DIR}"
)

find_program(PROTOBUF_PROTOC
    NAMES protoc
    PATHS "$ENV{HOME}/.local/bin" /usr/bin /usr/local/bin
    REQUIRED
)
find_program(GRPC_CPP_PLUGIN
    NAMES grpc_cpp_plugin
    PATHS "$ENV{HOME}/.local/bin" /usr/bin /usr/local/bin
    REQUIRED
)
message(STATUS "Protobuf protoc: ${PROTOBUF_PROTOC}")
message(STATUS "gRPC C++ plugin: ${GRPC_CPP_PLUGIN}")

# --- Qt（可选）---
find_package(QT NAMES Qt6 Qt5 QUIET COMPONENTS Widgets)
if(QT_FOUND)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)
    set(RADAR_QT_FOUND ON)
    message(STATUS "Qt ${QT_VERSION_MAJOR} 已找到")
endif()

# --- FFTW3（带 fallback 链：find_package -> PkgConfig）---
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

# ===========================================================================
# 子目录
# ===========================================================================
add_subdirectory(radar_protobuf)    # Protobuf/gRPC 代码生成
add_subdirectory(radar_common)      # 雷达核心算法库（含 CUDA 加速）

if(RADAR_BUILD_SERVER)
    add_subdirectory(radar_server)  # gRPC 服务端
endif()

if(RADAR_BUILD_CLIENT AND RADAR_QT_FOUND)
    add_subdirectory(radar_client)  # Qt GUI 客户端
endif()

# ===========================================================================
# 测试
# ===========================================================================
if(RADAR_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
    set(RADAR_TEST_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/test/radar_tests.cpp")
    if(EXISTS "${RADAR_TEST_MAIN}")
        add_executable(radar_tests "${RADAR_TEST_MAIN}")
        target_link_libraries(radar_tests PRIVATE radar::common)
        enable_testing()
        add_test(NAME radar_tests COMMAND radar_tests)
    endif()

    set(RADAR_ECHO_ROTATION_TOOL "${CMAKE_CURRENT_SOURCE_DIR}/test/generate_echo_rotation.cpp")
    if(EXISTS "${RADAR_ECHO_ROTATION_TOOL}")
        add_executable(generate_echo_rotation "${RADAR_ECHO_ROTATION_TOOL}")
        target_link_libraries(generate_echo_rotation PRIVATE radar::common)
    endif()

    set(RADAR_CLUTTER_BENCHMARK "${CMAKE_CURRENT_SOURCE_DIR}/test/benchmark_clutter_gpu_rotation.cpp")
    if(EXISTS "${RADAR_CLUTTER_BENCHMARK}")
        add_executable(benchmark_clutter_gpu_rotation "${RADAR_CLUTTER_BENCHMARK}")
        target_link_libraries(benchmark_clutter_gpu_rotation PRIVATE radar::common)
    endif()

    set(RADAR_CLUTTER_2D_BENCHMARK "${CMAKE_CURRENT_SOURCE_DIR}/test/benchmark_clutter_gpu_2d.cpp")
    if(EXISTS "${RADAR_CLUTTER_2D_BENCHMARK}")
        add_executable(benchmark_clutter_gpu_2d "${RADAR_CLUTTER_2D_BENCHMARK}")
        target_link_libraries(benchmark_clutter_gpu_2d PRIVATE radar::common)
    endif()

    set(RADAR_CLUTTER_STREAM_BENCHMARK "${CMAKE_CURRENT_SOURCE_DIR}/test/benchmark_clutter_gpu_stream.cpp")
    if(EXISTS "${RADAR_CLUTTER_STREAM_BENCHMARK}")
        add_executable(benchmark_clutter_gpu_stream "${RADAR_CLUTTER_STREAM_BENCHMARK}")
        target_link_libraries(benchmark_clutter_gpu_stream PRIVATE radar::common)
    endif()

    set(RADAR_CLUTTER_PATCH_TRACE "${CMAKE_CURRENT_SOURCE_DIR}/test/export_clutter_patch_trace.cpp")
    if(EXISTS "${RADAR_CLUTTER_PATCH_TRACE}")
        add_executable(export_clutter_patch_trace "${RADAR_CLUTTER_PATCH_TRACE}")
        target_link_libraries(export_clutter_patch_trace PRIVATE radar::common)
    endif()

    set(RADAR_CLUTTER_ROTATION_PPI "${CMAKE_CURRENT_SOURCE_DIR}/test/export_clutter_rotation_ppi.cpp")
    if(EXISTS "${RADAR_CLUTTER_ROTATION_PPI}")
        add_executable(export_clutter_rotation_ppi "${RADAR_CLUTTER_ROTATION_PPI}")
        target_link_libraries(export_clutter_rotation_ppi PRIVATE radar::common)
    endif()
endif()
