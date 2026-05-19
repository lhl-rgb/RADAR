cmake_minimum_required(VERSION 3.21)

project(Radar VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/out/bin")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/out/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/out/bin")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(RADAR_ENABLE_CUDA "Enable CUDA support" ON)
option(RADAR_BUILD_TESTS "Build test targets" ON)
option(RADAR_BUILD_CLIENT "Build Qt client (radar_client)" ON)
option(RADAR_BUILD_SERVER "Build gRPC server (radar_server)" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)
find_package(spdlog REQUIRED)
find_package(GDAL REQUIRED)

set(PROTOBUF_LIBRARY "/usr/lib/x86_64-linux-gnu/libprotobuf.so.32.0.12")
set(GRPC_LIBRARY "/usr/lib/x86_64-linux-gnu/libgrpc++.so.1.51.1")
set(GRPC_CORE_LIBRARY "/usr/lib/x86_64-linux-gnu/libgrpc.so.29.0.0")
set(GRPC_CPP_PLUGIN "/home/lhlj/.local/bin/grpc_cpp_plugin")

set(PROTOBUF_INCLUDE_DIR "/home/lhlj/.local/include")
set(GRPC_INCLUDE_DIR "/home/lhlj/.local/include")

add_library(protobuf::libprotobuf UNKNOWN IMPORTED)
set_target_properties(protobuf::libprotobuf PROPERTIES
    IMPORTED_LOCATION "${PROTOBUF_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${PROTOBUF_INCLUDE_DIR}"
)

add_library(gRPC::grpc++ UNKNOWN IMPORTED)
set_target_properties(gRPC::grpc++ PROPERTIES
    IMPORTED_LOCATION "${GRPC_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GRPC_INCLUDE_DIR}"
)

add_library(gRPC::grpc UNKNOWN IMPORTED)
set_target_properties(gRPC::grpc PROPERTIES
    IMPORTED_LOCATION "${GRPC_CORE_LIBRARY}"
)

find_program(PROTOBUF_PROTOC protoc REQUIRED)

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

add_subdirectory(radar_protobuf)
add_subdirectory(radar_common)

if(RADAR_BUILD_SERVER)
    add_subdirectory(radar_server)
endif()

if(RADAR_BUILD_CLIENT AND RADAR_QT_FOUND)
    add_subdirectory(radar_client)
endif()

if(RADAR_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
    file(GLOB RADAR_TEST_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp")
    if(RADAR_TEST_SOURCES)
        add_executable(radar_tests ${RADAR_TEST_SOURCES})
        target_link_libraries(radar_tests PRIVATE radar::common radar::deps)
        enable_testing()
        add_test(NAME radar_tests COMMAND radar_tests)
    endif()
endif()