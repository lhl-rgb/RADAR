# =============================================================================
# radar_protobuf - Protobuf/gRPC 代码生成
# =============================================================================
# 在 configure 阶段运行 protoc，生成：
#   - radar_service.pb.cc       (消息序列化)
#   - radar_service.grpc.pb.cc  (gRPC 服务桩)
# =============================================================================

# Proto 源文件路径
set(RADAR_PROTO "${CMAKE_CURRENT_SOURCE_DIR}/radar_service.proto")

# 生成输出目录（放在 build 目录下，不污染源码）
set(GRPC_OUTPUT_DIR "${CMAKE_BINARY_DIR}/grpc")
file(MAKE_DIRECTORY "${GRPC_OUTPUT_DIR}")

# ---------------------------------------------------------------------------
# 运行 protoc 生成 C++ 代码
# ---------------------------------------------------------------------------
# --proto_path  : proto 文件搜索路径
# --cpp_out     : 消息类生成目录
# --grpc_out    : gRPC 服务类生成目录
# --plugin      : gRPC C++ 插件路径
# ---------------------------------------------------------------------------
execute_process(
    COMMAND ${PROTOBUF_PROTOC}
    --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
    --cpp_out=${GRPC_OUTPUT_DIR}
    --grpc_out=${GRPC_OUTPUT_DIR}
    --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
    ${RADAR_PROTO}
    RESULT_VARIABLE GRPC_RESULT
)

# 生成的源文件
set(_proto_srcs "${GRPC_OUTPUT_DIR}/radar_service.pb.cc")
set(_grpc_srcs  "${GRPC_OUTPUT_DIR}/radar_service.grpc.pb.cc")

# ---------------------------------------------------------------------------
# radar::protobuf - 静态库
# ---------------------------------------------------------------------------
add_library(radar_protobuf STATIC
    ${_proto_srcs}
    ${_grpc_srcs}
)

# 头文件路径：build 目录（pb.h 生成位置）+ 当前源码目录
target_include_directories(radar_protobuf
    PUBLIC
        "${CMAKE_BINARY_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

# 依赖 gRPC 和 Protobuf 运行时
target_link_libraries(radar_protobuf
    PUBLIC
        gRPC::grpc++
        protobuf::libprotobuf
)

# 别名
add_library(radar::protobuf ALIAS radar_protobuf)
