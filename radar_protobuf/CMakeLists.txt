set(RADAR_PROTO "${CMAKE_CURRENT_SOURCE_DIR}/radar_service.proto")

set(GRPC_OUTPUT_DIR "${CMAKE_BINARY_DIR}/grpc")
file(MAKE_DIRECTORY "${GRPC_OUTPUT_DIR}")

execute_process(
    COMMAND ${PROTOBUF_PROTOC}
    --proto_path=${CMAKE_CURRENT_SOURCE_DIR}
    --cpp_out=${GRPC_OUTPUT_DIR}
    --grpc_out=${GRPC_OUTPUT_DIR}
    --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
    ${RADAR_PROTO}
    RESULT_VARIABLE GRPC_RESULT
)

set(_proto_srcs "${GRPC_OUTPUT_DIR}/radar_service.pb.cc")
set(_grpc_srcs "${GRPC_OUTPUT_DIR}/radar_service.grpc.pb.cc")

add_library(radar_protobuf STATIC
    ${_proto_srcs}
    ${_grpc_srcs}
)

target_include_directories(radar_protobuf
    PUBLIC
        "${CMAKE_BINARY_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(radar_protobuf
    PUBLIC
        gRPC::grpc++
        protobuf::libprotobuf
)

add_library(radar::protobuf ALIAS radar_protobuf)