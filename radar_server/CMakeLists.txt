file(GLOB_RECURSE server_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
)

add_library(radar_server_lib STATIC
    ${server_sources}
)

target_include_directories(radar_server_lib
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/radar_common/include"
        "${CMAKE_BINARY_DIR}/grpc"
)

target_link_libraries(radar_server_lib
    PUBLIC
        radar::common
        radar::protobuf
        gRPC::grpc++
        protobuf::libprotobuf
)

add_executable(radar_server "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

target_link_libraries(radar_server
    PRIVATE
        radar_server_lib
        radar::common
        radar::protobuf
        gRPC::grpc++
)

target_link_options(radar_server PRIVATE "LINKER:--copy-dt-needed-entries")