file(GLOB_RECURSE client_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
)

add_executable(radar_client ${client_sources})

set_target_properties(radar_client PROPERTIES AUTOMOC ON)

target_include_directories(radar_client
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/radar_common/include"
        "${CMAKE_SOURCE_DIR}/radar_server/include"
)

target_link_libraries(radar_client
    PRIVATE
        radar_server_lib
        radar::common
        radar::protobuf
        gRPC::grpc++
        Qt${QT_VERSION_MAJOR}::Widgets
)

target_link_options(radar_client PRIVATE "LINKER:--copy-dt-needed-entries")

set_target_properties(radar_client PROPERTIES OUTPUT_NAME radar_client)