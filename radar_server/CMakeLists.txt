# =============================================================================
# radar_server - gRPC 雷达仿真服务端
# =============================================================================
# 构建两个目标：
#   radar_server_lib  - 服务端核心逻辑（静态库，供 client 复用）
#   radar_server      - 服务端可执行文件
# =============================================================================

# ---------------------------------------------------------------------------
# 收集源文件
# ---------------------------------------------------------------------------
file(GLOB_RECURSE server_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
)

# ---------------------------------------------------------------------------
# radar_server_lib - 服务端核心逻辑（静态库）
# ---------------------------------------------------------------------------
# 作为静态库是因为 radar_client 也需要链接服务端逻辑（嵌入式服务器模式）
# ---------------------------------------------------------------------------
add_library(radar_server_lib STATIC
    ${server_sources}
)

target_include_directories(radar_server_lib
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"      # 服务端头文件
        "${CMAKE_SOURCE_DIR}/radar_common/include"  # 雷达核心头文件
        "${CMAKE_BINARY_DIR}/grpc"                  # protobuf 生成的头文件
)

# 公开依赖：使用 radar_server_lib 的目标自动获得这些依赖
target_link_libraries(radar_server_lib
    PUBLIC
        radar::common       # 雷达核心算法
        radar::protobuf     # Protobuf/gRPC 桩代码
        gRPC::grpc++        # gRPC C++ 运行时
        protobuf::libprotobuf
)

# ---------------------------------------------------------------------------
# radar_server - 可执行文件
# ---------------------------------------------------------------------------
add_executable(radar_server "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

target_link_libraries(radar_server
    PRIVATE
        radar_server_lib    # 服务端核心逻辑
        radar::common       # 雷达核心算法
        radar::protobuf     # Protobuf/gRPC 桩代码
        gRPC::grpc++
)

# 链接器选项：允许延迟绑定（解决共享库依赖顺序问题）
target_link_options(radar_server PRIVATE "LINKER:--copy-dt-needed-entries")
