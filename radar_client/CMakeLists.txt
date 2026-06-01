# =============================================================================
# radar_client - Qt GUI 雷达仿真客户端
# =============================================================================
# 基于 Qt Widgets 的图形界面，内嵌 gRPC 客户端连接服务端
# 使用 AUTOMOC 自动处理 Qt 的信号/槽元对象编译
# =============================================================================

# ---------------------------------------------------------------------------
# 收集源文件
# ---------------------------------------------------------------------------
file(GLOB_RECURSE client_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
)

# ---------------------------------------------------------------------------
# radar_client - 可执行文件
# ---------------------------------------------------------------------------
add_executable(radar_client ${client_sources})

# Qt 元对象编译器（MOC）自动处理 Q_OBJECT 宏
set_target_properties(radar_client PROPERTIES AUTOMOC ON)

# 头文件搜索路径
target_include_directories(radar_client
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"       # 客户端头文件
        "${CMAKE_SOURCE_DIR}/radar_common/include"   # 雷达核心头文件
        "${CMAKE_SOURCE_DIR}/radar_server/include"   # 服务端头文件（嵌入式服务器）
)

# 链接依赖
target_link_libraries(radar_client
    PRIVATE
        radar_server_lib                # 服务端逻辑（嵌入式服务器模式）
        radar::common                   # 雷达核心算法
        radar::protobuf                 # Protobuf/gRPC 桩代码
        gRPC::grpc++                    # gRPC C++ 运行时
        Qt${QT_VERSION_MAJOR}::Widgets  # Qt Widgets GUI
)

# 链接器选项：允许延迟绑定
target_link_options(radar_client PRIVATE "LINKER:--copy-dt-needed-entries")

# 输出文件名
set_target_properties(radar_client PROPERTIES OUTPUT_NAME radar_client)
