# =============================================================================
# radar_common - 雷达核心算法库
# =============================================================================
# 提供所有雷达仿真的核心模块，通过 radar::deps 聚合第三方依赖
# 模块列表：core, antenna, waveform, clutter, target, noise
# 当启用 CUDA 时，链接 cufft / cublas / curand / cudart
# =============================================================================

# ---------------------------------------------------------------------------
# radar::deps - 第三方依赖聚合（INTERFACE 库）
# ---------------------------------------------------------------------------
add_library(radar_deps INTERFACE)
add_library(radar::deps ALIAS radar_deps)

# GDAL（带 fallback：优先用 imported target，否则手动拼接）
if(TARGET GDAL::GDAL)
    set(_radar_gdal_target GDAL::GDAL)
else()
    add_library(radar_gdal_fallback INTERFACE)
    target_include_directories(radar_gdal_fallback INTERFACE ${GDAL_INCLUDE_DIRS})
    target_link_libraries(radar_gdal_fallback INTERFACE ${GDAL_LIBRARIES})
    set(_radar_gdal_target radar_gdal_fallback)
endif()

# 基础依赖
target_link_libraries(
    radar_deps
    INTERFACE
        Eigen3::Eigen               # 线性代数
        nlohmann_json::nlohmann_json # JSON 序列化
        spdlog::spdlog              # 日志
        ${_radar_gdal_target}       # 地理数据
        ${_radar_fftw3_target}      # FFT（由根 CMake 定义）
)

# pthread（Linux 线程支持）
find_library(PTHREAD_LIBRARY pthread)
if(PTHREAD_LIBRARY)
    target_link_libraries(radar_deps INTERFACE pthread)
endif()

# ---------------------------------------------------------------------------
# CUDA 依赖（当 RADAR_ENABLE_CUDA=ON 时生效）
# ---------------------------------------------------------------------------
# 链接的 CUDA 库：
#   CUDA::cudart   - CUDA 运行时（动态链接，便于调试）
#   CUDA::cufft    - CUDA FFT 加速（替代 FFTW3 的 GPU 版本）
#   CUDA::cublas   - CUDA 线性代数（矩阵运算加速）
#   CUDA::curand   - CUDA 随机数生成（噪声/杂波模拟）
# ---------------------------------------------------------------------------
if(RADAR_ENABLE_CUDA AND TARGET CUDA::cudart)
    target_link_libraries(radar_deps INTERFACE
        CUDA::cudart    # CUDA 运行时
        CUDA::cufft     # GPU FFT（雷达信号处理核心）
        CUDA::cublas    # GPU BLAS（矩阵运算加速）
        CUDA::curand    # GPU 随机数（噪声/杂波模拟）
    )
    target_compile_definitions(radar_deps INTERFACE RADAR_HAS_CUDA=1)
    message(STATUS "radar_deps: CUDA 库已链接 (cudart, cufft, cublas, curand)")
else()
    target_compile_definitions(radar_deps INTERFACE RADAR_HAS_CUDA=0)
endif()

# 公共头文件搜索路径
target_include_directories(
    radar_deps
    INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/radar_common/include"
)

# ---------------------------------------------------------------------------
# 雷达功能模块（自动扫描 src/<module>/ 下的源文件）
# ---------------------------------------------------------------------------
set(RADAR_MODULES core antenna waveform clutter target noise cuda output preview)

foreach(module_name IN LISTS RADAR_MODULES)
    # 收集模块源文件（.cpp 和 .cc）
    file(GLOB_RECURSE module_sources
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}/*.cc"
    )

    # 收集 CUDA 源文件（.cu）
    file(GLOB_RECURSE module_cuda_sources
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}/*.cu"
    )
    if(NOT RADAR_ENABLE_CUDA)
        set(module_cuda_sources)
    endif()

    set(module_target "radar_${module_name}")

    if(module_sources OR module_cuda_sources)
        # 有源文件 -> STATIC 库
        add_library("${module_target}" STATIC ${module_sources} ${module_cuda_sources})

        # CUDA 源文件属性设置
        if(module_cuda_sources)
            set_source_files_properties(${module_cuda_sources} PROPERTIES
                LANGUAGE CUDA
            )
        endif()
    else()
        # 无源文件 -> INTERFACE 库（仅传递依赖）
        add_library("${module_target}" INTERFACE)
    endif()

    if(TARGET "${module_target}")
        get_target_property(module_target_type "${module_target}" TYPE)
    endif()

    if(module_target_type STREQUAL "INTERFACE_LIBRARY")
        target_include_directories("${module_target}"
            INTERFACE
                "${CMAKE_CURRENT_SOURCE_DIR}/include"
                "${CMAKE_SOURCE_DIR}/radar_common/include"
        )
        target_link_libraries("${module_target}" INTERFACE radar::deps)
        target_compile_features("${module_target}" INTERFACE cxx_std_17)
    else()
        target_include_directories("${module_target}"
            PUBLIC
                "${CMAKE_CURRENT_SOURCE_DIR}/include"
                "${CMAKE_SOURCE_DIR}/radar_common/include"
        )
        target_link_libraries("${module_target}" PUBLIC radar::deps)
        target_compile_features("${module_target}" PUBLIC cxx_std_17)
    endif()

    # 创建别名 radar::<module>
    add_library("radar::${module_name}" ALIAS "${module_target}")
endforeach()

if(TARGET radar_clutter AND TARGET radar_cuda)
    target_link_libraries(radar_clutter PUBLIC radar::cuda)
endif()

if(TARGET radar_clutter AND TARGET radar_antenna)
    target_link_libraries(radar_clutter PUBLIC radar::antenna)
endif()

if(TARGET radar_noise AND TARGET radar_cuda)
    target_link_libraries(radar_noise PUBLIC radar::cuda)
endif()

# ---------------------------------------------------------------------------
# radar::common - 所有模块的聚合接口
# ---------------------------------------------------------------------------
add_library(radar_common INTERFACE)
add_library(radar::common ALIAS radar_common)

target_link_libraries(
    radar_common
    INTERFACE
        radar::core
        radar::antenna
        radar::waveform
        radar::clutter
        radar::target
        radar::noise
        radar::cuda
        radar::output
        radar::preview
)
