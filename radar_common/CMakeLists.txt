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
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_SOURCE_DIR}/radar_common/include"
)

set(RADAR_MODULES core antenna waveform clutter target noise)

foreach(module_name IN LISTS RADAR_MODULES)
    file(GLOB_RECURSE module_sources
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}/*.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/${module_name}/*.cc"
    )

    set(module_target "radar_${module_name}")
    if(module_sources)
        add_library("${module_target}" STATIC ${module_sources})
    else()
        add_library("${module_target}" INTERFACE)
    endif()

    target_include_directories("${module_target}" 
        PUBLIC 
            "${CMAKE_CURRENT_SOURCE_DIR}/include"
            "${CMAKE_SOURCE_DIR}/radar_common/include"
    )
    target_link_libraries("${module_target}" PUBLIC radar::deps)
    target_compile_features("${module_target}" PUBLIC cxx_std_17)

    add_library("radar::${module_name}" ALIAS "${module_target}")
endforeach()

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
)