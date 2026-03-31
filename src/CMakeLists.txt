set(RADAR_MODULES
  core
  antenna
  clutter
  target
  verify
  noise
)

function(radar_collect_module_sources out_var module_name)
  set(_patterns
    "${CMAKE_CURRENT_SOURCE_DIR}/${module_name}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/${module_name}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/${module_name}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/${module_name}/*.cxx"
  )

  if(RADAR_CUDA_LANGUAGE_ENABLED)
    list(APPEND _patterns
      "${CMAKE_CURRENT_SOURCE_DIR}/${module_name}/*.cu"
      "${CMAKE_CURRENT_SOURCE_DIR}/${module_name}/*.cuh"
    )
  endif()

  file(
    GLOB_RECURSE _sources
    CONFIGURE_DEPENDS
    ${_patterns}
  )
  set("${out_var}" "${_sources}" PARENT_SCOPE)
endfunction()

foreach(module_name IN LISTS RADAR_MODULES)
  radar_collect_module_sources(module_sources "${module_name}")

  set(module_target "radar_${module_name}")
  if(module_sources)
    add_library("${module_target}" STATIC)
    target_sources("${module_target}" PRIVATE ${module_sources})

    target_include_directories(
      "${module_target}"
      PUBLIC
        "${PROJECT_SOURCE_DIR}/include"
    )
    target_link_libraries(
      "${module_target}"
      PUBLIC
        radar::deps
    )
    radar_apply_target_options("${module_target}")
  else()
    add_library("${module_target}" INTERFACE)
    target_include_directories(
      "${module_target}"
      INTERFACE
        "${PROJECT_SOURCE_DIR}/include"
    )
    target_link_libraries(
      "${module_target}"
      INTERFACE
        radar::deps
    )
    radar_apply_target_options("${module_target}")
  endif()

  add_library("radar::${module_name}" ALIAS "${module_target}")
endforeach()

add_library(radar_modules INTERFACE)
add_library(radar::modules ALIAS radar_modules)
target_link_libraries(
  radar_modules
  INTERFACE
    radar::core
    radar::antenna
    radar::clutter
    radar::target
    radar::verify
    radar::noise
)
