set(_app_patterns
  "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
)

if(RADAR_CUDA_LANGUAGE_ENABLED)
  list(APPEND _app_patterns "${CMAKE_CURRENT_SOURCE_DIR}/*.cu")
endif()

file(
  GLOB_RECURSE RADAR_APP_SOURCES
  CONFIGURE_DEPENDS
  ${_app_patterns}
)

if(NOT RADAR_APP_SOURCES)
  message(FATAL_ERROR "No application sources found in '${CMAKE_CURRENT_SOURCE_DIR}'.")
endif()

add_executable(radar_app ${RADAR_APP_SOURCES})
add_executable(radar::app ALIAS radar_app)

target_include_directories(
  radar_app
  PRIVATE
    "${PROJECT_SOURCE_DIR}/include"
)

target_link_libraries(
  radar_app
  PRIVATE
    radar::modules
    radar::deps
)

radar_apply_target_options(radar_app)
set_target_properties(radar_app PROPERTIES OUTPUT_NAME radar)
