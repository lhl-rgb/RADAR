set(_test_patterns
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cxx"
)

if(RADAR_CUDA_LANGUAGE_ENABLED)
  list(APPEND _test_patterns "${CMAKE_CURRENT_SOURCE_DIR}/*.cu")
endif()

file(
  GLOB_RECURSE RADAR_TEST_SOURCES
  CONFIGURE_DEPENDS
  ${_test_patterns}
)

if(NOT RADAR_TEST_SOURCES)
  message(STATUS "No test sources found in '${CMAKE_CURRENT_SOURCE_DIR}', skipping test target creation.")
  return()
endif()

add_executable(radar_tests ${RADAR_TEST_SOURCES})
target_link_libraries(radar_tests PRIVATE radar::modules radar::deps)
radar_apply_target_options(radar_tests)

add_test(NAME radar_tests COMMAND radar_tests)
