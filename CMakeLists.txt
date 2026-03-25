cmake_minimum_required(VERSION 3.21)

project(Radar VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(RADAR_ENABLE_CUDA "Enable CUDA support" ON)
option(RADAR_BUILD_TESTS "Build test targets" ON)

if(RADAR_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    set(RADAR_CUDA_LANGUAGE_ENABLED ON)
  else()
    set(RADAR_CUDA_LANGUAGE_ENABLED OFF)
    message(WARNING "CUDA compiler not found. CUDA source files (*.cu) will be ignored.")
  endif()
else()
  set(RADAR_CUDA_LANGUAGE_ENABLED OFF)
endif()

set(
  RADAR_CUDA_LANGUAGE_ENABLED
  "${RADAR_CUDA_LANGUAGE_ENABLED}"
  CACHE INTERNAL "Whether CUDA language is enabled for this configure run."
)

include(ProjectOptions)
include(Dependencies)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src")
  add_subdirectory(src)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/app")
  add_subdirectory(app)
endif()

if(RADAR_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test")
  enable_testing()
  add_subdirectory(test)
endif()
