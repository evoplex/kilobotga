cmake_minimum_required(VERSION 2.8.12)
project(kilobotga)

# Make sure we are under Unix
if(NOT UNIX)
  message(FATAL_ERROR "ARGoS compiles only under UNIX, sorry!")
endif(NOT UNIX)

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Find and include additional cmake scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildOptions.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildFlags.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/ARGoSBuildChecks.cmake)

# Descend into the subdirectories
add_subdirectory(controllers)
add_subdirectory(loop_functions)

