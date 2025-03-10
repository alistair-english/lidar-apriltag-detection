cmake_minimum_required(VERSION 3.10)
project(lidar_aruco_detection VERSION 0.1.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the library subdirectory
add_subdirectory(lidar_aruco_detection)

# Add the examples subdirectory
add_subdirectory(examples)

# Copy config file to build directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config.yaml COPYONLY)
endif()
