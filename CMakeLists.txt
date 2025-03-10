cmake_minimum_required(VERSION 3.14)
project(
    lidar_aruco_detection
    VERSION 0.1.0
    DESCRIPTION "LiDAR ArUco marker detection library"
    LANGUAGES C CXX
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for tools like clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add the library subdirectory
add_subdirectory(lidar_aruco_detection)

# Add the examples subdirectory
add_subdirectory(examples)
