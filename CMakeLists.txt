cmake_minimum_required(VERSION 3.1)
project(aruco_detection)

find_package(PCL 1.3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${YAMLCPP_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

# add_executable( original_tag_detection src/original/main.cpp src/original/configuration.cpp
# src/original/visualisation.cpp src/original/feature_extraction.cpp ) target_link_libraries(original_tag_detection
# ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)

add_executable(tag_detection src/main.cpp)
target_link_libraries(tag_detection ${PCL_LIBRARIES} ${OpenCV_LIBS})

# # Copy config file to build directory if it exists if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml")
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config.yaml COPYONLY) endif()
