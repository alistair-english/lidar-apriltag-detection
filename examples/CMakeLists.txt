# Find required packages
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Create the main application
add_executable(tag_detection main.cpp)
target_link_libraries(tag_detection
    PRIVATE
        lidar_aruco_detection
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        Eigen3::Eigen
        yaml-cpp
)

# Create the marker creator application
add_executable(create_markers main_create_markers.cpp)
target_link_libraries(create_markers
    PRIVATE
        lidar_aruco_detection
        ${OpenCV_LIBS}
)

# Install the applications
install(TARGETS tag_detection create_markers
    RUNTIME DESTINATION bin
)

# Copy config file to build directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml")
  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.yaml ${CMAKE_CURRENT_BINARY_DIR}/config.yaml COPYONLY)
endif()
