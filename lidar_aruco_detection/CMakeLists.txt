# Find required packages
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Define the library source files
set(LIDAR_ARUCO_DETECTION_SOURCES
    src/euclidean_clustering.cpp
    src/image_marker_detection.cpp
    src/image_projection.cpp
    src/intensity_gradient_filtering.cpp
    src/oriented_bounding_box.cpp
    src/visualisation.cpp
)

# Create the shared library
add_library(lidar_aruco_detection SHARED ${LIDAR_ARUCO_DETECTION_SOURCES})

# Set the library's include directories
target_include_directories(lidar_aruco_detection
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS}
)

# Link against required libraries
target_link_libraries(lidar_aruco_detection
    PUBLIC
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        Eigen3::Eigen
        yaml-cpp
)

# Add PCL definitions
target_compile_definitions(lidar_aruco_detection PRIVATE ${PCL_DEFINITIONS})

# Set the library's output name
set_target_properties(lidar_aruco_detection PROPERTIES
    OUTPUT_NAME "lidar_aruco_detection"
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Install the library
install(TARGETS lidar_aruco_detection
    EXPORT lidar_aruco_detection-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install the headers
install(DIRECTORY include/
    DESTINATION include
)

# Export the targets
install(EXPORT lidar_aruco_detection-targets
    FILE lidar_aruco_detection-targets.cmake
    NAMESPACE lidar_aruco_detection::
    DESTINATION lib/cmake/lidar_aruco_detection
)

# Create and install the config file
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/lidar_aruco_detection-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/lidar_aruco_detection-config.cmake
    INSTALL_DESTINATION lib/cmake/lidar_aruco_detection
)

# Install the config file
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/lidar_aruco_detection-config.cmake
    DESTINATION lib/cmake/lidar_aruco_detection
)
