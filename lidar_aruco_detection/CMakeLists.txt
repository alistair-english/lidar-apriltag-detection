# Find required packages
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Define the library source files
set(LIDAR_ARUCO_DETECTION_SOURCES
    src/euclidean_clustering.cpp src/image_marker_detection.cpp src/image_projection.cpp
    src/intensity_gradient_filtering.cpp src/oriented_bounding_box.cpp src/visualisation.cpp
)

# Create the shared library
add_library(lidar_aruco_detection SHARED ${LIDAR_ARUCO_DETECTION_SOURCES})
add_library(lidar_aruco_detection::lidar_aruco_detection ALIAS lidar_aruco_detection)

# Set the library's include directories
target_include_directories(
    lidar_aruco_detection PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)

# Link against required libraries
target_link_libraries(lidar_aruco_detection PUBLIC Eigen3::Eigen PRIVATE ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Add PCL include directories and definitions
target_include_directories(lidar_aruco_detection PRIVATE ${PCL_INCLUDE_DIRS})
target_compile_definitions(lidar_aruco_detection PRIVATE ${PCL_DEFINITIONS})

# Add OpenCV include directories
target_include_directories(lidar_aruco_detection PRIVATE ${OpenCV_INCLUDE_DIRS})

# Set the library's output name and properties
set_target_properties(
    lidar_aruco_detection
    PROPERTIES OUTPUT_NAME "lidar_aruco_detection"
               VERSION ${PROJECT_VERSION}
               SOVERSION ${PROJECT_VERSION_MAJOR}
               POSITION_INDEPENDENT_CODE ON
)

# Generate and install package configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/lidar_aruco_detection-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/lidar_aruco_detection-config.cmake INSTALL_DESTINATION lib/cmake/lidar_aruco_detection
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/lidar_aruco_detection-config-version.cmake VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Install targets
install(
    TARGETS lidar_aruco_detection
    EXPORT lidar_aruco_detection-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install CMake configuration files
install(
    EXPORT lidar_aruco_detection-targets
    FILE lidar_aruco_detection-targets.cmake
    NAMESPACE lidar_aruco_detection::
    DESTINATION lib/cmake/lidar_aruco_detection
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/lidar_aruco_detection-config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/lidar_aruco_detection-config-version.cmake
        DESTINATION lib/cmake/lidar_aruco_detection
)
