# Find required packages
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Create the main application
add_executable(tag_detection tag_detection.cpp)
target_link_libraries(
    tag_detection PRIVATE lidar_aruco_detection::lidar_aruco_detection ${PCL_LIBRARIES} ${OpenCV_LIBS}
)

# Create the marker creator application
add_executable(create_markers create_markers.cpp)
target_link_libraries(create_markers PRIVATE ${OpenCV_LIBS})

# Set output directory for executables
set_target_properties(tag_detection create_markers PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Install the applications
install(TARGETS tag_detection create_markers RUNTIME DESTINATION bin)
