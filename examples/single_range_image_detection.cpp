#include <iostream>
#include <memory>
#include <pcl/common/common.h>

#include <lidar_aruco_detection/lidar_aruco_detection.hpp>
#include <lidar_aruco_detection/pointcloud.hpp>
#include <lidar_aruco_detection/visualisation.hpp>

#include "helpers.hpp"

// Use the namespace for convenience
using namespace lidar_aruco_detection;

void print_usage(const char *programName) {
    std::cout << "Usage: " << programName << " <pcd_file_path> [scale]" << std::endl;
    std::cout << "  pcd_file_path: Path to the PCD file to process" << std::endl;
    std::cout << "  scale: Optional scaling factor for coordinates (default: 1.0)" << std::endl;
}

struct ProgramArgs {
    std::string pcd_file_path;
    float scale = 1.0f;
    bool valid = false;
};

ProgramArgs parse_arguments(int argc, char **argv) {
    ProgramArgs args;

    if (argc < 2 || argc > 3) {
        print_usage(argv[0]);
        return args;
    }

    args.pcd_file_path = argv[1];

    if (argc == 3) {
        try {
            args.scale = std::stof(argv[2]);
            if (args.scale <= 0) {
                std::cerr << "Error: Scale must be a positive number" << std::endl;
                return args;
            }
        } catch (const std::exception &e) {
            std::cerr << "Error parsing scale value: " << e.what() << std::endl;
            return args;
        }
    }

    args.valid = true;
    return args;
}

int main(int argc, char **argv) {
    ProgramArgs args = parse_arguments(argc, argv);
    if (!args.valid) {
        return 1;
    }

    auto [viewer, viewports] = create_visualizer();

    std::cout << "Loading point cloud from: " << args.pcd_file_path << std::endl;
    std::cout << "Using scale factor: " << args.scale << std::endl;

    const auto cloud = load_cloud(args.pcd_file_path, args.scale);
    if (!cloud) {
        std::cerr << "Failed to load point cloud from: " << args.pcd_file_path << std::endl;
        return -1;
    }

    std::cout << "Successfully loaded point cloud with " << cloud->size() << " points." << std::endl;

    // Add the original point cloud to the first viewport
    add_point_cloud_intensity(viewer, cloud, "original", viewports.v1);

    // Add the filtered point cloud to the second viewport (if available after detection)

    // Create marker search configuration with default values
    MarkerSearchConfiguration marker_search_config;
    marker_search_config.angular_resolution_deg = 0.5;

    // Create debug data structure
    auto debug_data = std::make_shared<MarkerSearchDebugData>();

    // Detect markers using the single range image approach
    const auto markers = detect_markers_using_single_range_image_search(cloud, marker_search_config, debug_data);

    std::cout << "Detected " << markers.size() << " markers" << std::endl;

    // Save the range and intensity images
    const auto range_cv_image = convert_range_image_to_cv_mat(debug_data->range_image);
    cv::Mat marked_image = draw_markers(debug_data->intensity_image, debug_data->image_marker_detections);
    cv::imwrite("range_image.png", range_cv_image);
    cv::imwrite("intensity_image.png", debug_data->intensity_image);
    cv::imwrite("threshold_intensity_image.png", debug_data->thresholded_intensity_image);
    cv::imwrite("marker_detection.png", marked_image);

    // Visualize the detected markers
    for (size_t i = 0; i < markers.size(); i++) {
        const auto &marker = markers[i];
        std::string marker_pose_id = "marker_pose_" + std::to_string(i) + "_id_" + std::to_string(marker.id);
        visualize_marker_pose(viewer, marker.position, marker.orientation, marker_pose_id, viewports.v4, 0.2);
    }

    // Visualize the 3D corner points of detected markers
    for (size_t i = 0; i < debug_data->marker_detections.size(); i++) {
        const auto &detection = debug_data->marker_detections[i];
        std::string marker_points_id = "marker_" + std::to_string(i) + "_id_" + std::to_string(detection.id);
        visualize_3d_points(viewer, detection.corner_points, marker_points_id, viewports.v3, 1.0, 0.0, 0.0, 8.0);
    }

    // Add the original cloud to the fourth viewport for reference
    add_point_cloud_intensity(viewer, cloud, "original_cloud_v4", viewports.v4, 1.0);

    std::cout << "Press 'q' to exit visualization..." << std::endl;
    viewer->spin();

    return 0;
}
