#include <cstddef>
#include <iostream>

#include <memory>
#include <pcl/common/angles.h>
#include <pcl/common/common.h>

#include <lidar_aruco_detection/lidar_aruco_detection.hpp>
#include <lidar_aruco_detection/visualisation.hpp>

#include "helpers.hpp"

// Use the namespace for convenience
using namespace lidar_aruco_detection;

void print_usage(const char *programName) {
    std::cout << "Usage: " << programName << " <pcd_file_path> [scale]" << std::endl;
    std::cout << "  pcd_file_path: Path to the PCD file to process" << std::endl;
    std::cout << "  scale: Optional scaling factor for coordinates (default: 1.0)" << std::endl;
}

void print_cloud_bounds(const PointCloud::Ptr &cloud) {
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    std::cout << "Point cloud bounds:" << std::endl;
    std::cout << "  X: [" << min_pt[0] << ", " << max_pt[0] << "]" << std::endl;
    std::cout << "  Y: [" << min_pt[1] << ", " << max_pt[1] << "]" << std::endl;
    std::cout << "  Z: [" << min_pt[2] << ", " << max_pt[2] << "]" << std::endl;
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

    print_cloud_bounds(cloud);

    add_point_cloud_intensity(viewer, cloud, "original", viewports.v1);

    // Create configuration with default values
    IntensityGradientConfiguration intensity_gradient_config;
    MarkerSearchConfiguration marker_search_config;

    // Create debug data structure
    auto debug_data = std::make_shared<IntensityGradientDebugData>();

    // Detect markers using the configuration
    const auto markers = detect_markers_using_intensity_gradient_clustering(
        cloud, intensity_gradient_config, marker_search_config, debug_data
    );

    add_point_cloud(viewer, debug_data->significant_gradient_points, "filtered", viewports.v2, 1.0, 0.0, 0.0, 2.0);
    visualize_clusters(viewer, debug_data->euclidean_clusters, viewports.v2);
    visualize_oriented_bounding_boxes(viewer, debug_data->filtered_obbs, viewports.v2);

    size_t i = 0;
    for (const auto &filtered_points : debug_data->obb_filtered_points) {
        add_point_cloud_intensity(viewer, filtered_points, "box_points_" + std::to_string(i), viewports.v3, 3.0);
        i++;
    }

    i = 0;
    for (const auto &marker_search : debug_data->obb_marker_searches) {
        const auto range_cv_image = convert_range_image_to_cv_mat(marker_search.range_image);
        cv::Mat marked_image = draw_markers(marker_search.intensity_image, marker_search.image_marker_detections);
        cv::imwrite("range_image_" + std::to_string(i) + ".png", range_cv_image);
        cv::imwrite("intensity_image_" + std::to_string(i) + ".png", marker_search.intensity_image);
        cv::imwrite(
            "threshold_intensity_image_" + std::to_string(i) + ".png", marker_search.thresholded_intensity_image
        );
        cv::imwrite("marker_detection_" + std::to_string(i) + ".png", marked_image);

        for (const auto &detection : marker_search.marker_detections) {
            std::string marker_points_id = "marker_" + std::to_string(i) + "_id_" + std::to_string(detection.id);
            visualize_3d_points(viewer, detection.corner_points, marker_points_id, viewports.v4, 1.0, 0.0, 0.0, 8.0);
        }
        i++;
    }

    i = 0;
    for (const auto &marker : markers) {
        std::string marker_pose_id = "marker_pose_" + std::to_string(i) + "_id_" + std::to_string(marker.id);
        visualize_marker_pose(viewer, marker.position, marker.orientation, marker_pose_id, viewports.v4, 0.2);
        i++;
    }

    // Add the original cloud to the fourth viewport for reference
    add_point_cloud_intensity(viewer, cloud, "original_cloud_v4", viewports.v4, 1.0);

    std::cout << "Press 'q' to exit visualization..." << std::endl;
    viewer->spin();

    return 0;
}
