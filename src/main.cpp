#include <cstddef>
#include <iostream>

#include <pcl/common/angles.h>
#include <pcl/common/common.h>

#include "euclidean_clustering.hpp"
#include "image_projection.hpp"
#include "intensity_gradient_filtering.hpp"
#include "marker_detection.hpp"
#include "oriented_bounding_box.hpp"
#include "pointcloud.hpp"
#include "visualisation.hpp"

void printUsage(const char *programName) {
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
        printUsage(argv[0]);
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

    const auto normals = estimate_normals(cloud);
    std::cout << "Estimated " << normals->size() << " surface normals." << std::endl;

    const auto gradients = estimate_intensity_gradient(cloud, normals);
    std::cout << "Estimated " << gradients->size() << " intensity gradients." << std::endl;

    float threshold = calculate_intensity_threshold(gradients);
    std::cout << "Calculated intensity gradient threshold: " << threshold << std::endl;

    auto significant_points = extract_significant_gradient_points(cloud, gradients, threshold);
    std::cout << "Extracted " << significant_points->size() << " points with significant gradient magnitudes"
              << " (threshold: " << threshold << ")" << std::endl;

    add_point_cloud(viewer, significant_points, "filtered", viewports.v2, 1.0, 0.0, 0.0, 2.0);

    auto clusters = extract_euclidean_clusters(significant_points, 0.06, 100, 25000);
    std::cout << "Found " << clusters.size() << " clusters" << std::endl;

    visualize_clusters(viewer, clusters, viewports.v2);

    auto boxes = calculate_oriented_bounding_boxes(clusters);
    std::cout << "Calculated " << boxes.size() << " oriented bounding boxes" << std::endl;

    auto filtered_boxes = filter_obbs(boxes, 0.3, 1.0, 1.5);
    std::cout << "After OBB filtering: " << filtered_boxes.size() << " boxes remain" << std::endl;

    visualize_oriented_bounding_boxes(viewer, filtered_boxes, viewports.v2);

    float angular_resolution = pcl::deg2rad(0.3);

    size_t i = 0;
    for (const auto &obb : filtered_boxes) {
        const auto points_in_box = extract_points_in_obb(cloud, obb);

        add_point_cloud_intensity(viewer, points_in_box, "box_points_" + std::to_string(i), viewports.v3, 3.0);

        const auto transformed_cloud = transform_cloud_for_imaging(points_in_box, obb);
        const auto range_image = create_range_image_from_cloud(transformed_cloud, angular_resolution);
        auto intensity_image = create_intensity_image_from_cloud(transformed_cloud, angular_resolution, range_image);

        const auto range_cv_image = convert_range_image_to_cv_mat(range_image);

        cv::Mat threshold_intensity_image;
        cv::threshold(intensity_image, threshold_intensity_image, 10, 255, cv::THRESH_BINARY);

        // Detect ArUco markers in the blurred intensity image
        std::vector<MarkerDetection> detections = detect_markers(threshold_intensity_image, "DICT_APRILTAG_36h11");

        // Draw markers on the intensity image
        cv::Mat marked_image = draw_markers(intensity_image, detections);

        // Print marker detection results
        std::cout << "Box " << i << " marker detections:" << std::endl;
        for (const auto &detection : detections) {
            std::cout << "  Marker ID: " << detection.id << std::endl;
        }

        // Save images
        cv::imwrite("range_image_" + std::to_string(i) + ".png", range_cv_image);
        cv::imwrite("intensity_image_" + std::to_string(i) + ".png", intensity_image);
        cv::imwrite("threshold_intensity_image_" + std::to_string(i) + ".png", threshold_intensity_image);
        cv::imwrite("marker_detection_" + std::to_string(i) + ".png", marked_image);

        i++;
    }

    // std::cout << "Press 'q' to exit visualization..." << std::endl;
    // viewer->spin();

    return 0;
}
