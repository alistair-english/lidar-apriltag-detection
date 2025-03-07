#include <iostream>

#include <pcl/common/common.h>

#include "euclidean_clustering.hpp"
#include "image_projection.hpp"
#include "intensity_gradient_filtering.hpp"
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

    std::cout << "Loading point cloud from: " << args.pcd_file_path << std::endl;
    std::cout << "Using scale factor: " << args.scale << std::endl;

    const auto cloud = load_cloud(args.pcd_file_path, args.scale);
    if (!cloud) {
        std::cerr << "Failed to load point cloud from: " << args.pcd_file_path << std::endl;
        return -1;
    }

    std::cout << "Successfully loaded point cloud with " << cloud->size() << " points." << std::endl;

    print_cloud_bounds(cloud);

    const auto normals = estimate_normals(cloud);
    std::cout << "Estimated " << normals->size() << " surface normals." << std::endl;

    const auto gradients = estimate_intensity_gradient(cloud, normals);
    std::cout << "Estimated " << gradients->size() << " intensity gradients." << std::endl;

    float threshold = calculate_intensity_threshold(gradients);
    std::cout << "Calculated intensity gradient threshold: " << threshold << std::endl;

    auto significant_points = extract_significant_gradient_points(cloud, gradients, threshold);
    std::cout << "Extracted " << significant_points->size() << " points with significant gradient magnitudes"
              << " (threshold: " << threshold << ")" << std::endl;

    auto clusters = extract_euclidean_clusters(significant_points, 0.06, 100, 25000);
    std::cout << "Found " << clusters.size() << " clusters" << std::endl;

    auto boxes = calculate_oriented_bounding_boxes(clusters);
    std::cout << "Calculated " << boxes.size() << " oriented bounding boxes" << std::endl;

    auto filtered_boxes = filter_obbs(boxes, 0.3, 1.0, 1.5);
    std::cout << "After OBB filtering: " << filtered_boxes.size() << " boxes remain" << std::endl;

    auto points_in_boxes = extract_points_in_obbs(cloud, filtered_boxes);

    auto range_images = create_range_images(points_in_boxes);

    // Save range images as PNG files
    std::cout << "Saving range images as PNG files..." << std::endl;
    auto saved_filenames = save_range_images(range_images, "range_image_");
    std::cout << "Saved " << saved_filenames.size() << " range images." << std::endl;

    auto [viewer, viewports] = create_visualizer();
    add_point_cloud_intensity(viewer, cloud, "original", viewports.v1);
    add_point_cloud(viewer, significant_points, "filtered", viewports.v2, 1.0, 0.0, 0.0, 2.0);

    visualize_clusters(viewer, clusters, viewports.v2);
    visualize_oriented_bounding_boxes(viewer, filtered_boxes, viewports.v2);

    for (size_t i = 0; i < points_in_boxes.size(); ++i) {
        std::string id = "box_points_" + std::to_string(i);
        add_point_cloud_intensity(viewer, points_in_boxes[i], id, viewports.v3, 3.0);
    }

    std::cout << "Press 'q' to exit visualization..." << std::endl;
    viewer->spin();

    return 0;
}
