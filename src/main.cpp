#include <iostream>

#include <pcl/common/common.h>

#include "intensity_gradient_filtering.hpp"
#include "pointcloud.hpp"
#include "visualisation.hpp"

void printUsage(const char *programName) {
    std::cout << "Usage: " << programName << " <pcd_file_path> [scale]" << std::endl;
    std::cout << "  pcd_file_path: Path to the PCD file to process" << std::endl;
    std::cout << "  scale: Optional scaling factor for coordinates (default: 1.0)" << std::endl;
}

void print_cloud_bounds(const PointCloud::Ptr &cloud) {
    // Calculate and print the bounds of the point cloud
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    std::cout << "Point cloud bounds:" << std::endl;
    std::cout << "  X: [" << min_pt[0] << ", " << max_pt[0] << "]" << std::endl;
    std::cout << "  Y: [" << min_pt[1] << ", " << max_pt[1] << "]" << std::endl;
    std::cout << "  Z: [" << min_pt[2] << ", " << max_pt[2] << "]" << std::endl;
}

// Parse command line arguments
struct ProgramArgs {
    std::string pcd_file_path;
    float scale = 1.0f;
    bool valid = false;
};

ProgramArgs parse_arguments(int argc, char **argv) {
    ProgramArgs args;

    // Check if a file path was provided
    if (argc < 2 || argc > 3) {
        printUsage(argv[0]);
        return args;
    }

    args.pcd_file_path = argv[1];

    // Parse scale if provided
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
    // Parse command line arguments
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

    // Print the bounds of the loaded point cloud
    print_cloud_bounds(cloud);

    // Estimate surface normals
    const auto normals = estimate_normals(cloud);
    std::cout << "Estimated " << normals->size() << " surface normals." << std::endl;

    // Estimate intensity gradients
    const auto gradients = estimate_intensity_gradient(cloud, normals);
    std::cout << "Estimated " << gradients->size() << " intensity gradients." << std::endl;

    // Calculate threshold for significant gradients
    float threshold = calculate_intensity_threshold(gradients);
    std::cout << "Calculated intensity gradient threshold: " << threshold << std::endl;

    // Extract points with significant gradient magnitudes
    auto significant_points = extract_significant_gradient_points(cloud, gradients, threshold);
    std::cout << "Extracted " << significant_points->size() << " points with significant gradient magnitudes"
              << " (threshold: " << threshold << ")" << std::endl;

    // Visualize the original and filtered point clouds
    visualize_point_clouds(cloud, significant_points);

    return 0;
}
