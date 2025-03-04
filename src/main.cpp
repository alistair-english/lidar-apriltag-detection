#include <iostream>

#include "intensity_gradient_filtering.hpp"
#include "pointcloud.hpp"

int main(int argc, char **argv) {
    const auto cloud = load_cloud("base.pcd");
    if (!cloud) {
        std::cerr << "Failed to load point cloud." << std::endl;
        return -1;
    }

    std::cout << "Loaded point cloud with " << cloud->size() << " points." << std::endl;

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

    return 0;
}
