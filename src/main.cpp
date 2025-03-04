#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <queue>
#include <vector>

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;
using NormalCloud = pcl::PointCloud<pcl::Normal>;
using GradientCloud = pcl::PointCloud<pcl::IntensityGradient>;

PointCloud::Ptr load_cloud(const std::string &filename) {
    auto cloud = std::make_shared<PointCloud>();

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename.c_str(), *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", filename.c_str());
        return nullptr;
    }

    return cloud;
}

NormalCloud::Ptr estimate_normals(const PointCloud::Ptr &cloud, float radius = 0.01) {
    auto normals = std::make_shared<NormalCloud>();

    // Create normal estimation object
    pcl::NormalEstimation<Point, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(cloud);

    // Create KdTree for searching
    auto tree = std::make_shared<pcl::search::KdTree<Point>>();
    normal_estimator.setSearchMethod(tree);

    // Use radius for neighbor search
    normal_estimator.setRadiusSearch(radius);

    // Compute the normals
    normal_estimator.compute(*normals);

    return normals;
}

GradientCloud::Ptr
estimate_intensity_gradient(const PointCloud::Ptr &cloud, const NormalCloud::Ptr &normals, float radius = 0.008) {
    auto gradients = std::make_shared<GradientCloud>();

    // Create intensity gradient estimation object
    pcl::IntensityGradientEstimation<Point, pcl::Normal, pcl::IntensityGradient> grad_est;
    grad_est.setInputCloud(cloud);
    grad_est.setInputNormals(normals);

    // Create KdTree for searching
    auto tree = std::make_shared<pcl::search::KdTree<Point>>();
    grad_est.setSearchMethod(tree);

    // Use radius for neighbor search
    grad_est.setRadiusSearch(radius);

    // Compute the intensity gradients
    grad_est.compute(*gradients);

    return gradients;
}

// Calculate the magnitude of a gradient
float calculate_gradient_magnitude(const pcl::IntensityGradient &gradient) {
    const float *g = gradient.gradient;
    return std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
}

// Calculate a dynamic threshold for significant intensity gradients
float calculate_intensity_threshold(const GradientCloud::Ptr &gradients, float percentile = 0.9) {
    // Collect all valid gradient magnitudes
    std::vector<float> magnitudes;
    magnitudes.reserve(gradients->size());

    for (const auto &gradient : *gradients) {
        float magnitude = calculate_gradient_magnitude(gradient);
        if (!std::isnan(magnitude)) {
            magnitudes.push_back(magnitude);
        }
    }

    if (magnitudes.empty()) {
        return 0.0f;
    }

    // Sort to find the percentile value
    const size_t threshold_idx = static_cast<size_t>(percentile * magnitudes.size());

    // std::nth_element is an efficient partial sorting algorithm that rearranges the elements so that the element at
    // the specified position is the one that would be there if the range was fully sorted
    std::nth_element(magnitudes.begin(), magnitudes.begin() + threshold_idx, magnitudes.end());

    return magnitudes[threshold_idx];
}

// Extract points with significant gradient magnitudes
PointCloud::Ptr extract_significant_gradient_points(
    const PointCloud::Ptr &cloud, const GradientCloud::Ptr &gradients, float threshold
) {

    auto significant_points = std::make_shared<PointCloud>();

    // Only keep points where the gradient magnitude exceeds the threshold
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (i < gradients->size()) { // Safety check
            float magnitude = calculate_gradient_magnitude(gradients->points[i]);
            if (!std::isnan(magnitude) && magnitude > threshold) {
                significant_points->push_back(cloud->points[i]);
            }
        }
    }

    // Set the width and height for the new point cloud
    significant_points->width = significant_points->size();
    significant_points->height = 1;
    significant_points->is_dense = false;

    return significant_points;
}

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
