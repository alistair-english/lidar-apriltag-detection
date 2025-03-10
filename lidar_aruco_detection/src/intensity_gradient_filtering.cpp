#include "lidar_aruco_detection/intensity_gradient_filtering.hpp"

namespace lidar_aruco_detection {

NormalCloud::Ptr estimate_normals(const PointCloud::Ptr &cloud, float radius) {
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
estimate_intensity_gradient(const PointCloud::Ptr &cloud, const NormalCloud::Ptr &normals, float radius) {
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

float calculate_intensity_threshold(const GradientCloud::Ptr &gradients, float percentile) {
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

} // namespace lidar_aruco_detection
