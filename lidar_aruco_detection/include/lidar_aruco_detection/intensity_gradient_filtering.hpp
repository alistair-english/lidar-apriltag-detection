#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <vector>

#include "pointcloud.hpp"

namespace lidar_aruco_detection {

using NormalCloud = pcl::PointCloud<pcl::Normal>;
using GradientCloud = pcl::PointCloud<pcl::IntensityGradient>;

NormalCloud::Ptr estimate_normals(const PointCloud::Ptr &cloud, float radius = 0.01);

GradientCloud::Ptr
estimate_intensity_gradient(const PointCloud::Ptr &cloud, const NormalCloud::Ptr &normals, float radius = 0.008);

// Calculate the magnitude of a gradient
inline float calculate_gradient_magnitude(const pcl::IntensityGradient &gradient) {
    const float *g = gradient.gradient;
    return std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
}

// Calculate a dynamic threshold for significant intensity gradients
float calculate_intensity_threshold(const GradientCloud::Ptr &gradients, float percentile = 0.9);

// Extract points with significant gradient magnitudes
PointCloud::Ptr
extract_significant_gradient_points(const PointCloud::Ptr &cloud, const GradientCloud::Ptr &gradients, float threshold);

} // namespace lidar_aruco_detection
