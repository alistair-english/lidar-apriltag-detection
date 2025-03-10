#pragma once

#include "pointcloud.hpp"
#include <vector>

namespace lidar_aruco_detection {

/**
 * @brief Extract Euclidean clusters from a point cloud
 *
 * @param cloud Input point cloud
 * @param cluster_tolerance Distance threshold for clustering
 * @param min_cluster_size Minimum number of points in a cluster
 * @param max_cluster_size Maximum number of points in a cluster
 * @return std::vector<PointCloud::Ptr> Vector of point clouds, each representing a cluster
 */
std::vector<PointCloud::Ptr> extract_euclidean_clusters(
    const PointCloud::Ptr &cloud,
    float cluster_tolerance = 0.02,
    int min_cluster_size = 100,
    int max_cluster_size = 25000
);

} // namespace lidar_aruco_detection
