#pragma once

#include "pointcloud.hpp"
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

/**
 * @brief Structure to represent an oriented bounding box
 */
struct OrientedBoundingBox {
    Eigen::Vector3f position;        // Center position
    Eigen::Vector3f dimensions;      // Width, height, depth
    Eigen::Quaternionf orientation;  // Orientation as quaternion
    Eigen::Matrix3f rotation_matrix; // Rotation matrix
};

/**
 * @brief Calculate the oriented bounding box for a point cloud
 *
 * @param cloud Input point cloud
 * @return OrientedBoundingBox The calculated oriented bounding box
 */
OrientedBoundingBox calculate_oriented_bounding_box(const PointCloud::Ptr &cloud);

/**
 * @brief Calculate oriented bounding boxes for a vector of clusters
 *
 * @param clusters Vector of point cloud clusters
 * @return std::vector<OrientedBoundingBox> Vector of oriented bounding boxes
 */
std::vector<OrientedBoundingBox> calculate_oriented_bounding_boxes(const std::vector<PointCloud::Ptr> &clusters);

/**
 * @brief Filter oriented bounding boxes based on their properties
 *
 * @param boxes Vector of oriented bounding boxes
 * @param min_diagonal Minimum diagonal length threshold
 * @param max_diagonal Maximum diagonal length threshold
 * @param max_aspect_ratio Maximum aspect ratio between largest and smallest dimension
 * @return std::vector<OrientedBoundingBox> Filtered bounding boxes
 */
std::vector<OrientedBoundingBox> filter_obbs(
    const std::vector<OrientedBoundingBox> &boxes,
    float min_diagonal = 0.02,
    float max_diagonal = 1.0,
    float max_aspect_ratio = 10.0
);
