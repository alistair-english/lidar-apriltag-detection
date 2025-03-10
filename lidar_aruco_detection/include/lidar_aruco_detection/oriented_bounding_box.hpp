#pragma once

#include "pointcloud.hpp"
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>

namespace lidar_aruco_detection {

struct OrientedBoundingBox {
    Eigen::Vector3f position;        // Center position
    Eigen::Vector3f dimensions;      // Width, height, depth
    Eigen::Quaternionf orientation;  // Orientation as quaternion
    Eigen::Matrix3f rotation_matrix; // Rotation matrix
};

OrientedBoundingBox calculate_oriented_bounding_box(const PointCloud::Ptr &cloud);

std::vector<OrientedBoundingBox> calculate_oriented_bounding_boxes(const std::vector<PointCloud::Ptr> &clusters);

std::vector<OrientedBoundingBox> filter_obbs(
    const std::vector<OrientedBoundingBox> &boxes,
    float min_diagonal = 0.02,
    float max_diagonal = 1.0,
    float max_aspect_ratio = 10.0
);

PointCloud::Ptr extract_points_in_obb(const PointCloud::Ptr &cloud, const OrientedBoundingBox &obb);

} // namespace lidar_aruco_detection