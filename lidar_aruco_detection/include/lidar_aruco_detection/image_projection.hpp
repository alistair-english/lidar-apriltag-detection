#pragma once

#include "oriented_bounding_box.hpp"
#include "pointcloud.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

namespace lidar_aruco_detection {

std::tuple<PointCloud::Ptr, Eigen::Affine3f>
transform_cloud_for_imaging(const PointCloud::Ptr &cloud, const OrientedBoundingBox &obb);

pcl::RangeImage::Ptr create_range_image_from_cloud(const PointCloud::Ptr &cloud, float angular_resolution);

cv::Mat create_intensity_image_from_cloud(
    const PointCloud::Ptr &cloud,
    float angular_resolution,
    const pcl::RangeImage::Ptr &pre_computed_range_image = nullptr
);

// Convert 2D marker corner points to 3D points in the original cloud frame
std::vector<Eigen::Vector3f> convert_marker_points_to_3d(
    const std::vector<cv::Point2f> &marker_corners,
    const pcl::RangeImage::Ptr &range_image,
    const Eigen::Affine3f &transform
);

} // namespace lidar_aruco_detection