#pragma once

#include "oriented_bounding_box.hpp"
#include "pointcloud.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

/**
 * @brief Transform a point cloud for range image creation
 *
 * @param cloud Input point cloud
 * @param obb Oriented bounding box of the point cloud (used to position the camera)
 * @return PointCloud::Ptr Transformed point cloud ready for range image creation
 */
PointCloud::Ptr transform_cloud_for_imaging(const PointCloud::Ptr &cloud, const OrientedBoundingBox &obb);

/**
 * @brief Create a range image from a point cloud using the oriented bounding box to position the camera
 *
 * @param cloud Input point cloud
 * @param obb Oriented bounding box of the point cloud (used to position the camera)
 * @param angular_resolution Angular resolution of the range image in degrees
 * @return pcl::RangeImage::Ptr The created range image
 */
pcl::RangeImage::Ptr create_range_image_from_cloud(const PointCloud::Ptr &cloud, float angular_resolution = 0.5f);

/**
 * @brief Create range images from multiple point clouds
 *
 * @param clouds Vector of point clouds
 * @param boxes Vector of oriented bounding boxes corresponding to the clouds
 * @param angular_resolution Angular resolution of the range image in degrees
 * @param max_angle_width Maximum horizontal view angle in degrees
 * @param max_angle_height Maximum vertical view angle in degrees
 * @return std::vector<pcl::RangeImage::Ptr> Vector of range images
 */
std::vector<std::tuple<pcl::RangeImage::Ptr, pcl::RangeImage::Ptr>> create_range_images(
    const std::vector<PointCloud::Ptr> &clouds,
    const std::vector<OrientedBoundingBox> &boxes,
    float angular_resolution = 0.5f
);

/**
 * @brief Create a new point cloud with points scaled by their intensity values
 *
 * @param cloud Input point cloud
 * @return PointCloud::Ptr New point cloud with points scaled by intensity
 */
PointCloud::Ptr create_intensity_scaled_cloud(const PointCloud::Ptr &cloud);

/**
 * @brief Convert a PCL range image to an OpenCV Mat with normalized values and color mapping
 *
 * @param range_image Input range image
 * @return cv::Mat Colored OpenCV image
 */
cv::Mat convert_range_image_to_cv_mat(const pcl::RangeImage::Ptr &range_image);
