#pragma once

#include <opencv2/opencv.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <string>

#include "pointcloud.hpp"

/**
 * @brief Create a range image from a point cloud
 *
 * @param cloud Input point cloud
 * @param angular_resolution_x Angular resolution in x direction (in degrees)
 * @param angular_resolution_y Angular resolution in y direction (in degrees)
 * @return pcl::RangeImage::Ptr The created range image
 */
pcl::RangeImage::Ptr
create_range_image(const PointCloud::Ptr &cloud, float angular_resolution_x = 0.5f, float angular_resolution_y = 0.5f);

/**
 * @brief Create range images for multiple point clouds
 *
 * @param clouds Vector of point clouds
 * @param angular_resolution_x Angular resolution in x direction (in degrees)
 * @param angular_resolution_y Angular resolution in y direction (in degrees)
 * @return std::vector<pcl::RangeImage::Ptr> Vector of created range images
 */
std::vector<pcl::RangeImage::Ptr> create_range_images(
    const std::vector<PointCloud::Ptr> &clouds, float angular_resolution_x = 0.5f, float angular_resolution_y = 0.5f
);

/**
 * @brief Convert a PCL range image to an OpenCV matrix
 *
 * @param range_image Input range image
 * @return cv::Mat OpenCV matrix representation of the range image
 */
cv::Mat range_image_to_opencv(const pcl::RangeImage::Ptr &range_image);

/**
 * @brief Save a range image as a PNG file
 *
 * @param range_image Input range image
 * @param filename Output filename
 * @return bool True if successful, false otherwise
 */
bool save_range_image(const pcl::RangeImage::Ptr &range_image, const std::string &filename);

/**
 * @brief Save multiple range images as PNG files
 *
 * @param range_images Vector of range images
 * @param base_filename Base filename (will be appended with index)
 * @return std::vector<std::string> Vector of saved filenames
 */
std::vector<std::string> save_range_images(
    const std::vector<pcl::RangeImage::Ptr> &range_images, const std::string &base_filename = "range_image_"
);
