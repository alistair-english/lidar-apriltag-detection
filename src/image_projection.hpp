#pragma once

#include "oriented_bounding_box.hpp"
#include "pointcloud.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>

PointCloud::Ptr transform_cloud_for_imaging(const PointCloud::Ptr &cloud, const OrientedBoundingBox &obb);

pcl::RangeImage::Ptr create_range_image_from_cloud(const PointCloud::Ptr &cloud, float angular_resolution);

cv::Mat create_intensity_image_from_cloud(
    const PointCloud::Ptr &cloud,
    float angular_resolution,
    const pcl::RangeImage::Ptr &pre_computed_range_image = nullptr
);
