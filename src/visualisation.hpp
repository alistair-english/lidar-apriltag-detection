#pragma once

#include "oriented_bounding_box.hpp"
#include "pointcloud.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tuple>
#include <vector>

struct Viewports {
    int v1;
    int v2;
    int v3;
};

std::tuple<std::shared_ptr<pcl::visualization::PCLVisualizer>, Viewports> create_visualizer();

void add_point_cloud(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const PointCloud::Ptr &cloud,
    const std::string &id,
    int viewport,
    double r = 1.0,
    double g = 1.0,
    double b = 1.0,
    double point_size = 1.0
);

void add_point_cloud_intensity(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const PointCloud::Ptr &cloud,
    const std::string &id,
    int viewport,
    double point_size = 1.0
);

void visualize_clusters(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::vector<PointCloud::Ptr> &clusters,
    int viewport_id
);

void add_oriented_bounding_box(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const OrientedBoundingBox &obb,
    const std::string &id,
    int viewport_id,
    double r = 0.0,
    double g = 1.0,
    double b = 0.0
);

void visualize_oriented_bounding_boxes(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::vector<OrientedBoundingBox> &boxes,
    int viewport_id,
    double r = 1.0,
    double g = 1.0,
    double b = 1.0
);

bool save_range_image_as_png(
    const pcl::RangeImage::Ptr &range_image,
    const std::string &filename,
    float min_range = -1.0f,
    float max_range = -1.0f
);

cv::Mat convert_range_image_to_cv_mat(const pcl::RangeImage::Ptr &range_image);
