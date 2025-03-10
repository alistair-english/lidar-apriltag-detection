#pragma once

#include "oriented_bounding_box.hpp"
#include "pointcloud.hpp"
#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tuple>
#include <vector>

namespace lidar_aruco_detection {

struct Viewports {
    int v1;
    int v2;
    int v3;
    int v4;
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

void visualize_3d_points(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::vector<Eigen::Vector3f> &points,
    const std::string &id,
    int viewport_id,
    double r = 1.0,
    double g = 0.0,
    double b = 0.0,
    double point_size = 5.0
);

/**
 * Visualize a marker pose (position and orientation)
 *
 * @param viewer PCL visualizer
 * @param position Marker center position
 * @param orientation Marker orientation as quaternion
 * @param id Unique identifier for the visualization
 * @param viewport_id Viewport to add the visualization to
 * @param scale Scale of the coordinate system
 */
void visualize_marker_pose(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const Eigen::Vector3f &position,
    const Eigen::Quaternionf &orientation,
    const std::string &id,
    int viewport_id,
    double scale = 0.1
);

} // namespace lidar_aruco_detection