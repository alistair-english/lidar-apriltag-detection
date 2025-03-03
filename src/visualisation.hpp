#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tuple>

// Struct to hold viewport IDs
struct Viewports {
    int v1;
    int v2;
    int v3;
    int v4;
    int v5;
    int v6;
};

// Create and configure PCL visualizer with multiple viewports
std::tuple<std::shared_ptr<pcl::visualization::PCLVisualizer>, Viewports> create_visualizer();

// Add raw point cloud to visualizer
void add_raw_pointcloud(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    int viewport_id
);

// Add feature point cloud to multiple viewports
void add_feature_pointcloud(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const Viewports &viewports
);

// Add oriented bounding box to visualizer
void add_oriented_bounding_box(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const Eigen::Vector3f &position,
    const Eigen::Quaternionf &orientation,
    float width,
    float height,
    float depth,
    const std::string &id,
    int viewport_id
);

// Add points in box to visualizer
void add_points_in_box(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const std::string &id,
    int viewport_id
);

// Add marker vertices to visualizer
void add_marker_vertices(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::vector<std::vector<float>> &vertices,
    int viewport_id
);
