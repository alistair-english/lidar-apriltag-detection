#pragma once

#include "pointcloud.hpp"
#include <memory>
#include <pcl/visualization/pcl_visualizer.h>

// Create a PCL visualizer with two viewports
std::shared_ptr<pcl::visualization::PCLVisualizer> create_visualizer();

// Add point cloud to the visualizer with custom color
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

// Add point cloud to the visualizer with intensity-based coloring
void add_point_cloud_intensity(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const PointCloud::Ptr &cloud,
    const std::string &id,
    int viewport,
    double point_size = 1.0
);

// Visualize original and filtered point clouds
void visualize_point_clouds(const PointCloud::Ptr &original_cloud, const PointCloud::Ptr &filtered_cloud);
