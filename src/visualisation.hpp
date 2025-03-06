#pragma once

#include "pointcloud.hpp"
#include <memory>
#include <pcl/visualization/pcl_visualizer.h>
#include <tuple>
#include <vector>

struct Viewports {
    int v1;
    int v2;
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

/**
 * @brief Visualize clusters with different colors
 *
 * @param viewer PCL visualizer
 * @param clusters Vector of point cloud clusters
 * @param viewport_id Viewport ID for visualization
 */
void visualize_clusters(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::vector<PointCloud::Ptr> &clusters,
    int viewport_id
);
