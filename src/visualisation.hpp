#pragma once

#include "oriented_bounding_box.hpp"
#include "pointcloud.hpp"
#include <memory>
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

/**
 * @brief Add an oriented bounding box to the visualizer
 *
 * @param viewer PCL visualizer
 * @param obb Oriented bounding box to visualize
 * @param id Unique identifier for the box
 * @param viewport_id Viewport to add the box to
 * @param r Red color component (0-1)
 * @param g Green color component (0-1)
 * @param b Blue color component (0-1)
 */
void add_oriented_bounding_box(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const OrientedBoundingBox &obb,
    const std::string &id,
    int viewport_id,
    double r = 0.0,
    double g = 1.0,
    double b = 0.0
);

/**
 * @brief Visualize oriented bounding boxes
 *
 * @param viewer PCL visualizer
 * @param boxes Vector of oriented bounding boxes
 * @param viewport_id Viewport to add the visualization to
 * @param r Red color component (0-1)
 * @param g Green color component (0-1)
 * @param b Blue color component (0-1)
 */
void visualize_oriented_bounding_boxes(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::vector<OrientedBoundingBox> &boxes,
    int viewport_id,
    double r = 1.0,
    double g = 1.0,
    double b = 1.0
);
