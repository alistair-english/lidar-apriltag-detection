#pragma once

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
std::tuple<boost::shared_ptr<pcl::visualization::PCLVisualizer>, Viewports> create_visualizer();
