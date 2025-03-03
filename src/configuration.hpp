#pragma once

#include "yaml-cpp/yaml.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <tuple>

struct Configuration {
    std::string filename;
    std::string tag_family;
    float marker_size;
    float tolerance;
    float ratio_threshold;
    float buffer;
    float intensity_threshold;
    float EuclideanTolerance;
    float ang_resolution;
    float binary_threshold;
    float norm_radius;
    float gradient_radius;

    // Derived values
    float cuboid_dia_min;
    float cuboid_dia_max;
    float cuboid_area_min;
    float cuboid_area_max;
};

// Struct to hold viewport IDs
struct Viewports {
    int v1;
    int v2;
    int v3;
    int v4;
    int v5;
    int v6;
};

// Load configuration from a YAML file
Configuration load_configuration(const std::string &config_file = "config.yaml");

// Load point cloud from a file
pcl::PointCloud<pcl::PointXYZI>::Ptr load_pointcloud(const std::string &filename);

// Create and configure PCL visualizer with multiple viewports
std::tuple<boost::shared_ptr<pcl::visualization::PCLVisualizer>, Viewports> create_visualizer();
