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

// Load configuration from a YAML file
Configuration load_configuration(const std::string &config_file = "config.yaml");

// Load point cloud from a file
pcl::PointCloud<pcl::PointXYZI>::Ptr load_pointcloud(const std::string &filename);
