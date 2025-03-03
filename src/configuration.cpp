#include "configuration.hpp"
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

Configuration load_configuration(const std::string &config_file) {
    YAML::Node conf = YAML::LoadFile(config_file);

    Configuration config;

    // Load basic configuration values
    config.filename = conf["filename"].as<std::string>();
    config.tag_family = conf["tag_family"].as<std::string>();
    config.marker_size = conf["marker_size"].as<float>();
    config.tolerance = conf["tolerance"].as<float>();
    config.ratio_threshold = conf["ratio_threshold"].as<float>();
    config.buffer = conf["buffer"].as<float>();
    config.intensity_threshold = conf["intensity_threshold"].as<float>();
    config.EuclideanTolerance = conf["EuclideanTolerance"].as<float>();
    config.ang_resolution = conf["ang_resolution"].as<float>();
    config.binary_threshold = conf["binary_threshold"].as<float>();
    config.norm_radius = conf["norm_radius"].as<float>();
    config.gradient_radius = conf["gradient_radius"].as<float>();

    // Calculate derived values
    config.cuboid_dia_min = sqrt(2 * config.marker_size * config.marker_size);
    config.cuboid_dia_max = sqrt(4 * config.marker_size * config.marker_size + config.tolerance * config.tolerance);

    config.cuboid_area_min = sqrt(config.marker_size * config.marker_size);
    config.cuboid_area_max = sqrt(2 * config.marker_size * config.marker_size + config.tolerance * config.tolerance);

    return config;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr load_pointcloud(const std::string &filename) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    if (pcl::io::loadPCDFile(filename, *cloud) == -1) {
        PCL_ERROR("Failed to read point cloud file");
        return nullptr;
    }

    return cloud;
}
