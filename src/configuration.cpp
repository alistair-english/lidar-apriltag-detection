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
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile(filename, *cloud) == -1) {
        PCL_ERROR("Failed to read point cloud file");
        return nullptr;
    }

    return cloud;
}

std::tuple<boost::shared_ptr<pcl::visualization::PCLVisualizer>, Viewports> create_visualizer() {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3d view"));

    Viewports viewports = {0, 0, 0, 0, 0, 0};

    viewer->createViewPort(0.0, 0.5, 0.3333, 1.0, viewports.v1);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewports.v1);

    viewer->createViewPort(0.3333, 0.5, 0.6666, 1.0, viewports.v2);
    viewer->setBackgroundColor(0.2, 0.2, 0.2, viewports.v2);

    viewer->createViewPort(0.6666, 0.5, 1.0, 1.0, viewports.v3);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, viewports.v3);

    viewer->createViewPort(0.0, 0.0, 0.3333, 0.5, viewports.v4);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, viewports.v4);

    viewer->createViewPort(0.3333, 0.0, 0.6666, 0.5, viewports.v5);
    viewer->setBackgroundColor(0.4, 0.4, 0.4, viewports.v5);

    viewer->createViewPort(0.6666, 0.0, 1.0, 0.5, viewports.v6);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewports.v6);

    // Optional: Add text descriptions for each viewport
    /*
    viewer->addText("Input: raw point cloud", 10, 10, "text1", viewports.v1);
    viewer->addText("Step1: intensity feature extraction", 10, 10, "text2", viewports.v2);
    viewer->addText("Step2: feature clustering", 10, 10, "text3", viewports.v3);
    viewer->addText("Step3: filter out impractical candidates", 10, 10, "text4", viewports.v4);
    viewer->addText("Step4: conduct marker detection in ROI", 10, 10, "text5", viewports.v5);
    viewer->addText("Step5: output the vertices of the detected marker", 10, 10, "text6", viewports.v6);
    */

    // viewer->addCoordinateSystem();

    return std::make_tuple(viewer, viewports);
}
