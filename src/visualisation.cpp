#include "visualisation.hpp"
#include <iostream>
#include <random>

std::tuple<std::shared_ptr<pcl::visualization::PCLVisualizer>, Viewports> create_visualizer() {
    auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");
    Viewports viewports;

    // Create two viewports
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewports.v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, viewports.v2);

    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewports.v1);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewports.v2);

    viewer->addText("Original Point Cloud", 10, 10, 1.0, 1.0, 1.0, "v1_text", viewports.v1);
    viewer->addText("Filtered Point Cloud", 10, 10, 1.0, 1.0, 1.0, "v2_text", viewports.v2);

    return {viewer, viewports};
}

void add_point_cloud(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const PointCloud::Ptr &cloud,
    const std::string &id,
    int viewport,
    double r,
    double g,
    double b,
    double point_size
) {
    pcl::visualization::PointCloudColorHandlerCustom<Point> color_handler(cloud, r * 255, g * 255, b * 255);
    viewer->addPointCloud<Point>(cloud, color_handler, id, viewport);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
}

void add_point_cloud_intensity(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const PointCloud::Ptr &cloud,
    const std::string &id,
    int viewport,
    double point_size
) {
    pcl::visualization::PointCloudColorHandlerGenericField<Point> color_handler(cloud, "intensity");
    viewer->addPointCloud<Point>(cloud, color_handler, id, viewport);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id);
}

void visualize_clusters(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::vector<PointCloud::Ptr> &clusters,
    int viewport_id
) {
    // Random color generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Add each cluster with a different color
    for (size_t i = 0; i < clusters.size(); ++i) {
        // Generate a random color
        double r = dis(gen);
        double g = dis(gen);
        double b = dis(gen);

        // Create a unique ID for each cluster
        std::string id = "cluster_" + std::to_string(i);

        // Add the cluster to the viewer
        viewer->addPointCloud<Point>(clusters[i], id, viewport_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id, viewport_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id, viewport_id);
    }
}
