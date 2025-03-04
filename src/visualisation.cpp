#include "visualisation.hpp"
#include <iostream>

// Structure to hold viewport IDs
static struct {
    int v1;
    int v2;
} viewports;

std::shared_ptr<pcl::visualization::PCLVisualizer> create_visualizer() {
    auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");

    // Create two viewports
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, viewports.v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, viewports.v2);

    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewports.v1);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewports.v2);

    viewer->addText("Original Point Cloud", 10, 10, 1.0, 1.0, 1.0, "v1_text", viewports.v1);
    viewer->addText("Filtered Point Cloud", 10, 10, 1.0, 1.0, 1.0, "v2_text", viewports.v2);

    return viewer;
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

void visualize_point_clouds(const PointCloud::Ptr &original_cloud, const PointCloud::Ptr &filtered_cloud) {

    auto viewer = create_visualizer();

    // Add original point cloud to first viewport, colored by intensity
    add_point_cloud_intensity(viewer, original_cloud, "original", viewports.v1);

    // Add filtered point cloud to second viewport
    add_point_cloud(viewer, filtered_cloud, "filtered", viewports.v2, 1.0, 0.0, 0.0, 2.0);

    // Set camera position for both viewports
    viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    viewer->resetCamera();

    // Start visualization
    std::cout << "Press 'q' to exit visualization..." << std::endl;
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
    }
}
