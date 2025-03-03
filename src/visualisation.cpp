#include "visualisation.hpp"
#include <cmath>
#include <memory>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

std::tuple<std::shared_ptr<pcl::visualization::PCLVisualizer>, Viewports> create_visualizer() {
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3d view"));

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

void add_raw_pointcloud(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    int viewport_id
) {

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> raw_pointcloud(cloud, "intensity");
    viewer->addPointCloud(cloud, raw_pointcloud, "raw point cloud", viewport_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.25, "raw point cloud");
}

void add_feature_pointcloud(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const Viewports &viewports
) {

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rgb(cloud, 0, 255, 0); // feature cloud (green)

    viewer->addPointCloud(cloud, rgb, "intensity_feature_v2", viewports.v2);
    viewer->addPointCloud(cloud, rgb, "intensity_feature_v3", viewports.v3);
    viewer->addPointCloud(cloud, rgb, "intensity_feature_v4", viewports.v4);
    viewer->addPointCloud(cloud, rgb, "intensity_feature_v5", viewports.v5);
    viewer->addPointCloud(cloud, rgb, "intensity_feature_v6", viewports.v6);
}

void add_oriented_bounding_box(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const Eigen::Vector3f &position,
    const Eigen::Quaternionf &orientation,
    float width,
    float height,
    float depth,
    const std::string &id,
    int viewport_id
) {

    viewer->addCube(position, orientation, width, height, depth, id, viewport_id);
    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        id
    );
}

void add_points_in_box(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const std::string &id,
    int viewport_id
) {

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fieldColor(cloud, "intensity");
    viewer->addPointCloud(cloud, fieldColor, id, viewport_id);
}

void add_marker_vertices(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::vector<std::vector<float>> &vertices,
    int viewport_id
) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr vertex_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &vertex_cloud = *vertex_cloud_ptr;

    vertex_cloud.width = vertices.size();
    vertex_cloud.height = 1;
    vertex_cloud.is_dense = false;
    vertex_cloud.points.resize(vertex_cloud.width * vertex_cloud.height);

    for (size_t p = 0; p < vertices.size(); ++p) {
        vertex_cloud.points[p].x = vertices[p][0];
        vertex_cloud.points[p].y = vertices[p][1];
        vertex_cloud.points[p].z = vertices[p][2];
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(vertex_cloud_ptr, 255, 0, 0); // red vertices
    viewer->addPointCloud(vertex_cloud_ptr, rgb, "vertex_cloud", viewport_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "vertex_cloud");
}
