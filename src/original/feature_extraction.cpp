#include "feature_extraction.hpp"

#include <cmath>
#include <memory>
#include <queue>
#include <vector>

#include <pcl/segmentation/extract_clusters.h>

pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float radius) {

    auto cloud_normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> norm_est;

    norm_est.setInputCloud(cloud);

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>(false);
    norm_est.setSearchMethod(tree);
    norm_est.setRadiusSearch(radius);

    norm_est.compute(*cloud_normals);

    return cloud_normals;
}

pcl::PointCloud<pcl::IntensityGradient>::Ptr estimate_intensity_gradients(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, float radius
) {

    auto gradient = std::make_shared<pcl::PointCloud<pcl::IntensityGradient>>();
    pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> gradient_est;

    gradient_est.setInputCloud(cloud);
    gradient_est.setInputNormals(normals);

    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>(false);
    gradient_est.setSearchMethod(tree);
    gradient_est.setRadiusSearch(radius);

    gradient_est.compute(*gradient);

    return gradient;
}

float calculate_intensity_threshold(const pcl::PointCloud<pcl::IntensityGradient>::Ptr &gradient, float ratio_q) {
    std::priority_queue<float> q;

    // Calculate gradient magnitudes and add to priority queue
    for (size_t i = 0; i < gradient->size(); ++i) {
        const float *g_est = (*gradient)[i].gradient;
        float magnitude = std::sqrt(g_est[0] * g_est[0] + g_est[1] * g_est[1] + g_est[2] * g_est[2]);
        if (!std::isnan(magnitude)) {
            q.push(magnitude);
        }
    }

    // Pop top elements based on ratio
    for (int i = 0; i < q.size() * ratio_q; i++) {
        q.pop();
    }

    // Return threshold
    return q.empty() ? 0.0f : q.top();
}

std::vector<pcl::PointIndices> extract_euclidean_clusters(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    float cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size
) {
    // Create KdTree for searching
    auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
    tree->setInputCloud(cloud);

    // Initialize cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;

    // Set parameters
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);

    // Extract clusters
    ec.extract(cluster_indices);

    return cluster_indices;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr extract_intensity_feature_points(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    const pcl::PointCloud<pcl::IntensityGradient>::Ptr &gradient,
    float intensity_threshold
) {

    std::vector<std::vector<float>> valid_points;
    std::vector<float> valid_point;
    int counter = 0;

    // Extract points with gradient magnitude above threshold
    for (size_t p = 0; p < cloud->size(); ++p) {
        const float *g_est = (*gradient)[p].gradient;
        float magnitude = std::sqrt(g_est[0] * g_est[0] + g_est[1] * g_est[1] + g_est[2] * g_est[2]);

        if (!std::isnan(magnitude) && magnitude > intensity_threshold) {
            ++counter;
            valid_point = {cloud->points[p].x, cloud->points[p].y, cloud->points[p].z, cloud->points[p].intensity};
            valid_points.push_back(valid_point);
        }
    }

    // Create point cloud from valid points
    auto intensity_feature = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZI> &intensity_feature_pointcloud = *intensity_feature;

    intensity_feature_pointcloud.width = counter;
    intensity_feature_pointcloud.height = 1;
    intensity_feature_pointcloud.is_dense = false;
    intensity_feature_pointcloud.points.resize(
        intensity_feature_pointcloud.width * intensity_feature_pointcloud.height
    );

    for (size_t q = 0; q < counter; ++q) {
        intensity_feature_pointcloud.points[q].x = valid_points[q][0];
        intensity_feature_pointcloud.points[q].y = valid_points[q][1];
        intensity_feature_pointcloud.points[q].z = valid_points[q][2];
        intensity_feature_pointcloud.points[q].intensity = valid_points[q][3];
    }

    return intensity_feature;
}
