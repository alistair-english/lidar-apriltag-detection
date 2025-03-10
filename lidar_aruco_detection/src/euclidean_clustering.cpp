#include "lidar_aruco_detection/euclidean_clustering.hpp"
#include "lidar_aruco_detection/pointcloud.hpp"

#include <memory>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace lidar_aruco_detection {

std::vector<PointCloud::Ptr> extract_euclidean_clusters(
    const PointCloud::Ptr &cloud, float cluster_tolerance, int min_cluster_size, int max_cluster_size
) {

    std::vector<PointCloud::Ptr> clusters;

    // Create KdTree for searching
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(cloud);

    // Perform Euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    // Extract each cluster as a separate point cloud
    for (const auto &indices : cluster_indices) {
        auto cluster = std::make_shared<PointCloud>();
        for (const auto &idx : indices.indices) {
            cluster->push_back((*cloud)[idx]);
        }
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;

        clusters.push_back(cluster);
    }

    return clusters;
}

} // namespace lidar_aruco_detection
