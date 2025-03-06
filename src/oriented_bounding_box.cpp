#include "oriented_bounding_box.hpp"
#include "pointcloud.hpp"
#include <algorithm>
#include <memory>
#include <pcl/common/transforms.h>

OrientedBoundingBox calculate_oriented_bounding_box(const PointCloud::Ptr &cloud) {
    OrientedBoundingBox obb;

    // Check if cloud is empty
    if (cloud->empty()) {
        std::cerr << "Warning: Empty cloud passed to calculate_oriented_bounding_box" << std::endl;
        return obb;
    }

    // Compute the centroid of the cloud
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    // Perform PCA to find principal components
    pcl::PCA<Point> pca;
    pca.setInputCloud(cloud);

    // Get the rotation matrix from PCA
    obb.rotation_matrix = pca.getEigenVectors();

    // Make sure the rotation matrix is orthogonal (sometimes PCA can produce slightly non-orthogonal matrices)
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(obb.rotation_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
    obb.rotation_matrix = svd.matrixU() * svd.matrixV().transpose();

    // Convert rotation matrix to quaternion
    obb.orientation = Eigen::Quaternionf(obb.rotation_matrix);

    // Transform point cloud to align with principal components
    auto transformed_cloud = std::make_shared<PointCloud>();
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = obb.rotation_matrix.transpose();
    transform.block<3, 1>(0, 3) = -obb.rotation_matrix.transpose() * centroid.head<3>();
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // Get min and max points in the transformed coordinate system
    Point min_pt, max_pt;
    pcl::getMinMax3D(*transformed_cloud, min_pt, max_pt);

    // Calculate dimensions
    obb.dimensions[0] = max_pt.x - min_pt.x; // width
    obb.dimensions[1] = max_pt.y - min_pt.y; // height
    obb.dimensions[2] = max_pt.z - min_pt.z; // depth

    // Calculate center position in transformed coordinate system
    Eigen::Vector3f position_transformed(
        (min_pt.x + max_pt.x) / 2.0f, (min_pt.y + max_pt.y) / 2.0f, (min_pt.z + max_pt.z) / 2.0f
    );

    // Transform center back to original coordinate system
    Eigen::Matrix4f inverse_transform = Eigen::Matrix4f::Identity();
    inverse_transform.block<3, 3>(0, 0) = obb.rotation_matrix;
    inverse_transform.block<3, 1>(0, 3) = centroid.head<3>();
    Eigen::Vector4f position_homogeneous(
        position_transformed[0], position_transformed[1], position_transformed[2], 1.0f
    );
    Eigen::Vector4f original_position = inverse_transform * position_homogeneous;
    obb.position = original_position.head<3>();

    return obb;
}

std::vector<OrientedBoundingBox> calculate_oriented_bounding_boxes(const std::vector<PointCloud::Ptr> &clusters) {

    std::vector<OrientedBoundingBox> boxes;
    boxes.reserve(clusters.size());

    for (const auto &cluster : clusters) {
        boxes.push_back(calculate_oriented_bounding_box(cluster));
    }

    return boxes;
}

std::vector<OrientedBoundingBox> filter_obbs(
    const std::vector<OrientedBoundingBox> &boxes, float min_dimension, float max_dimension, float max_aspect_ratio
) {

    std::vector<OrientedBoundingBox> filtered_boxes;

    for (const auto &obb : boxes) {
        // Check minimum and maximum dimension constraints
        float min_dim = std::min({obb.dimensions[0], obb.dimensions[1], obb.dimensions[2]});
        float max_dim = std::max({obb.dimensions[0], obb.dimensions[1], obb.dimensions[2]});

        // Calculate aspect ratio
        float aspect_ratio = max_dim / (min_dim > 0 ? min_dim : 0.001f); // Avoid division by zero

        // Apply filters
        bool passes_min_dim = min_dim >= min_dimension;
        bool passes_max_dim = max_dim <= max_dimension;
        bool passes_aspect_ratio = aspect_ratio <= max_aspect_ratio;

        if (passes_min_dim && passes_max_dim && passes_aspect_ratio) {
            filtered_boxes.push_back(obb);
        }
    }

    return filtered_boxes;
}
