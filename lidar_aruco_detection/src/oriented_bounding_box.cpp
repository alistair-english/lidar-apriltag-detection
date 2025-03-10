#include "lidar_aruco_detection/oriented_bounding_box.hpp"
#include "lidar_aruco_detection/pointcloud.hpp"
#include <algorithm>
#include <memory>
#include <pcl/common/transforms.h>

namespace lidar_aruco_detection {

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
    const std::vector<OrientedBoundingBox> &boxes, float min_diagonal, float max_diagonal, float max_aspect_ratio
) {
    std::vector<OrientedBoundingBox> filtered_boxes;

    for (const auto &obb : boxes) {
        // Calculate diagonal length of the bounding box
        float diagonal = std::sqrt(
            obb.dimensions[0] * obb.dimensions[0] + obb.dimensions[1] * obb.dimensions[1] +
            obb.dimensions[2] * obb.dimensions[2]
        );

        // Sort dimensions to find the two largest
        std::array<float, 3> dims = {obb.dimensions[0], obb.dimensions[1], obb.dimensions[2]};
        std::sort(dims.begin(), dims.end());

        // Calculate aspect ratio using the two largest dimensions
        float larger_dim = dims[2]; // Largest dimension
        float middle_dim = dims[1]; // Second largest dimension

        // Calculate aspect ratio
        float aspect_ratio = larger_dim / (middle_dim > 0 ? middle_dim : 0.001f); // Avoid division by zero

        // Apply filters
        bool passes_min_diagonal = diagonal >= min_diagonal;
        bool passes_max_diagonal = diagonal <= max_diagonal;
        bool passes_aspect_ratio = aspect_ratio <= max_aspect_ratio;

        if (passes_min_diagonal && passes_max_diagonal && passes_aspect_ratio) {
            filtered_boxes.push_back(obb);
        }
    }

    return filtered_boxes;
}

PointCloud::Ptr extract_points_in_obb(const PointCloud::Ptr &cloud, const OrientedBoundingBox &obb) {
    auto result = std::make_shared<PointCloud>();

    if (cloud->empty()) {
        return result;
    }

    // Create transformation matrix to transform points to OBB's local coordinate system
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = obb.rotation_matrix.transpose();
    transform.block<3, 1>(0, 3) = -obb.rotation_matrix.transpose() * obb.position;

    // Half dimensions of the box
    Eigen::Vector3f half_dimensions = obb.dimensions * 0.5f;

    // Check each point
    for (const auto &point : *cloud) {
        // Transform point to OBB's local coordinate system
        Eigen::Vector4f pt(point.x, point.y, point.z, 1.0f);
        Eigen::Vector4f transformed_pt = transform * pt;

        // Check if point is inside the box
        if (std::abs(transformed_pt[0]) <= half_dimensions[0] && std::abs(transformed_pt[1]) <= half_dimensions[1] &&
            std::abs(transformed_pt[2]) <= half_dimensions[2]) {
            result->push_back(point);
        }
    }

    return result;
}

} // namespace lidar_aruco_detection
