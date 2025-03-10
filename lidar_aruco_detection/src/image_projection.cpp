#include "lidar_aruco_detection/image_projection.hpp"
#include <memory>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <tuple>

#include "lidar_aruco_detection/pointcloud.hpp"

namespace lidar_aruco_detection {

std::tuple<PointCloud::Ptr, Eigen::Affine3f>
transform_cloud_for_imaging(const PointCloud::Ptr &cloud, const OrientedBoundingBox &obb) {
    if (cloud->empty()) {
        std::cerr << "Warning: Empty cloud passed to transform_cloud_for_imaging" << std::endl;
        return {std::make_shared<PointCloud>(), Eigen::Affine3f::Identity()};
    }

    // First transformation: align with the oriented bounding box
    // This brings the cloud to the origin with the OBB's orientation
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Set the rotation part of the transformation to the inverse of the OBB's rotation
    transform.linear() = obb.rotation_matrix.transpose();

    // Set the translation part to move the OBB's center to the origin
    transform.translation() = -transform.linear() * obb.position;

    // Second transformation
    Eigen::Affine3f sensor_transform = Eigen::Affine3f::Identity();

    // Rotate -90 degrees about Y axis
    Eigen::AngleAxisf rotationY(-M_PI / 2, Eigen::Vector3f::UnitY());

    // Rotate 90 degrees about X axis
    Eigen::AngleAxisf rotationX(M_PI / 2, Eigen::Vector3f::UnitX());

    // Combine rotations (apply Y rotation first, then X rotation)
    sensor_transform.rotate(rotationX * rotationY);

    // Translate 1 meter along X axis
    sensor_transform.translation() = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    // Combine both transformations for the complete transform
    Eigen::Affine3f complete_transform = sensor_transform * transform;

    // Apply the combined transformation in a single step
    auto transformed_cloud = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*cloud, *transformed_cloud, complete_transform);

    return {transformed_cloud, complete_transform};
}

pcl::RangeImage::Ptr create_range_image_from_cloud(const PointCloud::Ptr &cloud, float angular_resolution) {
    if (cloud->empty()) {
        std::cerr << "Warning: Empty cloud passed to create_range_images_from_cloud" << std::endl;
        return std::make_shared<pcl::RangeImage>();
    }

    // Calculate min and max points to determine FOV
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    // Calculate angles for FOV based on min/max Y and Z
    float min_y = min_pt[1], max_y = max_pt[1];
    float min_z = min_pt[2], max_z = max_pt[2];
    float min_x = min_pt[0]; // We'll use this for noise filtering

    // Calculate the angular FOV in Y and Z directions
    float y_angle_min = std::atan2(min_y, 1.0f);
    float y_angle_max = std::atan2(max_y, 1.0f);
    float z_angle_min = std::atan2(min_z, 1.0f);
    float z_angle_max = std::atan2(max_z, 1.0f);

    // Find the maximum absolute angle in each direction and double it for safety
    float y_max_abs = std::max(std::abs(y_angle_min), std::abs(y_angle_max));
    float z_max_abs = std::max(std::abs(z_angle_min), std::abs(z_angle_max));

    // Double the maximum angles to ensure we capture everything
    float angle_width = 2.0f * y_max_abs;
    float angle_height = 2.0f * z_max_abs;

    // Add a small buffer to ensure all points are captured
    angle_width += 0.2f;
    angle_height += 0.2f;

    // Create the range image
    auto range_image = std::make_shared<pcl::RangeImage>();
    range_image->createFromPointCloud(
        *cloud,
        angular_resolution,
        angle_width,
        angle_height,
        Eigen::Affine3f::Identity(),
        pcl::RangeImage::LASER_FRAME,
        0.0f // Noise level
    );

    // Check if the range image is empty or has invalid dimensions
    if (range_image->empty() || range_image->width <= 0 || range_image->height <= 0) {
        std::cerr << "Warning: Generated range image is empty or has invalid dimensions: " << range_image->width << "x"
                  << range_image->height << std::endl;
        std::cerr << "Angular resolution: " << pcl::rad2deg(angular_resolution) << " degrees" << std::endl;
        std::cerr << "Angle width: " << pcl::rad2deg(angle_width) << " degrees" << std::endl;
        std::cerr << "Angle height: " << pcl::rad2deg(angle_height) << " degrees" << std::endl;
        return std::make_shared<pcl::RangeImage>();
    }

    return range_image;
}

cv::Mat create_intensity_image_from_cloud(
    const PointCloud::Ptr &cloud, float angular_resolution, const pcl::RangeImage::Ptr &pre_computed_range_image
) {
    if (cloud->empty()) {
        std::cerr << "Warning: Empty cloud passed to create_intensity_image_from_cloud" << std::endl;
        return cv::Mat();
    }

    pcl::RangeImage::Ptr range_image;

    if (pre_computed_range_image != nullptr) {
        range_image = pre_computed_range_image;
    } else {
        range_image = create_range_image_from_cloud(cloud, angular_resolution);
    }

    // Get dimensions
    int width = range_image->width;
    int height = range_image->height;

    // Create an OpenCV matrix for the intensity image
    cv::Mat intensity_image(height, width, CV_8UC1, cv::Scalar(0));

    // Create a mapping from range image points to original cloud points
    std::vector<int> range_image_to_cloud_map(width * height, -1);

    // Build a KD-tree for the original cloud for efficient nearest neighbor search
    pcl::KdTreeFLANN<Point> kdtree;
    kdtree.setInputCloud(cloud);

    // Find min and max intensity for normalization
    float min_intensity = std::numeric_limits<float>::max();
    float max_intensity = std::numeric_limits<float>::lowest();

    // Create a temporary matrix to store intensity values
    std::vector<float> intensity_values(width * height, -1.0f);

    // First pass: map range image points to original cloud points and find min/max intensity
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const pcl::PointWithRange &range_point = range_image->at(x, y);
            if (!std::isfinite(range_point.range))
                continue; // Skip invalid points

            // Convert range image point to 3D point
            Point search_point;
            search_point.x = range_point.x;
            search_point.y = range_point.y;
            search_point.z = range_point.z;

            // Find the closest point in the original cloud
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);

            if (kdtree.nearestKSearch(search_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                int idx = pointIdxNKNSearch[0];
                float intensity = cloud->points[idx].intensity;

                // Store the intensity value
                intensity_values[y * width + x] = intensity;

                if (intensity < min_intensity)
                    min_intensity = intensity;
                if (intensity > max_intensity)
                    max_intensity = intensity;
            }
        }
    }

    // Avoid division by zero
    if (max_intensity == min_intensity) {
        max_intensity = min_intensity + 1.0f;
    }

    // Second pass: fill the intensity image
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float intensity = intensity_values[y * width + x];
            if (intensity < 0)
                continue; // Skip points without a valid mapping

            // Normalize intensity to 0-255 range
            uint8_t intensity_value =
                static_cast<uint8_t>(255.0f * (intensity - min_intensity) / (max_intensity - min_intensity));

            intensity_image.at<uint8_t>(y, x) = intensity_value;
        }
    }

    return intensity_image;
}

std::vector<Eigen::Vector3f> convert_marker_points_to_3d(
    const std::vector<cv::Point2f> &marker_corners,
    const pcl::RangeImage::Ptr &range_image,
    const Eigen::Affine3f &transform
) {
    std::vector<Eigen::Vector3f> points_3d;

    if (range_image->empty() || marker_corners.empty()) {
        std::cerr << "Warning: Empty range image or marker corners in convert_marker_points_to_3d" << std::endl;
        return points_3d;
    }

    // Get range image dimensions
    int width = range_image->width;
    int height = range_image->height;

    // For each corner point
    for (const auto &corner : marker_corners) {
        // Convert to integer coordinates for range image lookup
        int x = static_cast<int>(std::round(corner.x));
        int y = static_cast<int>(std::round(corner.y));

        // Check if the point is within the range image bounds
        if (x >= 0 && x < width && y >= 0 && y < height) {
            // Get the corresponding 3D point from the range image
            const pcl::PointWithRange &range_point = range_image->at(x, y);

            // Check if the range is valid
            if (std::isfinite(range_point.range)) {
                // Create a 3D point in the transformed cloud frame
                Eigen::Vector3f point_transformed(range_point.x, range_point.y, range_point.z);

                // Apply inverse transform to get the point in the original cloud frame
                Eigen::Vector3f point_original = transform.inverse() * point_transformed;

                // Add to the result vector
                points_3d.push_back(point_original);
            } else {
                // If the range is invalid, try to interpolate from neighboring points
                Eigen::Vector3f interpolated_point = Eigen::Vector3f::Zero();
                int valid_neighbors = 0;

                // Check 8 neighboring pixels
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0)
                            continue; // Skip the center point

                        int nx = x + dx;
                        int ny = y + dy;

                        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                            const pcl::PointWithRange &neighbor = range_image->at(nx, ny);
                            if (std::isfinite(neighbor.range)) {
                                interpolated_point += Eigen::Vector3f(neighbor.x, neighbor.y, neighbor.z);
                                valid_neighbors++;
                            }
                        }
                    }
                }

                if (valid_neighbors > 0) {
                    // Average the valid neighbors
                    interpolated_point /= static_cast<float>(valid_neighbors);

                    // Apply inverse transform
                    Eigen::Vector3f point_original = transform.inverse() * interpolated_point;
                    points_3d.push_back(point_original);
                } else {
                    // If no valid neighbors, push a NaN point
                    points_3d.push_back(Eigen::Vector3f(
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::quiet_NaN()
                    ));
                }
            }
        } else {
            // Point is outside the range image bounds
            points_3d.push_back(Eigen::Vector3f(
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN(),
                std::numeric_limits<float>::quiet_NaN()
            ));
        }
    }

    return points_3d;
}

} // namespace lidar_aruco_detection
