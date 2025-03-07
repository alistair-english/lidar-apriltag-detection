#include "image_projection.hpp"
#include <memory>
#include <pcl/common/common.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <tuple>

#include "pointcloud.hpp"

PointCloud::Ptr transform_cloud_for_imaging(const PointCloud::Ptr &cloud, const OrientedBoundingBox &obb) {
    if (cloud->empty()) {
        std::cerr << "Warning: Empty cloud passed to transform_cloud_for_imaging" << std::endl;
        return std::make_shared<PointCloud>();
    }

    // Transform the point cloud to align with the oriented bounding box
    // This brings the cloud to the origin with the OBB's orientation
    auto transformed_cloud = std::make_shared<PointCloud>();
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Set the rotation part of the transformation to the inverse of the OBB's rotation
    transform.linear() = obb.rotation_matrix.transpose();

    // Set the translation part to move the OBB's center to the origin
    transform.translation() = -transform.linear() * obb.position;

    // Apply the transformation to the point cloud
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // Create a second transformation to rotate -90 degrees about Y axis, then 90 degrees about X axis,
    // and translate 1m along X axis
    Eigen::Affine3f sensor_transform = Eigen::Affine3f::Identity();

    // Rotate -90 degrees about Y axis
    Eigen::AngleAxisf rotationY(-M_PI / 2, Eigen::Vector3f::UnitY());

    // Rotate 90 degrees about X axis
    Eigen::AngleAxisf rotationX(M_PI / 2, Eigen::Vector3f::UnitX());

    // Combine rotations (apply Y rotation first, then X rotation)
    sensor_transform.rotate(rotationX * rotationY);

    // Translate 1 meter along X axis
    sensor_transform.translation() = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    // Apply the second transformation
    auto sensor_cloud = std::make_shared<PointCloud>();
    pcl::transformPointCloud(*transformed_cloud, *sensor_cloud, sensor_transform);

    return sensor_cloud;
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

std::vector<std::tuple<pcl::RangeImage::Ptr, pcl::RangeImage::Ptr>> create_range_images(
    const std::vector<PointCloud::Ptr> &clouds, const std::vector<OrientedBoundingBox> &boxes, float angular_resolution
) {
    std::vector<std::tuple<pcl::RangeImage::Ptr, pcl::RangeImage::Ptr>> range_images;

    if (clouds.size() != boxes.size()) {
        std::cerr << "Error: Number of clouds (" << clouds.size() << ") does not match number of boxes ("
                  << boxes.size() << ")" << std::endl;
        return range_images;
    }

    angular_resolution = pcl::deg2rad(angular_resolution);

    range_images.reserve(clouds.size());

    for (size_t i = 0; i < clouds.size(); ++i) {
        auto transformed_cloud = transform_cloud_for_imaging(clouds[i], boxes[i]);
        auto intensity_cloud = create_intensity_scaled_cloud(transformed_cloud);

        range_images.push_back(std::make_tuple(
            create_range_image_from_cloud(transformed_cloud, angular_resolution),
            create_range_image_from_cloud(intensity_cloud, angular_resolution)
        ));
    }

    return range_images;
}

PointCloud::Ptr create_intensity_scaled_cloud(const PointCloud::Ptr &cloud) {
    if (cloud->empty()) {
        std::cerr << "Warning: Empty cloud passed to create_intensity_scaled_cloud" << std::endl;
        return std::make_shared<PointCloud>();
    }

    auto scaled_cloud = std::make_shared<PointCloud>();
    scaled_cloud->reserve(cloud->size());

    for (const auto &point : *cloud) {
        // Skip points with zero intensity
        if (point.intensity <= 0.0f) {
            continue;
        }

        // Create a new point with coordinates scaled by intensity
        Point scaled_point;
        scaled_point.x = point.x * point.intensity;
        scaled_point.y = point.y * point.intensity;
        scaled_point.z = point.z * point.intensity;
        scaled_point.intensity = point.intensity; // Keep the original intensity

        scaled_cloud->push_back(scaled_point);
    }

    // Copy the header information
    scaled_cloud->header = cloud->header;
    scaled_cloud->width = scaled_cloud->size();
    scaled_cloud->height = 1;       // Unorganized point cloud
    scaled_cloud->is_dense = false; // May contain invalid points

    return scaled_cloud;
}

cv::Mat convert_range_image_to_cv_mat(const pcl::RangeImage::Ptr &range_image) {
    if (!range_image || range_image->empty()) {
        std::cerr << "Warning: Empty range image passed to convert_range_image_to_cv_mat" << std::endl;
        return cv::Mat();
    }

    // Get the dimensions of the range image
    int width = range_image->width;
    int height = range_image->height;

    // Create an OpenCV matrix to hold the colored image (BGR format)
    cv::Mat cv_image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Find min and max range values for normalization
    float min_range = std::numeric_limits<float>::max();
    float max_range = -std::numeric_limits<float>::max();

    // First pass to find min and max values
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float range = range_image->getPoint(x, y).range;

            // Skip invalid points (NaN or infinity)
            if (!std::isfinite(range) || range < 0) {
                continue;
            }

            min_range = std::min(min_range, range);
            max_range = std::max(max_range, range);
        }
    }

    // If we couldn't find valid min/max values, return empty image
    if (min_range >= max_range) {
        std::cerr << "Warning: Could not determine valid range bounds in range image" << std::endl;
        return cv_image;
    }

    // Second pass to normalize and colorize
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float range = range_image->getPoint(x, y).range;

            // Skip invalid points
            if (!std::isfinite(range) || range < 0) {
                continue;
            }

            // Normalize the range value to [0, 1]
            float normalized_range = (range - min_range) / (max_range - min_range);

            // Get color for the normalized value using PCL's utility
            uint8_t r, g, b;
            pcl::visualization::FloatImageUtils::getColorForFloat(normalized_range, r, g, b);

            // OpenCV uses BGR format
            cv_image.at<cv::Vec3b>(y, x) = cv::Vec3b(b, g, r);
        }
    }

    return cv_image;
}
