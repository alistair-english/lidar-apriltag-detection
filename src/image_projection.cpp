#include "image_projection.hpp"

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

pcl::RangeImage::Ptr
create_range_image(const PointCloud::Ptr &cloud, float angular_resolution_x, float angular_resolution_y) {

    // Convert from degrees to radians
    angular_resolution_x = pcl::deg2rad(angular_resolution_x);
    angular_resolution_y = pcl::deg2rad(angular_resolution_y);

    // Create empty range image
    auto range_image_ptr = std::make_shared<pcl::RangeImage>();
    pcl::RangeImage &range_image = *range_image_ptr;

    // Set parameters
    Eigen::Affine3f sensor_pose = Eigen::Affine3f::Identity();
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;

    // Create range image from point cloud
    range_image.createFromPointCloud(
        *cloud,
        angular_resolution_x,
        angular_resolution_y,
        pcl::deg2rad(360.0f),
        pcl::deg2rad(180.0f),
        sensor_pose,
        coordinate_frame,
        noise_level,
        min_range,
        border_size
    );

    return range_image_ptr;
}

std::vector<pcl::RangeImage::Ptr> create_range_images(
    const std::vector<PointCloud::Ptr> &clouds, float angular_resolution_x, float angular_resolution_y
) {

    std::vector<pcl::RangeImage::Ptr> range_images;
    range_images.reserve(clouds.size());

    for (size_t i = 0; i < clouds.size(); ++i) {
        // Skip empty clouds
        if (clouds[i]->empty()) {
            std::cout << "Skipping empty cloud " << i << std::endl;
            continue;
        }

        // Create range image
        auto range_image = create_range_image(clouds[i], angular_resolution_x, angular_resolution_y);
        range_images.push_back(range_image);
    }

    return range_images;
}

cv::Mat range_image_to_opencv(const pcl::RangeImage::Ptr &range_image) {
    if (!range_image || range_image->empty()) {
        std::cerr << "Empty range image passed to range_image_to_opencv" << std::endl;
        return cv::Mat();
    }

    // Get range image dimensions
    int width = range_image->width;
    int height = range_image->height;

    // Create OpenCV matrix for the range image (8-bit, 3 channels)
    cv::Mat range_image_mat(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Find min and max range values for normalization
    float min_range = std::numeric_limits<float>::max();
    float max_range = -std::numeric_limits<float>::max();

    for (int i = 0; i < width * height; ++i) {
        float range = range_image->points[i].range;
        if (!std::isfinite(range))
            continue;

        min_range = std::min(min_range, range);
        max_range = std::max(max_range, range);
    }

    // Normalize and convert to color image
    float range_diff = max_range - min_range;
    if (range_diff <= 0)
        range_diff = 1.0f; // Avoid division by zero

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int idx = y * width + x;
            float range = range_image->points[idx].range;

            if (std::isfinite(range)) {
                // Normalize range to [0, 1]
                float normalized_range = (range - min_range) / range_diff;

                // Convert to color using a simple jet colormap
                cv::Vec3b color;
                if (normalized_range < 0.25) {
                    // Blue to cyan
                    color[0] = 255;
                    color[1] = static_cast<uchar>(255 * normalized_range * 4);
                    color[2] = 0;
                } else if (normalized_range < 0.5) {
                    // Cyan to green
                    color[0] = static_cast<uchar>(255 * (1 - (normalized_range - 0.25) * 4));
                    color[1] = 255;
                    color[2] = 0;
                } else if (normalized_range < 0.75) {
                    // Green to yellow
                    color[0] = 0;
                    color[1] = 255;
                    color[2] = static_cast<uchar>(255 * (normalized_range - 0.5) * 4);
                } else {
                    // Yellow to red
                    color[0] = 0;
                    color[1] = static_cast<uchar>(255 * (1 - (normalized_range - 0.75) * 4));
                    color[2] = 255;
                }

                range_image_mat.at<cv::Vec3b>(y, x) = color;
            }
        }
    }

    return range_image_mat;
}

bool save_range_image(const pcl::RangeImage::Ptr &range_image, const std::string &filename) {
    if (!range_image || range_image->empty()) {
        std::cerr << "Empty range image passed to save_range_image" << std::endl;
        return false;
    }

    cv::Mat range_image_mat = range_image_to_opencv(range_image);
    if (range_image_mat.empty()) {
        std::cerr << "Failed to convert range image to OpenCV matrix" << std::endl;
        return false;
    }

    try {
        bool success = cv::imwrite(filename, range_image_mat);
        if (success) {
            std::cout << "Saved range image to " << filename << std::endl;
        } else {
            std::cerr << "Failed to save range image to " << filename << std::endl;
        }
        return success;
    } catch (const cv::Exception &e) {
        std::cerr << "OpenCV exception while saving range image: " << e.what() << std::endl;
        return false;
    }
}

std::vector<std::string>
save_range_images(const std::vector<pcl::RangeImage::Ptr> &range_images, const std::string &base_filename) {

    std::vector<std::string> saved_filenames;
    saved_filenames.reserve(range_images.size());

    for (size_t i = 0; i < range_images.size(); ++i) {
        // Create filename with padded index
        std::ostringstream filename_stream;
        filename_stream << base_filename << std::setw(3) << std::setfill('0') << i << ".png";
        std::string filename = filename_stream.str();

        if (save_range_image(range_images[i], filename)) {
            saved_filenames.push_back(filename);
        }
    }

    return saved_filenames;
}
