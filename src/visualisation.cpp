#include "visualisation.hpp"
#include <opencv2/opencv.hpp>
#include <random>

std::tuple<std::shared_ptr<pcl::visualization::PCLVisualizer>, Viewports> create_visualizer() {
    auto viewer = std::make_shared<pcl::visualization::PCLVisualizer>("Point Cloud Viewer");
    Viewports viewports;

    // Create three viewports
    viewer->createViewPort(0.0, 0.0, 0.33, 1.0, viewports.v1);
    viewer->createViewPort(0.33, 0.0, 0.67, 1.0, viewports.v2);
    viewer->createViewPort(0.67, 0.0, 1.0, 1.0, viewports.v3);

    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewports.v1);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewports.v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewports.v3);

    // Add coordinate system axes to each viewport
    viewer->addCoordinateSystem(1.0, "coordinate_system_v1", viewports.v1);
    viewer->addCoordinateSystem(1.0, "coordinate_system_v2", viewports.v2);
    viewer->addCoordinateSystem(1.0, "coordinate_system_v3", viewports.v3);

    viewer->addText("Original Point Cloud", 10, 10, 1.0, 1.0, 1.0, "v1_text", viewports.v1);
    viewer->addText("Filtered Point Cloud", 10, 10, 1.0, 1.0, 1.0, "v2_text", viewports.v2);
    viewer->addText("Points in OBBs", 10, 10, 1.0, 1.0, 1.0, "v3_text", viewports.v3);

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

void add_oriented_bounding_box(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const OrientedBoundingBox &obb,
    const std::string &id,
    int viewport_id,
    double r,
    double g,
    double b
) {
    viewer->addCube(
        obb.position, obb.orientation, obb.dimensions[0], obb.dimensions[1], obb.dimensions[2], id, viewport_id
    );

    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, id, viewport_id);

    viewer->setShapeRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
        id,
        viewport_id
    );

    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2.0, id, viewport_id);
}

void visualize_oriented_bounding_boxes(
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
    const std::vector<OrientedBoundingBox> &boxes,
    int viewport_id,
    double r,
    double g,
    double b
) {
    // Add each oriented bounding box
    for (size_t i = 0; i < boxes.size(); ++i) {
        std::string obb_id = "obb_" + std::to_string(i);
        add_oriented_bounding_box(viewer, boxes[i], obb_id, viewport_id, r, g, b);
    }
}

bool save_range_image_as_png(
    const pcl::RangeImage::Ptr &range_image, const std::string &filename, float min_range, float max_range
) {
    if (!range_image || range_image->empty()) {
        std::cerr << "Error: Cannot save empty range image" << std::endl;
        return false;
    }

    // Get image dimensions
    int width = range_image->width;
    int height = range_image->height;

    if (width == 0 || height == 0) {
        std::cerr << "Error: Invalid range image dimensions: " << width << "x" << height << std::endl;
        return false;
    }

    // Create OpenCV image (8-bit, 1 channel)
    cv::Mat image(height, width, CV_8UC1);

    // Auto-detect range if not specified
    if (min_range < 0 || max_range < 0) {
        min_range = std::numeric_limits<float>::max();
        max_range = -std::numeric_limits<float>::max();

        // Find min and max range values
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                float range = range_image->at(x, y).range;
                if (std::isfinite(range)) {
                    min_range = std::min(min_range, range);
                    max_range = std::max(max_range, range);
                }
            }
        }

        // If no valid points were found
        if (min_range > max_range) {
            min_range = 0.0f;
            max_range = 1.0f;
        }
    }

    float range_diff = max_range - min_range;
    if (range_diff <= 0)
        range_diff = 1.0f; // Avoid division by zero

    // Convert range values to grayscale
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float range = range_image->at(x, y).range;

            // Handle invalid points (NaN or out of range)
            if (!std::isfinite(range)) {
                image.at<uchar>(y, x) = 0; // Black for invalid points
            } else {
                // Normalize to 0-255 range
                float normalized = (range - min_range) / range_diff;
                image.at<uchar>(y, x) = static_cast<uchar>(normalized * 255);
            }
        }
    }

    // Save the image
    bool success = cv::imwrite(filename, image);
    if (success) {
        std::cout << "Range image saved to: " << filename << std::endl;
    } else {
        std::cerr << "Failed to save range image to: " << filename << std::endl;
    }

    return success;
}
