#include <Eigen/Eigen>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <pcl/range_image/range_image.h>
#include <string>
#include <vector>

#include "lidar_aruco_detection/image_marker_detection.hpp"
#include "lidar_aruco_detection/oriented_bounding_box.hpp"
#include "pointcloud.hpp"

namespace lidar_aruco_detection {

struct MarkerSearchConfiguration {
    // Image projection parameters
    float angular_resolution_deg = 0.3f;

    // Marker detection parameters
    std::string dictionary_name = "DICT_APRILTAG_36h11";
    int intensity_threshold = 10;
};

struct IntensityGradientConfiguration {
    // Normal estimation parameters
    float normal_estimation_radius = 0.01f;

    // Intensity gradient parameters
    float gradient_estimation_radius = 0.008f;
    float intensity_threshold_percentile = 0.9f;

    // Clustering parameters
    float cluster_tolerance = 0.06f;
    int min_cluster_size = 100;
    int max_cluster_size = 25000;

    // OBB filtering parameters
    float min_diagonal = 0.3f;
    float max_diagonal = 1.0f;
    float max_aspect_ratio = 1.5f;
};

struct MarkerDetection {
    int id;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
};

struct MarkerDetectionDebugData {
    int id;
    std::vector<Eigen::Vector3f> corner_points;
};

struct MarkerSearchDebugData {
    pcl::RangeImage::Ptr range_image;
    cv::Mat intensity_image;
    cv::Mat thresholded_intensity_image;
    std::vector<ImageMarkerDetection> image_marker_detections;
    std::vector<MarkerDetectionDebugData> marker_detections;
    PointCloud::Ptr filtered_cloud = nullptr; // Store the filtered cloud for visualization
};

struct IntensityGradientDebugData {
    PointCloud::Ptr significant_gradient_points;
    std::vector<PointCloud::Ptr> euclidean_clusters;
    std::vector<OrientedBoundingBox> all_obbs;
    std::vector<OrientedBoundingBox> filtered_obbs;
    std::vector<PointCloud::Ptr> obb_filtered_points;
    std::vector<MarkerSearchDebugData> obb_marker_searches;
};

std::vector<MarkerDetection> detect_markers_using_intensity_gradient_clustering(
    const PointCloud::Ptr &cloud,
    const IntensityGradientConfiguration &intensity_gradient_config,
    const MarkerSearchConfiguration &marker_search_config,
    std::shared_ptr<IntensityGradientDebugData> debug_data = nullptr
);

std::vector<MarkerDetection> detect_markers_using_single_range_image_search(
    const PointCloud::Ptr &cloud,
    const MarkerSearchConfiguration &marker_search_config,
    std::shared_ptr<MarkerSearchDebugData> debug_data = nullptr
);

} // namespace lidar_aruco_detection
