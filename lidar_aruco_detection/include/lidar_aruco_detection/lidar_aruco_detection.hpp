#include <Eigen/Eigen>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <pcl/range_image/range_image.h>
#include <vector>

#include "lidar_aruco_detection/image_marker_detection.hpp"
#include "lidar_aruco_detection/oriented_bounding_box.hpp"
#include "pointcloud.hpp"

namespace lidar_aruco_detection {

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
};

struct IntensityGradientDebugData {
    PointCloud::Ptr significant_gradient_points;
    std::vector<PointCloud::Ptr> euclidean_clusters;
    std::vector<OrientedBoundingBox> all_obbs;
    std::vector<OrientedBoundingBox> filtered_obbs;
    std::vector<PointCloud::Ptr> obb_filtered_points;
    std::vector<MarkerSearchDebugData> obb_marker_searches;
};

/**
 * @brief Detect markers in a point cloud using intensity gradient clustering
 *
 * @param cloud Input point cloud
 * @param debug_data Optional reference to store debug data
 * @return std::vector<MarkerDetection> Vector of detected markers
 */
std::vector<MarkerDetection> detect_markers_using_intensity_gradient_clustering(
    const PointCloud::Ptr &cloud, std::shared_ptr<IntensityGradientDebugData> debug_data = nullptr
);
} // namespace lidar_aruco_detection
