#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace lidar_aruco_detection {

struct MarkerDetection {
    int id;                           // Marker ID
    std::vector<cv::Point2f> corners; // Four corners of the marker
};

/**
 * Detect ArUco markers in an image
 *
 * @param image Input image to detect markers in
 * @param dictionary_name Name of the ArUco dictionary to use (e.g., "DICT_4X4_50")
 * @return Vector of detected markers
 */
std::vector<MarkerDetection> detect_markers(const cv::Mat &image, const std::string &dictionary_name);

/**
 * Draw detected markers on an image
 *
 * @param image Input/output image to draw markers on
 * @param detections Vector of marker detections
 * @return Image with markers drawn on it
 */
cv::Mat draw_markers(const cv::Mat &image, const std::vector<MarkerDetection> &detections);

/**
 * Get ArUco dictionary by name
 *
 * @param name Dictionary name (e.g., "DICT_4X4_50")
 * @return ArUco dictionary
 */
cv::Ptr<cv::aruco::Dictionary> get_dictionary_by_name(const std::string &name);

/**
 * Calculate the pose of a marker from its 3D corner points
 *
 * @param corner_points_3d 3D coordinates of the marker corners
 * @return Tuple containing position (center) and orientation (quaternion) of the marker
 */
std::tuple<Eigen::Vector3f, Eigen::Quaternionf>
calculate_marker_pose(const std::vector<Eigen::Vector3f> &corner_points_3d);

} // namespace lidar_aruco_detection
