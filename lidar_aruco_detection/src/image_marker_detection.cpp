#include "lidar_aruco_detection/image_marker_detection.hpp"
#include <iostream>

namespace lidar_aruco_detection {

cv::Ptr<cv::aruco::Dictionary> get_dictionary_by_name(const std::string &name) {
    if (name == "DICT_4X4_50")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    if (name == "DICT_4X4_100")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
    if (name == "DICT_4X4_250")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    if (name == "DICT_4X4_1000")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);
    if (name == "DICT_5X5_50")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    if (name == "DICT_5X5_100")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
    if (name == "DICT_5X5_250")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    if (name == "DICT_5X5_1000")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);
    if (name == "DICT_6X6_50")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    if (name == "DICT_6X6_100")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
    if (name == "DICT_6X6_250")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    if (name == "DICT_6X6_1000")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    if (name == "DICT_7X7_50")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
    if (name == "DICT_7X7_100")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_100);
    if (name == "DICT_7X7_250")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_250);
    if (name == "DICT_7X7_1000")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_1000);
    if (name == "DICT_ARUCO_ORIGINAL")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    if (name == "DICT_APRILTAG_16h5")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
    if (name == "DICT_APRILTAG_25h9")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_25h9);
    if (name == "DICT_APRILTAG_36h10")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h10);
    if (name == "DICT_APRILTAG_36h11")
        return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

    // Default to DICT_4X4_50 if name not recognized
    std::cerr << "Warning: Dictionary name '" << name << "' not recognized. Using DICT_4X4_50." << std::endl;
    return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
}

std::vector<MarkerDetection> detect_markers(const cv::Mat &image, const std::string &dictionary_name) {
    std::vector<MarkerDetection> result;

    // Check if image is valid
    if (image.empty()) {
        std::cerr << "Error: Empty image passed to detect_markers" << std::endl;
        return result;
    }

    // Get the dictionary
    cv::Ptr<cv::aruco::Dictionary> dictionary = get_dictionary_by_name(dictionary_name);

    // Create detector parameters
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    // Variables to store detection results
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;

    // Detect markers
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);

    // Process detection results
    for (size_t i = 0; i < markerIds.size(); i++) {
        MarkerDetection detection;
        detection.id = markerIds[i];
        detection.corners = markerCorners[i];

        result.push_back(detection);
    }

    return result;
}

cv::Mat draw_markers(const cv::Mat &image, const std::vector<MarkerDetection> &detections) {
    // Create a copy of the input image to draw on
    cv::Mat outputImage;
    if (image.channels() == 1) {
        // Convert grayscale to color for better visualization
        cv::cvtColor(image, outputImage, cv::COLOR_GRAY2BGR);
    } else {
        image.copyTo(outputImage);
    }

    // Prepare data structures for drawing
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;

    for (const auto &detection : detections) {
        corners.push_back(detection.corners);
        ids.push_back(detection.id);
    }

    // Draw the markers
    if (!corners.empty()) {
        cv::aruco::drawDetectedMarkers(outputImage, corners, ids);
    }

    return outputImage;
}

std::tuple<Eigen::Vector3f, Eigen::Quaternionf>
calculate_marker_pose(const std::vector<Eigen::Vector3f> &corner_points_3d) {

    // Default return values
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();

    // Count valid points and compute center
    std::vector<Eigen::Vector3f> valid_points;
    for (const auto &pt : corner_points_3d) {
        if (std::isfinite(pt.x()) && std::isfinite(pt.y()) && std::isfinite(pt.z())) {
            valid_points.push_back(pt);
            position += pt;
        }
    }

    if (valid_points.size() < 4) {
        std::cerr << "Warning: Not enough valid corner points to calculate marker pose" << std::endl;
        return {position, orientation};
    }

    // Calculate center position (average of all valid corners)
    position /= static_cast<float>(valid_points.size());

    // Calculate marker orientation
    // We need to define the marker coordinate system:
    // - Origin at the center of the marker
    // - Y axis from center to the middle of the first edge (between corners 0 and 1)
    // - X axis from center to the middle of the second edge (between corners 1 and 2)
    // - Z axis as the cross product of X and Y (perpendicular to the marker plane)

    // For a standard ArUco marker, corners are ordered:
    // 0: top-left, 1: top-right, 2: bottom-right, 3: bottom-left

    if (valid_points.size() == 4) {
        // We have all four corners, use them to define the coordinate system

        // X axis: from center to the middle of the top edge (between corners 1 and 2)
        Eigen::Vector3f x_axis = ((valid_points[1] + valid_points[2]) / 2.0f) - position;
        x_axis.normalize();

        // Y axis: from center to the middle of the left edge (between corners 0 and 1)
        Eigen::Vector3f y_axis = ((valid_points[0] + valid_points[1]) / 2.0f) - position;
        y_axis.normalize();

        // Make Y orthogonal to X
        y_axis = y_axis - x_axis * (x_axis.dot(y_axis));
        y_axis.normalize();

        // Z axis: cross product of X and Y
        Eigen::Vector3f z_axis = x_axis.cross(y_axis);
        z_axis.normalize();

        // Create rotation matrix from the three axes
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix.col(0) = x_axis;
        rotation_matrix.col(1) = y_axis;
        rotation_matrix.col(2) = z_axis;

        // Convert to quaternion
        orientation = Eigen::Quaternionf(rotation_matrix);
    } else {
        // We don't have all four corners, use a simpler approach
        // Fit a plane to the points and use the normal as Z axis

        // Calculate centroid
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto &pt : valid_points) {
            centroid += pt;
        }
        centroid /= static_cast<float>(valid_points.size());

        // Calculate covariance matrix
        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto &pt : valid_points) {
            Eigen::Vector3f centered = pt - centroid;
            covariance += centered * centered.transpose();
        }

        // Perform SVD to find the normal (smallest eigenvector)
        Eigen::JacobiSVD<Eigen::Matrix3f> svd(covariance, Eigen::ComputeFullU);
        Eigen::Vector3f normal = svd.matrixU().col(2);

        // Make sure the normal points towards the viewer (positive Z)
        if (normal.z() < 0) {
            normal = -normal;
        }

        // Create a coordinate system with Z as the normal
        Eigen::Vector3f z_axis = normal;

        // Choose X axis (any vector perpendicular to Z)
        Eigen::Vector3f x_axis;
        if (std::abs(z_axis.x()) < std::abs(z_axis.y()) && std::abs(z_axis.x()) < std::abs(z_axis.z())) {
            x_axis = Eigen::Vector3f(1, 0, 0).cross(z_axis);
        } else if (std::abs(z_axis.y()) < std::abs(z_axis.z())) {
            x_axis = Eigen::Vector3f(0, 1, 0).cross(z_axis);
        } else {
            x_axis = Eigen::Vector3f(0, 0, 1).cross(z_axis);
        }
        x_axis.normalize();

        // Y axis is the cross product of Z and X
        Eigen::Vector3f y_axis = z_axis.cross(x_axis);
        y_axis.normalize();

        // Create rotation matrix from the three axes
        Eigen::Matrix3f rotation_matrix;
        rotation_matrix.col(0) = x_axis;
        rotation_matrix.col(1) = y_axis;
        rotation_matrix.col(2) = z_axis;

        // Convert to quaternion
        orientation = Eigen::Quaternionf(rotation_matrix);
    }

    return {position, orientation};
}

} // namespace lidar_aruco_detection
