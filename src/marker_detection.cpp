#include "marker_detection.hpp"
#include <iostream>

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
