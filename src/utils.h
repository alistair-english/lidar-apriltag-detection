#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <random>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl/range_image/range_image.h>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Core>
#include "yaml-cpp/yaml.h"

/**
 * Generates random RGB values for visualization
 * @return Pointer to array of 3 integers representing RGB values
 */
int* generateRandomRGB();

/**
 * Loads configuration from YAML file
 * @param configPath Path to the YAML configuration file
 * @return YAML::Node containing the configuration
 */
YAML::Node loadConfiguration(const std::string& configPath);

/**
 * Sets up PCL visualizer with multiple viewports
 * @param viewerPtr Pointer to PCL visualizer
 * @param viewportIds Array to store viewport IDs
 * @param numViewports Number of viewports to create
 */
void setupVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, 
                     std::vector<int>& viewportIds,
                     int numViewports = 6);

/**
 * Estimates surface normals for a point cloud
 * @param cloud Input point cloud
 * @param normRadius Radius for normal estimation
 * @return Point cloud of normals
 */
pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float normRadius);

/**
 * Estimates intensity gradients for a point cloud
 * @param cloud Input point cloud
 * @param normals Point cloud of normals
 * @param gradientRadius Radius for gradient estimation
 * @return Point cloud of intensity gradients
 */
pcl::PointCloud<pcl::IntensityGradient>::Ptr estimateIntensityGradients(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    float gradientRadius);

/**
 * Calculates intensity threshold based on gradient magnitudes
 * @param gradient Intensity gradient point cloud
 * @param ratioThreshold Ratio for threshold calculation
 * @return Calculated intensity threshold
 */
float calculateIntensityThreshold(
    const pcl::PointCloud<pcl::IntensityGradient>& gradient, 
    float ratioThreshold);

/**
 * Extracts intensity features from point cloud based on gradient magnitude
 * @param cloud Input point cloud
 * @param gradient Intensity gradient point cloud
 * @param intensityThreshold Threshold for feature extraction
 * @return Point cloud containing intensity features
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr extractIntensityFeatures(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const pcl::PointCloud<pcl::IntensityGradient>& gradient,
    float intensityThreshold);

/**
 * Performs Euclidean clustering on a point cloud
 * @param cloud Input point cloud
 * @param clusterTolerance Distance tolerance for clustering
 * @param minClusterSize Minimum cluster size
 * @param maxClusterSize Maximum cluster size
 * @return Vector of point indices for each cluster
 */
std::vector<pcl::PointIndices> performEuclideanClustering(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float clusterTolerance,
    int minClusterSize,
    int maxClusterSize);

/**
 * Creates a range image from a point cloud
 * @param cloud Input point cloud
 * @param angularResolution Angular resolution in radians
 * @return Range image created from the point cloud
 */
pcl::RangeImage::Ptr createRangeImage(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float angularResolution);

/**
 * Converts a range image to OpenCV Mat
 * @param rangeImage Input range image
 * @return OpenCV Mat representation of the range image
 */
cv::Mat rangeImageToMat(const pcl::RangeImage& rangeImage);

/**
 * Processes an image for ArUco marker detection
 * @param image Input image
 * @param binaryThreshold Threshold for binarization
 * @return Processed grayscale image
 */
cv::Mat preprocessImageForArUco(const cv::Mat& image, float binaryThreshold);

/**
 * Detects ArUco markers in an image
 * @param image Input image
 * @param dictionary ArUco dictionary to use
 * @param parameters Detection parameters
 * @param markerCorners Output vector of marker corners
 * @param markerIds Output vector of marker IDs
 * @return True if markers were detected, false otherwise
 */
bool detectArUcoMarkers(
    const cv::Mat& image,
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Ptr<cv::aruco::DetectorParameters>& parameters,
    std::vector<std::vector<cv::Point2f>>& markerCorners,
    std::vector<int>& markerIds);

/**
 * Calculates 3D coordinates of marker corners
 * @param rangeImage Range image for 3D point calculation
 * @param markerCorners 2D coordinates of marker corners
 * @param rotMatrix Rotation matrix for transformation
 * @param position Position vector for transformation
 * @return Vector of 3D coordinates for marker corners
 */
std::vector<std::vector<float>> calculateMarkerCorners3D(
    const pcl::RangeImage& rangeImage,
    const std::vector<std::vector<cv::Point2f>>& markerCorners,
    const Eigen::Matrix3f& rotMatrix,
    const Eigen::Vector3f& position);

#endif // UTILS_H
