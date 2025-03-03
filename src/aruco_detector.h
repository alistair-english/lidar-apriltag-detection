#ifndef ARUCO_DETECTOR_H
#define ARUCO_DETECTOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/range_image/range_image.h>
#include <opencv2/aruco.hpp>
#include <Eigen/Core>
#include "yaml-cpp/yaml.h"
#include <vector>
#include <string>

class ArUcoDetector {
public:
    /**
     * Constructor
     * @param configPath Path to configuration file
     */
    ArUcoDetector(const std::string& configPath = "config.yaml");
    
    /**
     * Detects ArUco markers in a point cloud
     * @param cloud Input point cloud
     * @return True if markers were detected, false otherwise
     */
    bool detectMarkers(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    /**
     * Visualizes the detection process and results
     * @param cloud Input point cloud
     */
    void visualizeResults(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    /**
     * Prints detected marker information
     */
    void printMarkerInfo() const;
    
private:
    // Configuration parameters
    YAML::Node config;
    std::string filename;
    std::string tagFamily;
    float markerSize;
    float tolerance;
    float ratioThreshold;
    float buffer;
    float intensityThreshold;
    float euclideanTolerance;
    float angResolution;
    float binaryThreshold;
    float normRadius;
    float gradientRadius;
    
    // Detection results
    std::vector<std::vector<float>> markerVertices;
    std::vector<int> markerIds;
    
    // Visualization
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    std::vector<int> viewportIds;
    
    /**
     * Loads configuration from YAML file
     * @param configPath Path to configuration file
     */
    void loadConfig(const std::string& configPath);
    
    /**
     * Processes a cluster to check if it's a potential marker
     * @param cluster Point cloud cluster
     * @param cloud Original point cloud
     * @param clusterIndex Index for visualization
     * @return True if the cluster is a potential marker, false otherwise
     */
    bool processCluster(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        int clusterIndex);
    
    /**
     * Extracts points within an oriented bounding box
     * @param cloud Input point cloud
     * @param position OBB position
     * @param rotMatrix OBB rotation matrix
     * @param minPoint OBB minimum point
     * @param maxPoint OBB maximum point
     * @return Point cloud containing points within the OBB
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr extractPointsInBox(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const Eigen::Vector3f& position,
        const Eigen::Matrix3f& rotMatrix,
        const pcl::PointXYZI& minPoint,
        const pcl::PointXYZI& maxPoint);
    
    /**
     * Detects ArUco markers in a range image
     * @param rangeImage Range image for detection
     * @param position OBB position
     * @param rotMatrix OBB rotation matrix
     * @return True if markers were detected, false otherwise
     */
    bool detectMarkersInRangeImage(
        const pcl::RangeImage::Ptr& rangeImage,
        const Eigen::Vector3f& position,
        const Eigen::Matrix3f& rotMatrix);
};

#endif // ARUCO_DETECTOR_H
