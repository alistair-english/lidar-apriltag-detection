#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <opencv2/core/version.hpp>
#include "utils.h"
#include "aruco_detector.h"

/**
 * Main function for ArUco marker detection in point clouds
 */
int main(int argc, char** argv) {
    // Print PCL and OpenCV versions for debugging
    std::cout << "Using PCL version: " << PCL_VERSION_PRETTY << std::endl;
    std::cout << "Using OpenCV version: " << CV_VERSION << std::endl;
    
    // Initialize random seed for visualization colors
    srand(static_cast<unsigned int>(time(nullptr)));
    
    // Create ArUco detector with default config path
    ArUcoDetector detector;
    
    // Load point cloud from file specified in config
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    YAML::Node config = loadConfiguration("config.yaml");
    std::string filename = config["filename"].as<std::string>();
    
    if (pcl::io::loadPCDFile(filename, *cloud) == -1) {
        PCL_ERROR("Failed to read point cloud file\n");
        return 1;
    }
    
    // Detect markers in the point cloud
    if (detector.detectMarkers(cloud)) {
        // Print information about detected markers
        detector.printMarkerInfo();
    }
    
    // Visualize the detection process and results
    detector.visualizeResults(cloud);
    
    return 0;
}
