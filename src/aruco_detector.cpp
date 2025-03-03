#include "aruco_detector.h"
#include "utils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>

ArUcoDetector::ArUcoDetector(const std::string& configPath) {
    loadConfig(configPath);
    
    // Initialize viewer
    viewer.reset(new pcl::visualization::PCLVisualizer("3D View"));
    setupVisualizer(viewer, viewportIds);
}

void ArUcoDetector::loadConfig(const std::string& configPath) {
    config = loadConfiguration(configPath);
    
    filename = config["filename"].as<std::string>();
    tagFamily = config["tag_family"].as<std::string>();
    markerSize = config["marker_size"].as<float>();
    tolerance = config["tolerance"].as<float>();
    ratioThreshold = config["ratio_threshold"].as<float>();
    buffer = config["buffer"].as<float>();
    intensityThreshold = config["intensity_threshold"].as<float>();
    euclideanTolerance = config["EuclideanTolerance"].as<float>();
    angResolution = config["ang_resolution"].as<float>();
    binaryThreshold = config["binary_threshold"].as<float>();
    normRadius = config["norm_radius"].as<float>();
    gradientRadius = config["gradient_radius"].as<float>();
}

bool ArUcoDetector::detectMarkers(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    // Clear previous results
    markerVertices.clear();
    markerIds.clear();
    
    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals = estimateNormals(cloud, normRadius);
    
    // Estimate intensity gradients
    pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients = 
        estimateIntensityGradients(cloud, normals, gradientRadius);
    
    // Calculate intensity threshold if not provided
    if (intensityThreshold <= 0) {
        intensityThreshold = calculateIntensityThreshold(*gradients, 0.02);
    }
    
    // Extract intensity features
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensityFeatures = 
        extractIntensityFeatures(cloud, *gradients, intensityThreshold);
    
    // Perform Euclidean clustering
    std::vector<pcl::PointIndices> clusterIndices = 
        performEuclideanClustering(intensityFeatures, euclideanTolerance, 100, 100000);
    
    // Process each cluster
    int clusterIndex = 0;
    for (const auto& indices : clusterIndices) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
        
        for (const auto& idx : indices.indices) {
            cluster->push_back((*intensityFeatures)[idx]);
        }
        
        cluster->width = cluster->size();
        cluster->height = 1;
        cluster->is_dense = true;
        
        // Process the cluster to check if it's a potential marker
        processCluster(cluster, cloud, clusterIndex++);
    }
    
    return !markerIds.empty();
}

bool ArUcoDetector::processCluster(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    int clusterIndex) {
    
    // Calculate moment of inertia and OBB
    pcl::MomentOfInertiaEstimation<pcl::PointXYZI> featureExtractor;
    featureExtractor.setInputCloud(cluster);
    featureExtractor.compute();
    
    std::vector<float> momentOfInertia;
    std::vector<float> eccentricity;
    pcl::PointXYZI minPointOBB, maxPointOBB, positionOBB;
    Eigen::Matrix3f rotationalMatrixOBB;
    float majorValue, middleValue, minorValue;
    Eigen::Vector3f majorVector, middleVector, minorVector;
    Eigen::Vector3f massCenter;
    
    featureExtractor.getMomentOfInertia(momentOfInertia);
    featureExtractor.getEccentricity(eccentricity);
    featureExtractor.getOBB(minPointOBB, maxPointOBB, positionOBB, rotationalMatrixOBB);
    featureExtractor.getEigenValues(majorValue, middleValue, minorValue);
    featureExtractor.getEigenVectors(majorVector, middleVector, minorVector);
    featureExtractor.getMassCenter(massCenter);
    
    // Calculate OBB dimensions
    float obbWidth = maxPointOBB.x - minPointOBB.x;
    float obbHeight = maxPointOBB.y - minPointOBB.y;
    float obbDepth = maxPointOBB.z - minPointOBB.z;
    float obbDiagonal = std::sqrt(obbWidth*obbWidth + obbHeight*obbHeight + obbDepth*obbDepth);
    
    // Find the two largest dimensions
    float max1 = obbWidth, max2 = obbWidth;
    
    if (obbHeight > max1) {
        max2 = max1;
        max1 = obbHeight;
    } else {
        max2 = obbHeight;
    }
    
    if (obbDepth > max1) {
        max2 = max1;
        max1 = obbDepth;
    } else if (obbDepth > max2) {
        max2 = obbDepth;
    }
    
    // Calculate min/max diagonal based on marker size and tolerance
    float cuboidDiaMin = std::sqrt(2 * markerSize * markerSize);
    float cuboidDiaMax = std::sqrt(4 * markerSize * markerSize + tolerance * tolerance);
    
    // Check if the cluster is a potential marker
    if (max1/max2 < ratioThreshold && max2/max1 > 1/ratioThreshold && 
        cuboidDiaMin <= obbDiagonal && obbDiagonal <= cuboidDiaMax) {
        
        // Extract points within the OBB
        Eigen::Vector3f position(positionOBB.x, positionOBB.y, positionOBB.z);
        Eigen::Vector3f eulerAngles = rotationalMatrixOBB.eulerAngles(1, 0, 2);
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointsInBox = extractPointsInBox(
            cloud, position, rotationalMatrixOBB, minPointOBB, maxPointOBB);
        
        // Create range image from points in box
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformedPoints(new pcl::PointCloud<pcl::PointXYZI>);
        
        // First transformation - to OBB coordinate system
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3,3>(0,0) = rotationalMatrixOBB.inverse();
        transform.block<3,1>(0,3) = -rotationalMatrixOBB.inverse() * position;
        
        pcl::transformPointCloud(*pointsInBox, *transformedPoints, transform);
        
        // Second transformation - for better viewing angle
        Eigen::Matrix4f transform2 = Eigen::Matrix4f::Identity();
        Eigen::Matrix3f R_o;
        R_o << 0, 0, 1, 0, 1, 0, -1, 0, 0;
        transform2.block<3,3>(0,0) = R_o;
        transform2.block<3,1>(0,3) << 1.0, 0, 0;
        
        pcl::transformPointCloud(*transformedPoints, *transformedPoints, transform2);
        
        // Create range image
        pcl::RangeImage::Ptr rangeImage = createRangeImage(
            transformedPoints, angResolution);
        
        // Detect markers in the range image
        return detectMarkersInRangeImage(rangeImage, position, rotationalMatrixOBB);
    }
    
    return false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ArUcoDetector::extractPointsInBox(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const Eigen::Vector3f& position,
    const Eigen::Matrix3f& rotMatrix,
    const pcl::PointXYZI& minPoint,
    const pcl::PointXYZI& maxPoint) {
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointsInBox(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::CropBox<pcl::PointXYZI> boxFilter;
    
    boxFilter.setInputCloud(cloud);
    Eigen::Vector3f rotation = rotMatrix.eulerAngles(1, 0, 2);
    boxFilter.setRotation(rotation);
    boxFilter.setTranslation(position);
    boxFilter.setMin(Eigen::Vector4f(buffer * minPoint.x, buffer * minPoint.y, buffer * minPoint.z, 1.0));
    boxFilter.setMax(Eigen::Vector4f(buffer * maxPoint.x, buffer * maxPoint.y, buffer * maxPoint.x, 1.0));
    
    boxFilter.filter(*pointsInBox);
    
    return pointsInBox;
}

bool ArUcoDetector::detectMarkersInRangeImage(
    const pcl::RangeImage::Ptr& rangeImage,
    const Eigen::Vector3f& position,
    const Eigen::Matrix3f& rotMatrix) {
    
    // Convert range image to OpenCV Mat
    cv::Mat image = rangeImageToMat(*rangeImage);
    
    // Save the image for debugging
    static int imageCounter = 0;
    std::stringstream ss;
    ss << "result" << imageCounter++ << ".png";
    cv::imwrite(ss.str(), image);
    
    // Preprocess image for ArUco detection
    cv::Mat gray = preprocessImageForArUco(image, binaryThreshold);
    cv::Mat grayFlip;
    cv::flip(gray, grayFlip, 1);
    
    // Save processed images for debugging
    std::stringstream ssGray, ssFlip;
    ssGray << "gray" << imageCounter << ".png";
    ssFlip << "flip" << imageCounter << ".png";
    cv::imwrite(ssGray.str(), gray);
    cv::imwrite(ssFlip.str(), grayFlip);
    
    // Set up ArUco detector
    // In newer OpenCV versions, the ArUco API might have changed
    // This is compatible with both older and newer OpenCV versions
    cv::Ptr<cv::aruco::DetectorParameters> parameters;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    
    #if CV_MAJOR_VERSION >= 4 && CV_MINOR_VERSION >= 7
        // For OpenCV 4.7+
        parameters = cv::aruco::DetectorParameters();
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    #else
        // For older OpenCV versions
        parameters = cv::aruco::DetectorParameters::create();
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    #endif
    
    // Detect markers
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<int> ids;
    
    bool useFlip = false;
    bool detected = detectArUcoMarkers(gray, dictionary, parameters, markerCorners, ids);
    
    // If no markers detected or ID is too large, try flipped image
    if (!detected || (ids.size() > 0 && ids[0] > 500)) {
        detected = detectArUcoMarkers(grayFlip, dictionary, parameters, markerCorners, ids);
        useFlip = true;
    }
    
    if (detected) {
        // Add marker IDs to the list
        for (const auto& id : ids) {
            markerIds.push_back(id);
        }
        
        // Calculate 3D coordinates of marker corners
        std::vector<std::vector<float>> corners3D;
        
        if (!useFlip) {
            corners3D = calculateMarkerCorners3D(*rangeImage, markerCorners, rotMatrix, position);
        } else {
            // Adjust marker corners for flipped image
            std::vector<std::vector<cv::Point2f>> flippedCorners = markerCorners;
            for (auto& corners : flippedCorners) {
                for (auto& corner : corners) {
                    corner.x = rangeImage->width - corner.x;
                }
            }
            corners3D = calculateMarkerCorners3D(*rangeImage, flippedCorners, rotMatrix, position);
        }
        
        // Add corners to the list
        markerVertices.insert(markerVertices.end(), corners3D.begin(), corners3D.end());
        
        return true;
    }
    
    return false;
}

void ArUcoDetector::visualizeResults(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    // Add original point cloud to viewer
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fieldColor(cloud, "intensity");
    viewer->addPointCloud(cloud, fieldColor, "raw point cloud", viewportIds[0]);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.25, "raw point cloud");
    
    // Add detected marker vertices to viewer
    if (!markerVertices.empty()) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr vertexCloud(new pcl::PointCloud<pcl::PointXYZ>);
        vertexCloud->width = markerVertices.size();
        vertexCloud->height = 1;
        vertexCloud->is_dense = false;
        vertexCloud->points.resize(vertexCloud->width * vertexCloud->height);
        
        for (size_t i = 0; i < markerVertices.size(); ++i) {
            vertexCloud->points[i].x = markerVertices[i][0];
            vertexCloud->points[i].y = markerVertices[i][1];
            vertexCloud->points[i].z = markerVertices[i][2];
        }
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(vertexCloud, 255, 0, 0);
        viewer->addPointCloud(vertexCloud, rgb, "vertex_cloud", viewportIds[5]);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "vertex_cloud");
    }
    
    // Run the viewer
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }
}

void ArUcoDetector::printMarkerInfo() const {
    std::cout << "Number of detected marker(s) is " << markerIds.size() << "." << std::endl;
    
    if (markerIds.empty()) {
        std::cout << "No aruco is detected." << std::endl;
        return;
    }
    
    std::cout << "----------------------Marker Detection-----------------------" << std::endl;
    for (size_t i = 0; i < markerIds.size(); ++i) {
        std::cout << "ID: " << markerIds[i] << std::endl;
        std::cout << "Location of the vertices with respect to the world coordinate system (m):" << std::endl;
        
        for (int j = 0; j < 4; ++j) {
            size_t idx = 4 * i + j;
            std::cout << "vertex " << j+1 << "\tx=" << markerVertices[idx][0]
                      << "\ty=" << markerVertices[idx][1]
                      << "\tz=" << markerVertices[idx][2] << std::endl;
        }
    }
}
