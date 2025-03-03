#include "utils.h"
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <ctime>
#include <cstdlib>
#include <queue>
#include <limits>
#include <algorithm>
#include <float.h>

int* generateRandomRGB() {
    int *rgb = new int[3];
    rgb[0] = rand() % 255;
    rgb[1] = rand() % 255;
    rgb[2] = rand() % 255;
    return rgb;
}

YAML::Node loadConfiguration(const std::string& configPath) {
    return YAML::LoadFile(configPath);
}

void setupVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, 
                     std::vector<int>& viewportIds,
                     int numViewports) {
    viewportIds.resize(numViewports);
    
    // Top row
    viewer->createViewPort(0.0, 0.5, 0.3333, 1.0, viewportIds[0]);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewportIds[0]);
    
    viewer->createViewPort(0.3333, 0.5, 0.6666, 1.0, viewportIds[1]);
    viewer->setBackgroundColor(0.2, 0.2, 0.2, viewportIds[1]);
    
    viewer->createViewPort(0.6666, 0.5, 1.0, 1.0, viewportIds[2]);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, viewportIds[2]);
    
    // Bottom row
    viewer->createViewPort(0.0, 0.0, 0.3333, 0.5, viewportIds[3]);
    viewer->setBackgroundColor(0.3, 0.3, 0.3, viewportIds[3]);
    
    viewer->createViewPort(0.3333, 0.0, 0.6666, 0.5, viewportIds[4]);
    viewer->setBackgroundColor(0.4, 0.4, 0.4, viewportIds[4]);
    
    viewer->createViewPort(0.6666, 0.0, 1.0, 0.5, viewportIds[5]);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, viewportIds[5]);
}

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float normRadius) {
    
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normEst;
    
    normEst.setInputCloud(cloud);
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    normEst.setSearchMethod(tree);
    normEst.setRadiusSearch(normRadius);
    
    normEst.compute(*normals);
    return normals;
}

pcl::PointCloud<pcl::IntensityGradient>::Ptr estimateIntensityGradients(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& normals,
    float gradientRadius) {
    
    pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(new pcl::PointCloud<pcl::IntensityGradient>);
    pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> gradientEst;
    
    gradientEst.setInputCloud(cloud);
    gradientEst.setInputNormals(normals);
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    gradientEst.setSearchMethod(tree);
    gradientEst.setRadiusSearch(gradientRadius);
    
    gradientEst.compute(*gradients);
    return gradients;
}

float calculateIntensityThreshold(const pcl::PointCloud<pcl::IntensityGradient>& gradient, float ratioThreshold) {
    std::priority_queue<float> magnitudeQueue;
    
    // Calculate gradient magnitudes and add to queue
    for (size_t i = 0; i < gradient.size(); ++i) {
        const float* gEst = gradient[i].gradient;
        float magnitude = std::sqrt(gEst[0]*gEst[0] + gEst[1]*gEst[1] + gEst[2]*gEst[2]);
        if (!std::isnan(magnitude)) {
            magnitudeQueue.push(magnitude);
        }
    }
    
    // Pop elements to get to the threshold percentile
    size_t popCount = static_cast<size_t>(magnitudeQueue.size() * ratioThreshold);
    for (size_t i = 0; i < popCount; i++) {
        magnitudeQueue.pop();
    }
    
    return magnitudeQueue.top();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr extractIntensityFeatures(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const pcl::PointCloud<pcl::IntensityGradient>& gradient,
    float intensityThreshold) {
    
    std::vector<std::vector<float>> validPoints;
    
    // Extract points with gradient magnitude above threshold
    for (size_t p = 0; p < cloud->size(); ++p) {
        const float* gEst = gradient[p].gradient;
        float magnitude = std::sqrt(gEst[0]*gEst[0] + gEst[1]*gEst[1] + gEst[2]*gEst[2]);
        
        if (!std::isnan(magnitude) && magnitude > intensityThreshold) {
            std::vector<float> point = {
                cloud->points[p].x, 
                cloud->points[p].y, 
                cloud->points[p].z, 
                cloud->points[p].intensity
            };
            validPoints.push_back(point);
        }
    }
    
    // Create new point cloud with extracted features
    pcl::PointCloud<pcl::PointXYZI>::Ptr featureCloud(new pcl::PointCloud<pcl::PointXYZI>);
    featureCloud->width = validPoints.size();
    featureCloud->height = 1;
    featureCloud->is_dense = false;
    featureCloud->points.resize(featureCloud->width * featureCloud->height);
    
    for (size_t i = 0; i < validPoints.size(); ++i) {
        featureCloud->points[i].x = validPoints[i][0];
        featureCloud->points[i].y = validPoints[i][1];
        featureCloud->points[i].z = validPoints[i][2];
        featureCloud->points[i].intensity = validPoints[i][3];
    }
    
    return featureCloud;
}

std::vector<pcl::PointIndices> performEuclideanClustering(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float clusterTolerance,
    int minClusterSize,
    int maxClusterSize) {
    
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minClusterSize);
    ec.setMaxClusterSize(maxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
    
    return clusterIndices;
}

pcl::RangeImage::Ptr createRangeImage(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    float angularResolution) {
    
    pcl::RangeImage::Ptr rangeImage(new pcl::RangeImage);
    
    float maxAngleWidth = 120.0f * (M_PI/180.0f);
    float maxAngleHeight = 120.0f * (M_PI/180.0f);
    Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(0.0f, 0.0f, 0.0f));
    pcl::RangeImage::CoordinateFrame coordinateFrame = pcl::RangeImage::LASER_FRAME;
    float noiseLevel = 0.0f;
    float minRange = 0.0f;
    int borderSize = 1;
    
    rangeImage->createFromPointCloud(*cloud, 
                                    angularResolution * (M_PI/180.0f),
                                    maxAngleWidth, 
                                    maxAngleHeight,
                                    sensorPose, 
                                    coordinateFrame, 
                                    noiseLevel, 
                                    minRange, 
                                    borderSize);
    
    // Ensure range image is properly initialized
    if (rangeImage->width == 0 || rangeImage->height == 0) {
        std::cerr << "Warning: Created range image has zero size. Check input point cloud." << std::endl;
    }
    
    return rangeImage;
}

cv::Mat rangeImageToMat(const pcl::RangeImage& rangeImage) {
    // Find maximum range value for normalization
    float* ranges = rangeImage.getRangesArray();
    float maxRange = 0;
    
    for (int i = 0; i < rangeImage.width * rangeImage.height; ++i) {
        float val = *(ranges + i);
        if (val > -FLT_MAX && val < FLT_MAX) {
            maxRange = (val > maxRange) ? val : maxRange;
        }
    }
    
    // Create OpenCV Mat
    cv::Mat image(rangeImage.height, rangeImage.width, CV_8UC4);
    unsigned char r, g, b;
    
    // Convert range image to Mat
    for (int y = 0; y < rangeImage.height; y++) {
        for (int x = 0; x < rangeImage.width; x++) {
            pcl::PointWithRange rangePt = rangeImage.getPoint(x, y);
            float value = rangePt.range / maxRange;
            
            pcl::visualization::FloatImageUtils::getColorForFloat(value, r, g, b);
            
            image.at<cv::Vec4b>(y, x)[0] = b;
            image.at<cv::Vec4b>(y, x)[1] = g;
            image.at<cv::Vec4b>(y, x)[2] = r;
            image.at<cv::Vec4b>(y, x)[3] = 255;
        }
    }
    
    return image;
}

cv::Mat preprocessImageForArUco(const cv::Mat& image, float binaryThreshold) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, gray, binaryThreshold, 255, cv::THRESH_BINARY);
    return gray;
}

bool detectArUcoMarkers(
    const cv::Mat& image,
    const cv::Ptr<cv::aruco::Dictionary>& dictionary,
    const cv::Ptr<cv::aruco::DetectorParameters>& parameters,
    std::vector<std::vector<cv::Point2f>>& markerCorners,
    std::vector<int>& markerIds) {
    
    cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters);
    return !markerIds.empty();
}

std::vector<std::vector<float>> calculateMarkerCorners3D(
    const pcl::RangeImage& rangeImage,
    const std::vector<std::vector<cv::Point2f>>& markerCorners,
    const Eigen::Matrix3f& rotMatrix,
    const Eigen::Vector3f& position) {
    
    std::vector<std::vector<float>> corners3D;
    Eigen::Matrix3f R_o;
    R_o << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    Eigen::Matrix3f R_inv = R_o.inverse();
    Eigen::Vector3f translation(1.0, 0, 0);
    Eigen::Vector3f translationBack = -R_inv * translation;
    
    for (const auto& corners : markerCorners) {
        for (const auto& corner : corners) {
            for (int mm = 0; mm < 4; mm++) {
                int x = static_cast<int>(std::round(corner.x));
                int yUp = static_cast<int>(std::round(corner.y + mm));
                int yDown = static_cast<int>(std::round(corner.y - mm));
                
                if (rangeImage.isObserved(x, yUp) &&
                    rangeImage.isObserved(x, yDown)) {
                    
                    pcl::PointWithRange pointUp, pointDown;
                    rangeImage.calculate3DPoint(x, yUp, pointUp);
                    rangeImage.calculate3DPoint(x, yDown, pointDown);
                    
                    float ratio = pointDown.range / pointUp.range;
                    Eigen::Vector3f tempVertex(
                        pointUp.x * (1/(1+ratio)) + pointDown.x * (ratio/(1+ratio)),
                        pointUp.y * (1/(1+ratio)) + pointDown.y * (ratio/(1+ratio)),
                        pointUp.z * (1/(1+ratio)) + pointDown.z * (ratio/(1+ratio))
                    );
                    
                    Eigen::Vector3f finalVertex = R_inv * tempVertex + translationBack;
                    finalVertex = rotMatrix * finalVertex + position;
                    
                    std::vector<float> finalVertexStd = {
                        finalVertex[0], finalVertex[1], finalVertex[2]
                    };
                    corners3D.push_back(finalVertexStd);
                    break;
                }
            }
        }
    }
    
    return corners3D;
}
