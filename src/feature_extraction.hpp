#pragma once

#include <pcl/features/intensity_gradient.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

/**
 * Estimate surface normals for a point cloud
 *
 * @param cloud Input point cloud
 * @param radius Search radius for normal estimation
 * @return Estimated surface normals
 */
pcl::PointCloud<pcl::Normal>::Ptr estimate_normals(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float radius);

/**
 * Estimate intensity gradients for a point cloud
 *
 * @param cloud Input point cloud
 * @param normals Surface normals
 * @param radius Search radius for gradient estimation
 * @return Estimated intensity gradients
 */
pcl::PointCloud<pcl::IntensityGradient>::Ptr estimate_intensity_gradients(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals, float radius
);

/**
 * Extract feature points based on intensity gradient magnitude
 *
 * @param cloud Input point cloud
 * @param gradient Intensity gradients
 * @param intensity_threshold Threshold for feature extraction
 * @return Feature point cloud
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr extract_feature_points(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
    const pcl::PointCloud<pcl::IntensityGradient>::Ptr &gradient,
    float intensity_threshold
);
