#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_aruco_detection {

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;

} // namespace lidar_aruco_detection
