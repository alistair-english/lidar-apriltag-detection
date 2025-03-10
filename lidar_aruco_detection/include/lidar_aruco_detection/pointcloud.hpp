#pragma once

#include <cmath>
#include <iostream>
#include <memory>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vector>

namespace lidar_aruco_detection {

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;

} // namespace lidar_aruco_detection
