#pragma once

#include <cmath>
#include <memory>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;

inline PointCloud::Ptr load_cloud(const std::string &filename) {
    auto cloud = std::make_shared<PointCloud>();

    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename.c_str(), *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", filename.c_str());
        return nullptr;
    }

    return cloud;
}
