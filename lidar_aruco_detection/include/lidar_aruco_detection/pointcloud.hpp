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

// Print all available fields in a PCD file
inline void print_pcd_fields(const pcl::PCLPointCloud2 &cloud_blob) {
    std::cout << "=== PCD File Information ===" << std::endl;
    std::cout << "Width: " << cloud_blob.width << std::endl;
    std::cout << "Height: " << cloud_blob.height << std::endl;
    std::cout << "Point count: " << cloud_blob.width * cloud_blob.height << std::endl;
    std::cout << "Is binary: " << (cloud_blob.is_bigendian ? "Yes" : "No") << std::endl;
    std::cout << "Is dense: " << (cloud_blob.is_dense ? "Yes" : "No") << std::endl;
    std::cout << "Data size: " << cloud_blob.data.size() << " bytes" << std::endl;

    std::cout << "\nFields (" << cloud_blob.fields.size() << "):" << std::endl;
    for (const auto &field : cloud_blob.fields) {
        std::cout << "  - " << field.name << " (offset: " << field.offset
                  << ", datatype: " << static_cast<int>(field.datatype) << ", count: " << field.count << ")"
                  << std::endl;
    }
    std::cout << "===========================" << std::endl;
}

inline PointCloud::Ptr load_cloud(const std::string &filename, const float scale = 1.0) {
    // Load the cloud using PCLPointCloud2
    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPCDFile(filename.c_str(), cloud_blob) == -1) {
        PCL_ERROR("Couldn't read file %s\n", filename.c_str());
        return nullptr;
    }

    print_pcd_fields(cloud_blob);

    auto cloud = std::make_shared<PointCloud>();
    cloud->resize(cloud_blob.width * cloud_blob.height);
    cloud->width = cloud_blob.width;
    cloud->height = cloud_blob.height;
    cloud->is_dense = cloud_blob.is_dense;

    // Find field offsets
    int x_idx = -1, y_idx = -1, z_idx = -1, intensity_idx = -1;
    for (size_t i = 0; i < cloud_blob.fields.size(); ++i) {
        if (cloud_blob.fields[i].name == "x")
            x_idx = i;
        else if (cloud_blob.fields[i].name == "y")
            y_idx = i;
        else if (cloud_blob.fields[i].name == "z")
            z_idx = i;
        else if (cloud_blob.fields[i].name == "intensity")
            intensity_idx = i;
    }

    if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
        std::cerr << "Error: PCD file missing x, y, or z fields" << std::endl;
        return nullptr;
    }

    // Helper function to extract a value based on datatype
    auto extract_value = [](const uint8_t *data, uint8_t datatype) -> float {
        switch (datatype) {
        case 1:
            return static_cast<float>(*reinterpret_cast<const int8_t *>(data));
        case 2:
            return static_cast<float>(*reinterpret_cast<const uint8_t *>(data));
        case 3:
            return static_cast<float>(*reinterpret_cast<const int16_t *>(data));
        case 4:
            return static_cast<float>(*reinterpret_cast<const uint16_t *>(data));
        case 5:
            return static_cast<float>(*reinterpret_cast<const int32_t *>(data));
        case 6:
            return static_cast<float>(*reinterpret_cast<const uint32_t *>(data));
        case 7:
            return *reinterpret_cast<const float *>(data);
        case 8:
            return static_cast<float>(*reinterpret_cast<const double *>(data));
        default:
            return 0.0f;
        }
    };

    // Create a temporary cloud to hold valid points
    PointCloud temp_cloud;
    temp_cloud.reserve(cloud_blob.width * cloud_blob.height);

    // Extract all fields manually
    for (size_t i = 0; i < cloud_blob.width * cloud_blob.height; ++i) {
        const uint8_t *point_data = &cloud_blob.data[i * cloud_blob.point_step];

        // Extract x, y, z
        const auto &x_field = cloud_blob.fields[x_idx];
        const auto &y_field = cloud_blob.fields[y_idx];
        const auto &z_field = cloud_blob.fields[z_idx];

        float x = extract_value(point_data + x_field.offset, x_field.datatype) / scale;
        float y = extract_value(point_data + y_field.offset, y_field.datatype) / scale;
        float z = extract_value(point_data + z_field.offset, z_field.datatype) / scale;

        // Skip points where x, y, and z are all zero
        if (x == 0.0f && y == 0.0f && z == 0.0f) {
            continue;
        }

        // Create a new point
        Point point;
        point.x = x;
        point.y = y;
        point.z = z;

        // Extract intensity if available
        if (intensity_idx != -1) {
            const auto &intensity_field = cloud_blob.fields[intensity_idx];
            point.intensity = extract_value(point_data + intensity_field.offset, intensity_field.datatype);
        } else {
            point.intensity = 0.0f;
        }

        // Add the point to our temporary cloud
        temp_cloud.push_back(point);
    }

    // Copy the valid points to the output cloud
    *cloud = temp_cloud;
    cloud->width = temp_cloud.size();
    cloud->height = 1; // Unorganized point cloud
    cloud->is_dense = true;

    std::cout << "Removed " << (cloud_blob.width * cloud_blob.height - cloud->size())
              << " zero-coordinate points from the cloud." << std::endl;

    return cloud;
}

} // namespace lidar_aruco_detection