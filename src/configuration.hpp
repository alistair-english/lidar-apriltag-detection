#pragma once

#include "yaml-cpp/yaml.h"
#include <string>

struct Configuration {
    std::string filename;
    std::string tag_family;
    float marker_size;
    float tolerance;
    float ratio_threshold;
    float buffer;
    float intensity_threshold;
    float EuclideanTolerance;
    float ang_resolution;
    float binary_threshold;
    float norm_radius;
    float gradient_radius;

    // Derived values
    float cuboid_dia_min;
    float cuboid_dia_max;
    float cuboid_area_min;
    float cuboid_area_max;
};

// Load configuration from a YAML file
Configuration load_configuration(const std::string &config_file = "config.yaml");
