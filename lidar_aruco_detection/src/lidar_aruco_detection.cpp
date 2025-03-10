#include "lidar_aruco_detection/lidar_aruco_detection.hpp"

#include <lidar_aruco_detection/euclidean_clustering.hpp>
#include <lidar_aruco_detection/image_marker_detection.hpp>
#include <lidar_aruco_detection/image_projection.hpp>
#include <lidar_aruco_detection/intensity_gradient_filtering.hpp>
#include <lidar_aruco_detection/oriented_bounding_box.hpp>
#include <lidar_aruco_detection/pointcloud.hpp>
#include <lidar_aruco_detection/visualisation.hpp>

namespace lidar_aruco_detection {

std::vector<MarkerDetection> detect_markers_using_intensity_gradient_clustering(
    const PointCloud::Ptr &cloud,
    const IntensityGradientConfiguration &config,
    std::shared_ptr<IntensityGradientDebugData> debug_data
) {
    // Step 1: Estimate normals
    const auto normals = estimate_normals(cloud, config.normal_estimation_radius);

    // Step 2: Estimate intensity gradients
    const auto gradients = estimate_intensity_gradient(cloud, normals, config.gradient_estimation_radius);

    // Step 3: Calculate intensity threshold and extract significant points
    float threshold = calculate_intensity_threshold(gradients, config.intensity_threshold_percentile);
    auto significant_points = extract_significant_gradient_points(cloud, gradients, threshold);
    if (debug_data) {
        debug_data->significant_gradient_points = significant_points;
    }

    // Step 4: Perform clustering
    auto clusters = extract_euclidean_clusters(
        significant_points, config.cluster_tolerance, config.min_cluster_size, config.max_cluster_size
    );
    if (debug_data) {
        debug_data->euclidean_clusters = clusters;
    }

    // Step 5: Calculate and filter oriented bounding boxes
    auto boxes = calculate_oriented_bounding_boxes(clusters);
    if (debug_data) {
        debug_data->all_obbs = boxes;
    }

    auto filtered_boxes = filter_obbs(boxes, config.min_diagonal, config.max_diagonal, config.max_aspect_ratio);
    if (debug_data) {
        debug_data->filtered_obbs = filtered_boxes;
    }

    // Convert angular resolution from degrees to radians
    float angular_resolution = pcl::deg2rad(config.angular_resolution_deg);
    std::vector<MarkerDetection> marker_detections;

    for (const auto &obb : filtered_boxes) {
        const auto points_in_box = extract_points_in_obb(cloud, obb);
        if (debug_data) {
            debug_data->obb_filtered_points.push_back(points_in_box);
        }

        const auto [transformed_cloud, applied_transform] = transform_cloud_for_imaging(points_in_box, obb);
        const auto range_image = create_range_image_from_cloud(transformed_cloud, angular_resolution);
        const auto intensity_image =
            create_intensity_image_from_cloud(transformed_cloud, angular_resolution, range_image);
        const auto range_cv_image = convert_range_image_to_cv_mat(range_image);

        cv::Mat threshold_intensity_image;
        cv::threshold(intensity_image, threshold_intensity_image, config.intensity_threshold, 255, cv::THRESH_BINARY);

        std::vector<ImageMarkerDetection> detections =
            detect_markers(threshold_intensity_image, config.dictionary_name);

        if (debug_data) {
            MarkerSearchDebugData search_debug;
            search_debug.range_image = range_image;
            search_debug.intensity_image = intensity_image;
            search_debug.thresholded_intensity_image = threshold_intensity_image;
            search_debug.image_marker_detections = detections;
            debug_data->obb_marker_searches.push_back(search_debug);
        }

        for (const auto &detection : detections) {
            std::vector<Eigen::Vector3f> corner_points_3d =
                convert_marker_points_to_3d(detection.corners, range_image, applied_transform);

            auto [marker_position, marker_orientation] = calculate_marker_pose(corner_points_3d);

            MarkerDetection marker;
            marker.id = detection.id;
            marker.position = marker_position;
            marker.orientation = marker_orientation;
            marker_detections.push_back(marker);

            if (debug_data) {
                MarkerDetectionDebugData marker_debug;
                marker_debug.id = detection.id;
                marker_debug.corner_points = corner_points_3d;
                debug_data->obb_marker_searches.back().marker_detections.push_back(marker_debug);
            }
        }
    }

    return marker_detections;
}

} // namespace lidar_aruco_detection
