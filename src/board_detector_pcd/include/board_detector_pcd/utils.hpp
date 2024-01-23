#ifndef BOARD_DETECTOR_PCD_UTILS_HPP_
#define BOARD_DETECTOR_PCD_UTILS_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

// Forward declaration of the PointXYZR struct
struct Points {
    float x, y, z, azimuth;
    uint16_t ring;

    // Constructor
    Points(float x, float y, float z, float azimuth, uint16_t ring);
};

// Parses a PointCloud2 ROS2 message into a vector of PointXYZR
std::vector<PointXYZR> parsePointCloud2(const sensor_msgs::msg::PointCloud2& msg);

// Converts a vector of PointXYZR back into a PointCloud2 ROS2 message
sensor_msgs::msg::PointCloud2 convertToPointCloud2(const std::vector<PointXYZR>& points);

// // Declaration of conversion functions
// template <typename PointT>
// void toPCLPointCloud2(const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg);

// template <typename PointT>
// void fromPCLPointCloud2(const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
//                         const pcl::MsgFieldMap& field_map);

#endif  // BOARD_DETECTOR_PCD_UTILS_HPP_
