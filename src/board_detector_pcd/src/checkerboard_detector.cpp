#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include "board_detector_pcd/utils.hpp"

// Defining some types
// using PointT = PointXYZIR;
// using PointCloudT = pcl::PointCloud<PointT>;

// ROS Node
class CheckerboardDetector : public rclcpp:Node {
  public:
    CheckerboardDetector() : Node("checkerboard_detector") {
      // Subscribe to the input point cloud topic
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/input/point_cloud", 10, std::bind(&CheckerboardDetector::pointCloudCallback, this, std::placeholders::_1));
      
      // Publisher for output processed point cloud
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("", 10);
    }

  private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // Convert ROS2 PointCloud2 message to PCL PointCloud
      PointCloudT::Ptr cloud(new PointCloudT);
      pcl::fromROSMsg(*msg, *cloud);

      // Process the point cloud to detect and track the checkerboard
      auto processed_cloud = processPointCloud(cloud);

      // Convert processed PCL PointCloud back to ROS2 message
      sensor_msgs::msg::PointCloud2 output_msg;
      output_msg.header.stamp = this->get_clock()->now();
      output_msg.header.frame_id = "processrd_cloud";

      // Publish the processed point cloud
      publisher_-> publish(output_msg);
    }


    PointCloudT::Ptr processPointCloud(const PointCloudT::Ptr& cloud) {
        // 1. Checkerboard Point Selection
        // a. Compute Pitch Angle and Segment Rings
        std::vector<PointCloudT::Ptr> segmented_rings = segmentRings(cloud);

        // b. Cluster Points within Each Ring using PCA and DBSCAN
        std::vector<PointCloudT::Ptr> clustered_points = clusterPoints(segmented_rings);

        // 2. Checkerboard Feature Extraction
        // a. RANSAC-based Plane Fitting
        PointCloudT::Ptr inlier_points(new PointCloudT);
        Eigen::VectorXf plane_coefficients;
        fitPlane(clustered_points, inlier_points, plane_coefficients);

        // b. Project Points onto Plane
        PointCloudT::Ptr projected_points = projectPoints(inlier_points, plane_coefficients);

        // c. Extract Edge Points
        PointCloudT::Ptr edge_points = extractEdgePoints(projected_points);

        // 3. Checkerboard Tracking
        // a. Bounding Box Creation and Tracking
        PointCloudT::Ptr tracked_points = trackCheckerboard(edge_points);

        return tracked_points;
    }
}
