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

struct PointXYZIR
{
  PCL_ADD_POINT4D;                  // Preferred way of adding a XYZ+padding
  int ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // Enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int, ring, ring)
)

// Defining some types
using PointT = PointXYZIR;
using PointCloudT = pcl::PointCloud<PointT>;

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
      // Placeholder for processed cloud, replace with actual processing logic
      PointCloudT::Ptr processed_cloud(new PointCloudT);

      // Checkerboard Point Selection
      Get the rings for the PointCloud

      // Cluster the Points within each ring using PCA and DBSCAN

    }
}
