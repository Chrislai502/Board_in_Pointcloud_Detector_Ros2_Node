#ifndef BOARD_DETECTOR_PCD_UTILS_HPP_
#define BOARD_DETECTOR_PCD_UTILS_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/console/print.h>
#include <pcl/for_each_type.h>
#include <vector>

// Custom point type definition
struct PointXYZIR
{
  PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])
  int ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Ensure proper alignment
} EIGEN_ALIGN16;  // Force SSE padding for correct memory alignment

// Register the custom point type
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int, ring, ring)
)

// Declaration of conversion functions
template <typename PointT>
void toPCLPointCloud2(const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg);

template <typename PointT>
void fromPCLPointCloud2(const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud,
                        const pcl::MsgFieldMap& field_map);

#endif  // BOARD_DETECTOR_PCD_UTILS_HPP_
