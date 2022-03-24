#ifndef GROUND_FILTER_COMMON_TYPE_HPP_
#define GROUND_FILTER_COMMON_TYPE_HPP_

// Eigen
#include <Eigen/Core>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

namespace pcl
{

struct PointXYZIT
{
  PCL_ADD_POINT4D;
  float intensity;
  float index;
  float time_stamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // end namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (float, index, index)
                                  (float, time_stamp, time_stamp)
)

namespace ground_filter
{

using Point         = pcl::PointXYZI;
using PointCloud    = pcl::PointCloud<Point>;
using PointCloudPtr = PointCloud::Ptr;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

const double DEG_TO_PI = M_PI/180.0;
const double PI_TO_DEG = 180.0/M_PI;

} // end namespace ground_filter

#endif