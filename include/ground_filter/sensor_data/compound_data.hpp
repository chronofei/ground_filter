#ifndef GROUND_FILTER_SENSOR_DATA_COMPOUND_DATA_HPP_
#define GROUND_FILTER_SENSOR_DATA_COMPOUND_DATA_HPP_

// C/C++
#include <pcl/io/io.h>

// project
#include "ground_filter/common/common.hpp"

namespace ground_filter
{

class CompoundData
{
public:
  CompoundData& operator=(const CompoundData& other)
  {
    if (this != &other)
    {
      time = other.time;
      pcl::copyPointCloud(other.cloud, cloud);
      pcl::copyPointCloud(other.ground, ground);
      pcl::copyPointCloud(other.nonground, nonground);
      pcl::copyPointCloud(other.cloth, cloth);
    }
    return *this;
  }

public:
  double time = 0.0;
  PointCloud cloud;
  PointCloud ground;
  PointCloud nonground;
  PointCloud cloth;
}; // end class CompoundData

}  // end namespace ground_filter

#endif