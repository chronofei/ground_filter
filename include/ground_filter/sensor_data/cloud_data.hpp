#ifndef GROUND_FILTER_SENSOR_DATA_CLOUD_DATA_HPP_
#define GROUND_FILTER_SENSOR_DATA_CLOUD_DATA_HPP_

// project
#include "ground_filter/common/common.hpp"

namespace ground_filter
{

class CloudData
{

public:
  double time = 0.0;
  PointCloud cloud;

}; // end class CloudData

}  // end namespace ground_filter

#endif