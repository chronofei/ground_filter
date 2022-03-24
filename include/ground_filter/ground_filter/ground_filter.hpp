#ifndef GROUND_FILTER_GROUND_FILTER_GROUND_FILTER_HPP_
#define GROUND_FILTER_GROUND_FILTER_GROUND_FILTER_HPP_

// C/C++
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/statistical_outlier_removal.h>

// project
#include "ground_filter/models/ground_filter_base.hpp"
#include "ground_filter/models/csf/CSF.hpp"

#include "ground_filter/common/common.hpp"
#include "ground_filter/tools/tools.hpp"
#include "ground_filter/sensor_data/sensor_data.hpp"

namespace ground_filter 
{

class GroundFilter
{
public:
  GroundFilter();
  ~GroundFilter();

  bool ReadScan(const CloudData& data);

  bool GetCompoundData(CompoundData& data);

private:
  void GroundFilterThread();

private:
  std::mutex                        new_scan_mutex_;
  std::deque<CloudData>             cloud_data_buff_;

  std::deque<CloudData>             current_cloud_data_buff_;

  std::mutex                        new_compound_mutex_;
  std::deque<CompoundData>          compound_data_buff_;

  Eigen::Matrix4d                   lidar_to_imu_transform_;

  std::atomic<bool>                 system_is_ok_;
  std::thread                       ground_filter_thread_;

  std::shared_ptr<GroundFilterBase> ground_filter_base_;

}; // end class GroundFilter

}  // end namespace ground_filter

#endif