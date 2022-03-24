#include "ground_filter/ground_filter/ground_filter.hpp"

namespace ground_filter
{

GroundFilter::GroundFilter()
{
  std::string method = ParseConfig::GetInstance()->getConfig("ground_filter")["ground_filter_method"].asString();
  if (method == "CSF")
    ground_filter_base_ = std::make_shared<CSF>();
  // else if (method == "LEGO_LOAM")
  //   ground_filter_base_ = std::make_shared<>();
  // else if (method == "LineFit")
  //   ground_filter_base_ = std::make_shared<LineFit>();
  else
  {
    std::cout << "Undefined ground filtering method" << std::endl;
    exit(0);
  }

  Vector6d lidar_to_imu_pose = Vector6d::Zero();
  lidar_to_imu_pose(0,0) = DEG_TO_PI * ParseConfig::GetInstance()->getConfig("ground_filter")["lidar_to_imu_transform"]["roll"].asDouble();
  lidar_to_imu_pose(1,0) = DEG_TO_PI * ParseConfig::GetInstance()->getConfig("ground_filter")["lidar_to_imu_transform"]["pitch"].asDouble();
  lidar_to_imu_pose(2,0) = DEG_TO_PI * ParseConfig::GetInstance()->getConfig("ground_filter")["lidar_to_imu_transform"]["yaw"].asDouble();
  lidar_to_imu_pose(3,0) =             ParseConfig::GetInstance()->getConfig("ground_filter")["lidar_to_imu_transform"]["x"].asDouble();
  lidar_to_imu_pose(4,0) =             ParseConfig::GetInstance()->getConfig("ground_filter")["lidar_to_imu_transform"]["y"].asDouble();
  lidar_to_imu_pose(5,0) =             ParseConfig::GetInstance()->getConfig("ground_filter")["lidar_to_imu_transform"]["z"].asDouble();
  lidar_to_imu_transform_ = Vector6dToEigenPose(lidar_to_imu_pose);

  system_is_ok_ = true;
  ground_filter_thread_ = std::thread(&GroundFilter::GroundFilterThread, this);
}

GroundFilter::~GroundFilter()
{
  system_is_ok_ = false;

  if (ground_filter_thread_.joinable())
    ground_filter_thread_.join();
}

bool GroundFilter::ReadScan(const CloudData& data)
{
  std::lock_guard<std::mutex> locker(new_scan_mutex_);
  cloud_data_buff_.emplace_back(data);
  return true;
}

void GroundFilter::GroundFilterThread()
{
  while (system_is_ok_)
  {
    std::unique_lock<std::mutex> lockerScan(new_scan_mutex_);
    if (cloud_data_buff_.size() > 0)
    {
      current_cloud_data_buff_.insert(current_cloud_data_buff_.end(),
                                              cloud_data_buff_.begin(),
                                              cloud_data_buff_.end());
      cloud_data_buff_.clear();
    }
    lockerScan.unlock();

    if (current_cloud_data_buff_.empty())
      continue;

    CompoundData result_data;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(current_cloud_data_buff_.front().cloud, current_cloud_data_buff_.front().cloud, indices);
    pcl::transformPointCloud(current_cloud_data_buff_.front().cloud, result_data.cloud, lidar_to_imu_transform_.cast<float>());

    if (ground_filter_base_->Filter(result_data))
    {
      std::unique_lock<std::mutex> lockerCompoundData(new_compound_mutex_);
      compound_data_buff_.emplace_back(result_data);
      lockerCompoundData.unlock();
    }

    current_cloud_data_buff_.pop_front();
  }
}

bool GroundFilter::GetCompoundData(CompoundData& data)
{
  std::lock_guard<std::mutex> lockerCompoundData(new_compound_mutex_);

  if (compound_data_buff_.empty())
    return false;
  
  data = compound_data_buff_.front();
  compound_data_buff_.pop_front();
  return true;
}

} //namespace ground_filter