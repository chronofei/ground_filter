// C/C++
#include <thread>
#include <mutex>
#include <atomic>
#include <iomanip>

// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// project
#include "ground_filter/ground_filter/ground_filter.hpp"

using namespace ground_filter;

class GroundFilterROSInterface
{
public:
GroundFilterROSInterface(ros::NodeHandle& nh)
:nh_(nh)
{
	cloud_sub_ = nh_.subscribe(ParseConfig::GetInstance()->getConfig("ros_interface")["cloud_sub_topic"].asString(),
							   1,
							   &GroundFilterROSInterface::CloudCallBack,
							   this);

	cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
							   	ParseConfig::GetInstance()->getConfig("ros_interface")["cloud_pub_topic"].asString(), 
							   	1);

	ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
							   	ParseConfig::GetInstance()->getConfig("ros_interface")["ground_cloud_pub_topic"].asString(), 
							   	1);
	nonground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
							   	   ParseConfig::GetInstance()->getConfig("ros_interface")["nonground_cloud_pub_topic"].asString(), 
							   	   1);

	system_is_ok_ = true;
	main_thread_  = std::thread(&GroundFilterROSInterface::MaintThread, this); 
}

~GroundFilterROSInterface()
{
	system_is_ok_ = false;
	if (main_thread_.joinable())
		main_thread_.join();
}

void CloudCallBack(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	CloudData cloud_data;
	cloud_data.time = msg->header.stamp.toSec();
	pcl::fromROSMsg(*msg, cloud_data.cloud);
	ground_filter_.ReadScan(cloud_data);
}

private:
void MaintThread()
{
	sensor_msgs::PointCloud2 cloud_msg;
	sensor_msgs::PointCloud2 ground_msg;
	sensor_msgs::PointCloud2 nonground_msg;

	ros::Rate loop(1000);

	while (ros::ok() && system_is_ok_)
	{
		if (ground_filter_.GetCompoundData(compound_data_))
		{
			if (cloud_pub_.getNumSubscribers() > 0)
			{
				pcl::toROSMsg(compound_data_.cloud, cloud_msg);
				cloud_msg.header.stamp = ros::Time().fromSec(compound_data_.time);
				cloud_msg.header.frame_id = "base_link";
				cloud_pub_.publish(cloud_msg);
			}

			if (ground_pub_.getNumSubscribers() > 0)
			{
				pcl::toROSMsg(compound_data_.ground, ground_msg);
				ground_msg.header.stamp = ros::Time().fromSec(compound_data_.time);
				ground_msg.header.frame_id = "base_link";
				ground_pub_.publish(ground_msg);
			}

			if (nonground_pub_.getNumSubscribers() > 0)
			{
				pcl::toROSMsg(compound_data_.nonground, nonground_msg);
				nonground_msg.header.stamp = ros::Time().fromSec(compound_data_.time);
				nonground_msg.header.frame_id = "base_link";
				nonground_pub_.publish(nonground_msg);
			}
		}

		loop.sleep();
	}
}

private:
	ros::NodeHandle nh_;

	ros::Subscriber cloud_sub_;
	ros::Publisher	cloud_pub_;
	ros::Publisher 	ground_pub_;
	ros::Publisher 	nonground_pub_;

	std::atomic<bool> system_is_ok_;
	std::thread main_thread_;

	CompoundData compound_data_;

	GroundFilter ground_filter_;

}; // end class GroundFilterROSInterface

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ground_filter_node");
  ros::NodeHandle nh;

  GroundFilterROSInterface ground_filter_node(nh);

  ros::AsyncSpinner spinner(8);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}