#ifndef GROUND_FILTER_COMMON_MATH_HPP_
#define GROUND_FILTER_COMMON_MATH_HPP_

// C/C++
#include <cmath>

// Eigen
#include <Eigen/Core>

// project
#include "ground_filter/common/type.hpp"
#include "ground_filter/tools/tic_toc.hpp"

namespace ground_filter
{
  
inline double NormalizeAngle(const double& angle)
{
  double temp = angle;

  while (temp < -M_PI)
    temp += 2*M_PI;

  while (temp >= M_PI)
    temp -= 2*M_PI;

  return temp;
}

/**
 * 一个旋转矩阵可以得到多组欧拉角度值。
 * Eigen中的eulerAngles()方法，计算的欧拉角度值可能不是我们想要的，但是，旋转矩阵若是采用Eigen中的轴角方法即AngleAxised得到的，
 * 则可以采用此方法准确回复，若是任意给的旋转矩阵，则存在问题。
 * 
 * 下面的计算方法来自网页https://zhuanlan.zhihu.com/p/55790406 
 **/
inline void ToEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
  double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr_cosp, cosr_cosp);
  
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
  pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
  pitch = asin(sinp);
  
  double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny_cosp, cosy_cosp);
}

inline Vector6d EigenPoseToVector6d(const Eigen::Matrix4d& pose)
{
  Vector6d current_pose = Vector6d::Zero();

  Eigen::Quaterniond q(pose.block<3,3>(0,0));
  double roll  = 0.0;
  double pitch = 0.0;
  double yaw   = 0.0;

  ToEulerAngle(q, roll, pitch, yaw);

  current_pose(0,0) = NormalizeAngle(roll);
  current_pose(1,0) = NormalizeAngle(pitch);
  current_pose(2,0) = NormalizeAngle(yaw);

  current_pose.block<3,1>(3,0) = pose.block<3,1>(0,3);

  return current_pose;
}

inline Eigen::Matrix4d Vector6dToEigenPose(const Vector6d& pose)
{
  Eigen::Matrix4d current_pose = Eigen::Matrix4d::Identity();
  double roll  = NormalizeAngle(pose[0]);
  double pitch = NormalizeAngle(pose[1]);
  double yaw   = NormalizeAngle(pose[2]);

  Eigen::Matrix3d rotX = Eigen::Matrix3d::Identity();
  rotX << 1,              0,              0,
          0, std::cos(roll),-std::sin(roll),
          0, std::sin(roll), std::cos(roll);

  Eigen::Matrix3d rotY = Eigen::Matrix3d::Identity();
  rotY << std::cos(pitch),0,std::sin(pitch),
                        0,1,              0,
         -std::sin(pitch),0,std::cos(pitch);

  Eigen::Matrix3d rotZ = Eigen::Matrix3d::Identity();
  rotZ << std::cos(yaw),-std::sin(yaw),0,
          std::sin(yaw), std::cos(yaw),0,
                      0,             0,1;

  current_pose.block<3,3>(0,0) = rotZ * rotY * rotX;
  current_pose.block<3,1>(0,3) = pose.block<3,1>(3,0);

  return current_pose;
}

inline double SquareDistanceXY(const double& x1, const double& y1,
                               const double& x2, const double& y2)
{
  return (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1);
}

} // end namespace ground_filter

#endif