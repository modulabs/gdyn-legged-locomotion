#pragma once

#include <Eigen/Eigen>
#include <Eigen/Geometry>

class Body
{
public:

  // parameter
  double _mass;
  Eigen::Matrix3d _inertia_com;
  Eigen::Vector3d _pos_body2com;

  // variable
  Eigen::Vector3d _pos, _pos_d, _pos_com, _pos_com_d;
  Eigen::Vector3d _vel, _vel_d, _vel_com, _vel_com_d;
  Eigen::Vector3d _acc_d;

  Eigen::Quaterniond _rot_quat, _rot_quat_d;
  Eigen::Vector3d _rot_vel, _rot_vel_d;
};
