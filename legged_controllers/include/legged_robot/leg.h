#pragma once

#include <Eigen/Eigen>



class Leg
{
public:
  // parameter
  double          _friction_coeff; // friction coefficient between ground and robot's foot

  bool            _contact_state;

  // joint space variable
  Eigen::Vector3d _joint_pos, _joint_pos_d;
  Eigen::Vector3d _joint_vel, _joint_vel_d;
  Eigen::Vector3d _joint_trq;




  // foot variable
  Eigen::Vector3d _foot_pos, _foot_pos_d;
  Eigen::Vector3d _foot_vel, _foot_vel_d;

  Eigen::Vector3d _foot_force, _foot_force_prev;

  Eigen::Vector3d _trq_grav;

};
