#pragma once

#include <utility/math_func.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;
using Eigen::AngleAxisd;

Vector3d logR(const Matrix3d& R);

Matrix3d skew(const Vector3d& v);


/* Pose class 
 * position, orientation
*/
class Pose
{
public:
  Pose():_pos(Vector3d::Zero()), _rot_quat(Quaterniond::Identity()) {}
  Pose(const Vector3d& pos, const Quaterniond& rot_quat) {_pos = pos; _rot_quat = rot_quat;}

  void setIdentity();

  const Pose operator*(const Pose& a) const;
  const Vector3d operator*(const Vector3d& b) const;

  Vector3d     _pos;
  Quaterniond  _rot_quat;
};

/* Twist class 
 * linear/angular velocity, linear/angular acceleration
*/
class Twist
{
public:
  Twist(): _linear(Vector3d::Zero()), _angular(Vector3d::Zero()) {}
  Twist(const Vector3d& linear, const Vector3d& angular) {_linear = linear; _angular = angular;}

  void setZero();

  Vector3d _linear;
  Vector3d _angular;
};

typedef Twist PoseVel;
typedef Twist PoseAcc;
