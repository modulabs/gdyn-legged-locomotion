#pragma once

#include <utility/math_func.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

Vector3d logR(const Matrix3d& R);

Matrix3d skew(const Vector3d& v);

/* Pose class 
 * position, orientation
*/
class Pose
{
public:
  Pose(const Vector3d& pos, const Vector3d& rot_quat) {_pos = pos; _rot_quat = rot_quat;}

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
  Twist(const Vector3d& linear, const Vector3d& angular) {_linear = linear; _angular = angular;}

  void setZero();

  Vector3d _linear;
  Vector3d _angular;
};
