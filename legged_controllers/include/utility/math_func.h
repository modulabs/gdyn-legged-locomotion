#pragma once

#include <utility/math_func.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

Vector3d logR(const Matrix3d& R);

Matrix3d skew(const Vector3d& v);

class Pose
{
public:
  const Pose operator*(const Pose& a) const;
  const Vector3d operator*(const Vector3d& b) const;

  Vector3d     _pos;
  Quaterniond  _rot_quat;
};

class Twist
{
public:
  void setZero();

  Vector3d _linear;
  Vector3d _angular;
};
