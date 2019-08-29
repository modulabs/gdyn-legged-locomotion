#pragma once

#include <utility/math_func.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;

Vector3d logR(const Matrix3d& R);

Matrix3d skew(const Vector3d& v);