/*
  Author: Modulabs
  File Name: math_func.cpp
*/

#include "legged_robot_math/math_func.h"


Vector3d logR(const Matrix3d& R)
{
	double w1, w2, w3;
	double trR = R.trace();

	// [w] = Log(R) when tr(R) = -1
	//if(trR + 1 < _EPS)
	if(trR + 1 < 2.2204E-16)
	{
		w1 = sqrt( (R(0,0) + 1.0)/2.0 );
		w2 = sqrt( (R(1,1) + 1.0)/2.0 );
		w3 = sqrt( (R(2,2) + 1.0)/2.0 );
		if( w1 != 0.0 )
		{
			w2 = R(0,1)/(2.0*w1);
			w3 = R(0,2)/(2.0*w1);
		}
		else if ( w2 != 0.0 )
		{
			w1 = R(0,1)/(2.0*w2);
			w3 = R(1,2)/(2.0*w2);
		}
		else if ( w3 != 0.0 )
		{
			w1 = R(0,2)/(2.0*w3);
			w2 = R(1,2)/(2.0*w3);
		}
		else
		{
			//Error!
			return Vector3d::Zero();
		}
		return Vector3d( w1, w2, w3);
	}

	double d = 0.5 * (R(0,0) + R(1,1) + R(2,2) - 1.0);
	if (d > 1.0) { d = (double)1.0; }
	if (d < -1.0) { d = (double)-1.0; }

	double theta = acos(d);
	if ( fabs(theta) < 1e-6 ) return Vector3d(0.0, 0.0, 0.0);

	double cof = theta / (2.0 * sin(theta));

	return Vector3d(cof * (R(2,1) - R(1,2)), cof * (R(0,2) - R(2,0)), cof * (R(1,0) - R(0,1)));
}

Matrix3d skew(const Vector3d& v)
{
    Matrix3d m;
    m << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;

    return m;
}

void Pose::setIdentity()
{
	_pos.setZero();
	_rot_quat.setIdentity();
}

const Pose Pose::operator*(const Pose &b) const
{
  Pose a;

  a._pos = _pos + _rot_quat * b._pos;
  a._rot_quat = _rot_quat * b._rot_quat;

  return a;
}

const Eigen::Vector3d Pose::operator*(const Eigen::Vector3d &b) const
{
  Eigen::Vector3d a;

  a = _pos + _rot_quat * b;

  return a;
}

void Twist::setZero()
{
  _linear.setZero();
  _angular.setIdentity();
}
