#pragma once

#include <Eigen/Eigen>

class Body
{
public:
    Eigen::Vector3d _pos, _pos_d;
	Eigen::Vector3d _vel, _vel_d;
    Eigen::Vector3d _acc_d;
};