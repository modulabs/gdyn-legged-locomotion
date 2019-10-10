#pragma once

#include <Eigen/Eigen>

class Leg
{
public:
    Eigen::Vector3d _pos, _pos_d;
	Eigen::Vector3d _vel, _vel_d;
	Eigen::Vector3d _force;
	Eigen::Vector3d _trq;
    
};