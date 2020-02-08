/*
  Author: Modulabs
  File Name: virtual_spring_damper_controller.h
*/

#pragma once

#include <array>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/console.h>

#include "legged_robot_controller/quadruped_robot.h"


class VirtualSpringDamperController
{
public:
  VirtualSpringDamperController() {}

  void init();

  void update(quadruped_robot::QuadrupedRobot& robot, std::array<Eigen::Vector3d, 4>& F_leg);

public:
  std::array<Eigen::Vector3d, 4> _kp_leg;
  std::array<Eigen::Vector3d, 4> _kd_leg;
  std::array<KDL::Frame, 4> _x_offset;
  std::array<KDL::Frame, 4> _xd;
  std::array<KDL::Twist, 4> _xd_dot;

  std::array<Eigen::Vector3d, 4> _p_leg;
  std::array<Eigen::Vector3d, 4> _v_leg;
  std::array<Eigen::Vector3d, 4> _F_leg;

  std::array<Eigen::Vector3d, 4> _G_leg;
};
