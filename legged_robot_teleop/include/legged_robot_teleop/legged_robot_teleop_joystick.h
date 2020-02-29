/*
  Author: Modulabs
  File Name: legged_robot_teleop_joystick.h
*/

#pragma once

#include <termios.h>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include "legged_robot_msgs/UICommand.h"


class LeggedRobotTeleopJoystick
{
public:
  LeggedRobotTeleopJoystick();
  ~LeggedRobotTeleopJoystick();

  void joyCommandCallback(const sensor_msgs::Joy::ConstPtr &msg);
  void setGoal(const char *str);
  bool sendCommand(std::string mainCommand, std::string subCommand = std::string(), long int paramInt64 = 0, double paramFloat64 = 0.0);

private:
  ros::NodeHandle _nodeHandle;
  ros::Subscriber _joyCommandSub;
  ros::ServiceClient _uiCommandSrv;
};
