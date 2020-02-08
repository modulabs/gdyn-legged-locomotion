/*
  Author: Modulabs
  File Name: legged_robot_teleop_keyboard.h
*/

#pragma once

#include <termios.h>

#include <ros/ros.h>

#include "legged_robot_msgs/UICommand.h"


class LeggedRobotTeleopKeyboard
{
public:
  LeggedRobotTeleopKeyboard();
  ~LeggedRobotTeleopKeyboard();

  void setGoal(char ch);
  bool sendCommand(std::string mainCommand, std::string subCommand = std::string(), long int paramInt64 = 0, double paramFloat64 = 0.0);

  void printText();
  void restoreTerminalSettings(void);
  void disableWaitingForEnter(void);

private:
  ros::NodeHandle _nodeHandle;
  ros::ServiceClient _uiCommandSrv;
  struct termios oldt_;
};
