/*
  Author: Modulabs
  File Name: legged_robot_teleop_joystick.cpp
*/

#include "legged_robot_teleop/legged_robot_teleop_joystick.h"


LeggedRobotTeleopJoystick::LeggedRobotTeleopJoystick()
: _nodeHandle("")
{
  _joyCommandSub = _nodeHandle.subscribe("joy", 10, &LeggedRobotTeleopJoystick::joyCommandCallback, this);
  _uiCommandSrv = _nodeHandle.serviceClient<legged_robot_msgs::UICommand>("/hyq/main_controller/ui_command");
}

LeggedRobotTeleopJoystick::~LeggedRobotTeleopJoystick()
{
  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void LeggedRobotTeleopJoystick::joyCommandCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->axes.at(1) >= 0.9) setGoal("x+");
  else if(msg->axes.at(1) <= -0.9) setGoal("x-");
  else if(msg->axes.at(0) >=  0.9) setGoal("y+");
  else if(msg->axes.at(0) <= -0.9) setGoal("y-");
  else if(msg->buttons.at(3) == 1) setGoal("z+");
  else if(msg->buttons.at(0) == 1) setGoal("z-");
  else if(msg->buttons.at(5) == 1) setGoal("w+");
  else if(msg->buttons.at(4) == 1) setGoal("w-");
  else if(msg->buttons.at(2) == 1) setGoal("a+");
  else if(msg->buttons.at(1) == 1) setGoal("a-");
}

void LeggedRobotTeleopJoystick::setGoal(const char* str)
{
  if(str == "x+")
  {
    sendCommand("change_controller", "VSD");
  }
  else if(str == "x-")
  {
    sendCommand("change_controller", "BalQP");
  }
  else if(str == "y+")
  {
    sendCommand("change_controller", "BalMPC");
  }
  else if(str == "y-")
  {
    sendCommand("change_controller", "BalMPCWB");
  }
  else if(str == "z+")
  {
    sendCommand("move_body", "", 0);
  }
  else if(str == "z-")
  {
    sendCommand("move_body", "", 1);
  }
  else if(str == "w+")
  {
    sendCommand("move_body", "", 2);
  }
  else if(str == "w-")
  {
    sendCommand("move_body", "", 3);
  }
  else if(str == "a+")
  {
    sendCommand("Order", "", 0);
  }
  else if(str == "a-")
  {
    sendCommand("Order", "", 1);
  }
}

bool LeggedRobotTeleopJoystick::sendCommand(std::string mainCommand, std::string subCommand, long int paramInt64, double paramFloat64)
{
  legged_robot_msgs::UICommand srv;
  srv.request.main_command = mainCommand;
  srv.request.sub_command = subCommand;
  srv.request.param_int64 = paramInt64;
  srv.request.param_float64 = paramFloat64;
  if (_uiCommandSrv.call(srv))
  {
    if (srv.response.result)
      return true;
  }

  ROS_INFO("Failed to send a command");
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "legged_robot_teleop_joystick");
  LeggedRobotTeleopJoystick leggedRobotTeleopJoystick;

  ros::spin();

  return 0;
}
