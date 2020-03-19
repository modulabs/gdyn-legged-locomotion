/*
  Author: Modulabs
  File Name: legged_robot_teleop_keyboard.cpp
*/

#include "legged_robot_teleop/legged_robot_teleop_keyboard.h"


LeggedRobotTeleopKeyboard::LeggedRobotTeleopKeyboard()
: _nodeHandle("")
{
  _uiCommandSrv = _nodeHandle.serviceClient<legged_robot_msgs::UICommand>("/hyq/main_controller/ui_command");
}

LeggedRobotTeleopKeyboard::~LeggedRobotTeleopKeyboard()
{
  if(ros::isStarted()) 
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void LeggedRobotTeleopKeyboard::printText()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Control Legged Robot\n");
  printf("---------------------------\n");
  printf("q: Change control mode to VSD\n");
  printf("w: Change control mode to BalQP\n");
  printf("e: Change control mode to BalMPC\n");
  printf("r: Change control mode to BalMPCWB\n");
  printf("\n");
  printf("a: Body 0\n");
  printf("s: Body 1\n");
  printf("d: Body 2\n");
  printf("f: Body 3\n");
  printf("g: Body 4\n");
  printf("h: Body 5\n");
  printf("j: Body 6\n");
  printf("k: Body 7\n");
  printf("l: Body 8\n");
  printf("\n");
  printf("z: Order 0\n");
  printf("x: Order 1\n");
  printf("c: Order 2\n");
  printf("v: Order 3\n");
  printf("\n");
  printf("0 to quit\n");
  printf("---------------------------\n");
}

void LeggedRobotTeleopKeyboard::setGoal(char ch)
{
  if(ch == 'q' || ch == 'Q')
  {
    sendCommand("change_controller", "VSD");
  }
  else if(ch == 'w' || ch == 'W')
  {
    sendCommand("change_controller", "BalQP");
  }
  else if(ch == 'e' || ch == 'E')
  {
    sendCommand("change_controller", "BalMPC");
  }
  else if(ch == 'r' || ch == 'R')
  {
    sendCommand("change_controller", "BalMPCWB");
  }
  else if(ch == 'a' || ch == 'A')
  {
    sendCommand("move_body", "", 0);
  }
  else if(ch == 's' || ch == 'S')
  {
    sendCommand("move_body", "", 1);
  }
  else if(ch == 'd' || ch == 'D')
  {
    sendCommand("move_body", "", 2);
  }
  else if(ch == 'f' || ch == 'F')
  {
    sendCommand("move_body", "", 3);
  }
  else if(ch == 'g' || ch == 'G')
  {
    sendCommand("move_body", "", 4);
  }
  else if(ch == 'h' || ch == 'H')
  {
    sendCommand("move_body", "", 5);
  }
  else if(ch == 'j' || ch == 'J')
  {
    sendCommand("move_body", "", 6);
  }
  else if(ch == 'k' || ch == 'K')
  {
    sendCommand("move_body", "", 7);
  }
  else if(ch == 'l' || ch == 'L')
  {
    sendCommand("move_body", "", 8);
  }
  else if(ch == 'z' || ch == 'Z')
  {
    sendCommand("Order", "", 0);
  }
  else if(ch == 'x' || ch == 'X')
  {
    sendCommand("Order", "", 1);
  }
  else if(ch == 'c' || ch == 'C')
  {
    sendCommand("Order", "", 2);
  }
  else if(ch == 'v' || ch == 'V')
  {
    sendCommand("Order", "", 3);
  }
}

bool LeggedRobotTeleopKeyboard::sendCommand(std::string mainCommand, std::string subCommand, long int paramInt64, double paramFloat64)
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

void LeggedRobotTeleopKeyboard::restoreTerminalSettings(void)
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void LeggedRobotTeleopKeyboard::disableWaitingForEnter(void)
{
  struct termios newt;
  tcgetattr(0, &oldt_);             /* Save terminal settings */
  newt = oldt_;                     /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO); /* Change settings */
  tcsetattr(0, TCSANOW, &newt);     /* Apply settings */
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "legged_robot_teleop_keyboard");
  LeggedRobotTeleopKeyboard leggedRobotTeleopKeyboard;

  leggedRobotTeleopKeyboard.disableWaitingForEnter();
  ros::spinOnce();
  leggedRobotTeleopKeyboard.printText();
  char ch;
  while (ros::ok() && (ch = std::getchar()) != '0')
  {
    ros::spinOnce();
    leggedRobotTeleopKeyboard.printText();
    ros::spinOnce();
    leggedRobotTeleopKeyboard.setGoal(ch);
  }

  printf("terminated \n");
  leggedRobotTeleopKeyboard.restoreTerminalSettings();

  return 0;
}
