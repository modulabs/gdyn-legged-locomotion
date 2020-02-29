/*
  Author: Modulabs
  File Name: qnode.cpp
*/

#include <sstream>
#include <string>

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>

#include "legged_robot_gui/qnode.hpp"


namespace legged_robot_gui 
{
QNode::QNode(int argc, char** argv)
: init_argc(argc),
  init_argv(argv)
{}

QNode::~QNode() 
{
  if (ros::isStarted()) 
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
	wait();
}

bool QNode::init() 
{
	ros::init(init_argc,init_argv, "legged_robot_gui");
	if (!ros::master::check()) 
  {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	// Add your ros communications here.
  _uiCommandSrv = n.serviceClient<legged_robot_msgs::UICommand>("/hyq/main_controller/ui_command");

	start();
	return true;
}

void QNode::run()
{
	ros::Rate loop_rate(10);
	int count = 0;
	while (ros::ok())
  {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown();
}

bool QNode::sendCommand(std::string mainCommand, std::string subCommand, long int paramInt64, double paramFloat64)
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

  log(Info, std::string("Failed to send a command"));  
  return false;
}

void QNode::log(const LogLevel &level, const std::string &msg)
{
	logging_model.insertRows(logging_model.rowCount(), 1);
	std::stringstream logging_model_msg;
	switch (level)
  {
		case (Debug):
    {
			ROS_DEBUG_STREAM(msg);
			logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
			break;
		}
		case (Info):
    {
			ROS_INFO_STREAM(msg);
			logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << msg;
			break;
		}
		case (Warn):
    {
			ROS_WARN_STREAM(msg);
			logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
			break;
		}
		case (Error):
    {
      ROS_ERROR_STREAM(msg);
      logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
      break;
		}
		case (Fatal):
    {
			ROS_FATAL_STREAM(msg);
			logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
			break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}
}
