/*
  Author: Modulabs
  File Name: qnode.hpp
*/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>

#include "legged_robot_msgs/UICommand.h"


namespace legged_robot_gui 
{
class QNode : public QThread 
{
Q_OBJECT

public:
	QNode(int argc, char** argv);
	virtual ~QNode();
	bool init();
	void run();
  bool sendCommand(std::string mainCommand, std::string subCommand=std::string(), long int paramInt64=0, double paramFloat64=0.0);

  // Logging
  enum LogLevel 
  {
    Debug,
    Info,
    Warn,
    Error,
    Fatal
  };
	QStringListModel* loggingModel() {return &logging_model;}
	void log(const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();

private:
	int init_argc;
	char** init_argv;
  ros::ServiceClient _uiCommandSrv;
  QStringListModel logging_model;
};
}
