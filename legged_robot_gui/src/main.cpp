/*
  Author: Modulabs
  File Name: main.cpp
*/

#include <QtGui>
#include <QApplication>

#include "legged_robot_gui/main_window.hpp"


int main(int argc, char **argv) 
{
  QApplication app(argc, argv);
  legged_robot_gui::MainWindow w(argc,argv);
  w.show();
  app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
  int result = app.exec();

	return result;
}
