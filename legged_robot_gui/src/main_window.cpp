/*
  Author: Modulabs
  File Name: main_window.cpp
*/

#include <QtGui>
#include <QMessageBox>
#include <iostream>

#include "legged_robot_gui/main_window.hpp"


using namespace Qt;

namespace legged_robot_gui
{
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
: QMainWindow(parent),
  qnode(argc,argv)
{
  qnode.init();
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  // Logging
	ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
}

MainWindow::~MainWindow() {}

void MainWindow::on_pushButton_00_clicked(void)
{
  qnode.sendCommand("ChgCtrl", "VSD");
}

void MainWindow::on_pushButton_01_clicked(void)
{
  qnode.sendCommand("ChgCtrl", "BalQP");
}

void MainWindow::on_pushButton_02_clicked(void)
{
  qnode.sendCommand("ChgCtrl", "BalMPC");
}

void MainWindow::on_pushButton_03_clicked(void)
{
  qnode.sendCommand("ChgCtrl", "BalMPCWB");
}

void MainWindow::on_pushButton_04_clicked(void)
{
  qnode.sendCommand("Body", "", 0);
}

void MainWindow::on_pushButton_05_clicked(void)
{
  qnode.sendCommand("Body", "", 1);
}

void MainWindow::on_pushButton_06_clicked(void)
{
  qnode.sendCommand("Body", "", 2);
}

void MainWindow::on_pushButton_07_clicked(void)
{
  qnode.sendCommand("Body", "", 3);
}

void MainWindow::on_pushButton_08_clicked(void)
{
  qnode.sendCommand("Body", "", 4);
}

void MainWindow::on_pushButton_09_clicked(void)
{
  qnode.sendCommand("Body", "", 5);
}

void MainWindow::on_pushButton_10_clicked(void)
{
  qnode.sendCommand("Body", "", 6);
}

void MainWindow::on_pushButton_11_clicked(void)
{
  qnode.sendCommand("Body", "", 7);
}

void MainWindow::on_pushButton_12_clicked(void)
{
  qnode.sendCommand("Body", "", 8);
}

void MainWindow::on_pushButton_13_clicked(void)
{
  qnode.sendCommand("Order", "", 0);
}

void MainWindow::on_pushButton_14_clicked(void)
{
  qnode.sendCommand("Order", "", 1);
}

void MainWindow::on_pushButton_15_clicked(void)
{
  qnode.sendCommand("Order", "", 2);
}

void MainWindow::on_pushButton_16_clicked(void)
{
  qnode.sendCommand("Order", "", 3);
}

void MainWindow::on_pushButton_17_clicked(void)
{
  qnode.log(qnode.Info, std::string("Hahaha"));  
  double value = ui.doubleSpinBox->value();
  ui.lineEdit->setText(QString::number(value ,'f', 3));
}

void MainWindow::on_pushButton_18_clicked(void)
{
  qnode.log(qnode.Info, std::string("Hahaha"));  
  double value = ui.doubleSpinBox->value();
  ui.lineEdit->setText(QString::number(value ,'f', 3));
}

void MainWindow::on_pushButton_19_clicked(void)
{
  qnode.log(qnode.Info, std::string("Hahaha"));  
  double value = ui.doubleSpinBox->value();
  ui.lineEdit->setText(QString::number(value ,'f', 3));
}

void MainWindow::updateLoggingView()
{
  ui.view_logging->scrollToBottom();
}
}
