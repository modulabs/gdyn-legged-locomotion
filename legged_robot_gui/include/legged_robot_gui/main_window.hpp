/*
  Author: Modulabs
  File Name: main_window.hpp
*/
 
#pragma once

#include <QMainWindow>

#include "ui_main_window.h"
#include "legged_robot_gui/qnode.hpp"


namespace legged_robot_gui
{
class MainWindow : public QMainWindow 
{
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent=0);
	~MainWindow();

public Q_SLOTS:
  void on_pushButton_00_clicked(void);
  void on_pushButton_01_clicked(void);
  void on_pushButton_02_clicked(void);
  void on_pushButton_03_clicked(void);
  void on_pushButton_04_clicked(void);
  void on_pushButton_05_clicked(void);
  void on_pushButton_06_clicked(void);
  void on_pushButton_07_clicked(void);
  void on_pushButton_08_clicked(void);
  void on_pushButton_09_clicked(void);
  void on_pushButton_10_clicked(void);
  void on_pushButton_11_clicked(void);
  void on_pushButton_12_clicked(void);
  void on_pushButton_13_clicked(void);
  void on_pushButton_14_clicked(void);
  void on_pushButton_15_clicked(void);
  void on_pushButton_16_clicked(void);
  void on_pushButton_17_clicked(void);
  void on_pushButton_18_clicked(void);
  void on_pushButton_19_clicked(void);
  void updateLoggingView();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};
}
