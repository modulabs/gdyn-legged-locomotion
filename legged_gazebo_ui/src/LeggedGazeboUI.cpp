#include <gazebo/msgs/msgs.hh>
#include <legged_controllers/UIState.h>
#include <legged_controllers/UICommand.h>
#include "LeggedGazeboUI.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(LeggedGazeboUI)

/////////////////////////////////////////////////
LeggedGazeboUI* LeggedGazeboUI::_instancePtr = nullptr;

static void UIStateCallback(const legged_controllers::UIStateConstPtr &_msg)
{
  if (LeggedGazeboUI::_instancePtr)
    LeggedGazeboUI::_instancePtr->OnUIState(_msg);
}

/////////////////////////////////////////////////
LeggedGazeboUI::LeggedGazeboUI()
  : GUIPlugin()
{
  _instancePtr = this;
  _uiActivated = false;

  CreateLayout();

  // msg : controller => ui
  _uiStateSub = _nodeHandle.subscribe("/hyq/main_controller/ui_state", 100, UIStateCallback);
  // srv : ui => controller
  _uiCommandSrv = _nodeHandle.serviceClient<legged_controllers::UICommand>("/hyq/main_controller/ui_command");
}

LeggedGazeboUI::~LeggedGazeboUI()
{
  _instancePtr = nullptr;
}

void LeggedGazeboUI::OnUIState(const legged_controllers::UIStateConstPtr &_msg)
{
  _uiActivated = true;
  this->SetController0(QString::fromStdString(std::string("LF-") + _msg->controller_name[0]));
  this->SetController1(QString::fromStdString(std::string("RF-") + _msg->controller_name[1]));
  this->SetController2(QString::fromStdString(std::string("LH-") + _msg->controller_name[2]));
  this->SetController3(QString::fromStdString(std::string("RH-") + _msg->controller_name[3]));
}

bool LeggedGazeboUI::SendCommand(std::string mainCommand, std::string subCommand, long int paramInt64, double paramFloat64)
{
  if (!IsActivated()) return false;

  legged_controllers::UICommand srv;
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

void LeggedGazeboUI::CreateLayout()
{
  this->setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Initialize
  QHBoxLayout *layoutMain = new QHBoxLayout;
  QFrame *frameMain = new QFrame();
  QHBoxLayout *layoutFrame = new QHBoxLayout();

  // Body
  QGroupBox* boxBody = new QGroupBox(tr("Body"));
  QGridLayout *layoutBody = new QGridLayout(boxBody);
  QPushButton *buttonBody0 = new QPushButton(tr("tl"));
  QPushButton *buttonBody1 = new QPushButton(tr("x+"));
  QPushButton *buttonBody2 = new QPushButton(tr("tr"));
  QPushButton *buttonBody3 = new QPushButton(tr("y+"));
  QPushButton *buttonBody4 = new QPushButton(tr("0"));
  QPushButton *buttonBody5 = new QPushButton(tr("y-"));
  QPushButton *buttonBody6 = new QPushButton(tr("z+"));
  QPushButton *buttonBody7 = new QPushButton(tr("x-"));
  QPushButton *buttonBody8 = new QPushButton(tr("z-"));
  layoutBody->addWidget(buttonBody0, 0, 0);
  layoutBody->addWidget(buttonBody1, 0, 1);
  layoutBody->addWidget(buttonBody2, 0, 2);
  layoutBody->addWidget(buttonBody3, 1, 0);
  layoutBody->addWidget(buttonBody4, 1, 1);
  layoutBody->addWidget(buttonBody5, 1, 2);
  layoutBody->addWidget(buttonBody6, 2, 0);
  layoutBody->addWidget(buttonBody7, 2, 1);
  layoutBody->addWidget(buttonBody8, 2, 2);
  connect(buttonBody0, SIGNAL(clicked()), this, SLOT(OnButtonBody0()));
  connect(buttonBody1, SIGNAL(clicked()), this, SLOT(OnButtonBody1()));
  connect(buttonBody2, SIGNAL(clicked()), this, SLOT(OnButtonBody2()));
  connect(buttonBody3, SIGNAL(clicked()), this, SLOT(OnButtonBody3()));
  connect(buttonBody4, SIGNAL(clicked()), this, SLOT(OnButtonBody4()));
  connect(buttonBody5, SIGNAL(clicked()), this, SLOT(OnButtonBody5()));
  connect(buttonBody6, SIGNAL(clicked()), this, SLOT(OnButtonBody6()));
  connect(buttonBody7, SIGNAL(clicked()), this, SLOT(OnButtonBody7()));
  connect(buttonBody8, SIGNAL(clicked()), this, SLOT(OnButtonBody8()));
  layoutFrame->addWidget(boxBody);

  // Order
  QGroupBox* boxOrder = new QGroupBox(tr("Order"));
  QVBoxLayout *layoutOrder = new QVBoxLayout(boxOrder);
  QPushButton *buttonOrder0 = new QPushButton(tr("Leg Up"));
  QPushButton *buttonOrder1 = new QPushButton(tr("Leg Dn"));
  QPushButton *buttonOrder2 = new QPushButton(tr("-"));
  QPushButton *buttonOrder3 = new QPushButton(tr("Reset"));
  layoutOrder->addWidget(buttonOrder0);
  layoutOrder->addWidget(buttonOrder1);
  layoutOrder->addWidget(buttonOrder2);
  layoutOrder->addWidget(buttonOrder3);
  connect(buttonOrder0, SIGNAL(clicked()), this, SLOT(OnButtonOrder0()));
  connect(buttonOrder1, SIGNAL(clicked()), this, SLOT(OnButtonOrder1()));
  connect(buttonOrder2, SIGNAL(clicked()), this, SLOT(OnButtonOrder2()));
  connect(buttonOrder3, SIGNAL(clicked()), this, SLOT(OnButtonOrder3()));
  layoutFrame->addWidget(boxOrder);

  // ChgCtrl
  QGroupBox* boxChgCtrl = new QGroupBox(tr("ChgCtrl"));
  QVBoxLayout *layoutChgCtrl = new QVBoxLayout(boxChgCtrl);
  QPushButton *buttonChgCtrl0 = new QPushButton(tr("VSD"));
  QPushButton *buttonChgCtrl1 = new QPushButton(tr("BalQP"));
  QPushButton *buttonChgCtrl2 = new QPushButton(tr("BalMPC"));
  QPushButton *buttonChgCtrl3 = new QPushButton(tr("Swing"));
  layoutChgCtrl->addWidget(buttonChgCtrl0);
  layoutChgCtrl->addWidget(buttonChgCtrl1);
  layoutChgCtrl->addWidget(buttonChgCtrl2);
  layoutChgCtrl->addWidget(buttonChgCtrl3);
  connect(buttonChgCtrl0, SIGNAL(clicked()), this, SLOT(OnButtonChgCtrl0()));
  connect(buttonChgCtrl1, SIGNAL(clicked()), this, SLOT(OnButtonChgCtrl1()));
  connect(buttonChgCtrl2, SIGNAL(clicked()), this, SLOT(OnButtonChgCtrl2()));
  connect(buttonChgCtrl3, SIGNAL(clicked()), this, SLOT(OnButtonChgCtrl3()));
  layoutFrame->addWidget(boxChgCtrl);

  // Current Controller
  QGroupBox* boxController = new QGroupBox(tr("Ctrl"));
  QVBoxLayout *layoutController = new QVBoxLayout(boxController);
  QLabel *controller0 = new QLabel(tr("LF-*"));
  QLabel *controller1 = new QLabel(tr("RF-*"));
  QLabel *controller2 = new QLabel(tr("LH-*"));
  QLabel *controller3 = new QLabel(tr("RH-*"));
  layoutController->addWidget(controller0);
  layoutController->addWidget(controller1);
  layoutController->addWidget(controller2);
  layoutController->addWidget(controller3);
  connect(this, SIGNAL(SetController0(QString)), controller0, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetController1(QString)), controller1, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetController2(QString)), controller2, SLOT(setText(QString)), Qt::QueuedConnection);
  connect(this, SIGNAL(SetController3(QString)), controller3, SLOT(setText(QString)), Qt::QueuedConnection);
  layoutFrame->addWidget(boxController);

  // Finalize
  frameMain->setLayout(layoutFrame);
  layoutMain->addWidget(frameMain);
  layoutFrame->setContentsMargins(1, 1, 1, 1);
  layoutMain->setContentsMargins(0, 0, 0, 0);
  this->setLayout(layoutMain);
  this->move(10, 10);
  this->resize(600, 250);
}

void LeggedGazeboUI::OnButtonChgCtrl0()
{
  SendCommand("ChgCtrl", "VSD");
}

void LeggedGazeboUI::OnButtonChgCtrl1()
{
  SendCommand("ChgCtrl", "BalQP");
}

void LeggedGazeboUI::OnButtonChgCtrl2()
{
  SendCommand("ChgCtrl", "BalMPC");
}

void LeggedGazeboUI::OnButtonChgCtrl3()
{
  SendCommand("ChgCtrl", "BalMPCWB");
}

void LeggedGazeboUI::OnButtonBody0()
{
  SendCommand("Body", "", 0);
}

void LeggedGazeboUI::OnButtonBody1()
{
  SendCommand("Body", "", 1);
}

void LeggedGazeboUI::OnButtonBody2()
{
  SendCommand("Body", "", 2);
}

void LeggedGazeboUI::OnButtonBody3()
{
  SendCommand("Body", "", 3);
}

void LeggedGazeboUI::OnButtonBody4()
{
  SendCommand("Body", "", 4);
}

void LeggedGazeboUI::OnButtonBody5()
{
  SendCommand("Body", "", 5);
}

void LeggedGazeboUI::OnButtonBody6()
{
  SendCommand("Body", "", 6);
}

void LeggedGazeboUI::OnButtonBody7()
{
  SendCommand("Body", "", 7);
}

void LeggedGazeboUI::OnButtonBody8()
{
  SendCommand("Body", "", 8);
}

void LeggedGazeboUI::OnButtonOrder0()
{
  SendCommand("Order", "", 0);
}

void LeggedGazeboUI::OnButtonOrder1()
{
  SendCommand("Order", "", 1);
}

void LeggedGazeboUI::OnButtonOrder2()
{
  SendCommand("Order", "", 2);
}

void LeggedGazeboUI::OnButtonOrder3()
{
  SendCommand("Order", "", 3);
}
