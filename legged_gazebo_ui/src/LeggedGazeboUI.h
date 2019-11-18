
#pragma once

#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <legged_controllers/UIState.h>

namespace gazebo
{
  class GAZEBO_VISIBLE LeggedGazeboUI : public GUIPlugin
  {
    Q_OBJECT

  public:
    LeggedGazeboUI();
    virtual ~LeggedGazeboUI();

    static LeggedGazeboUI* _instancePtr;
    bool IsActivated()  { return _uiActivated; }
    void OnUIState(const legged_controllers::UIStateConstPtr &_msg);
    bool SendCommand(std::string mainCommand, std::string subCommand = std::string(), long int paramInt64 = 0, double paramFloat64 = 0.0);

    void CreateLayout();

    signals: void SetController0(QString _string);
    signals: void SetController1(QString _string);
    signals: void SetController2(QString _string);
    signals: void SetController3(QString _string);

    protected slots: void OnButtonChgCtrl0();
    protected slots: void OnButtonChgCtrl1();
    protected slots: void OnButtonChgCtrl2();
    protected slots: void OnButtonChgCtrl3();

    protected slots: void OnButtonBody0();
    protected slots: void OnButtonBody1();
    protected slots: void OnButtonBody2();
    protected slots: void OnButtonBody3();
    protected slots: void OnButtonBody4();
    protected slots: void OnButtonBody5();
    protected slots: void OnButtonBody6();
    protected slots: void OnButtonBody7();
    protected slots: void OnButtonBody8();

    protected slots: void OnButtonOrder0();
    protected slots: void OnButtonOrder1();
    protected slots: void OnButtonOrder2();
    protected slots: void OnButtonOrder3();
  protected:
    bool _uiActivated;
    ros::NodeHandle _nodeHandle;
    ros::Subscriber _uiStateSub;
    ros::ServiceClient _uiCommandSrv;
  };
}
