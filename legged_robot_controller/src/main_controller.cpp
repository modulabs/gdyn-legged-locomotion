/*
  Author: Modulabs
  File Name: main_controller.cpp
*/

#include "legged_robot_controller/main_controller.h"


using namespace legged_robot_controller;

namespace legged_robot_controller
{
bool MainController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
{
  _loop_count = 0;
  _node_ptr = &n;

  // List of controlled joints
  if (!n.getParam("joints", _joint_names))
  {
    ROS_ERROR("Could not find joint name");
    return false;
  }
  _n_joints = _joint_names.size();

  if (_n_joints == 0)
  {
    ROS_ERROR("List of joint names is empty.");
    return false;
  }
  else
  {
    ROS_INFO("Find %d joints", _n_joints);
  }

  // urdf
  urdf::Model urdf;
  if (!urdf.initParam("robot_description"))
  {
    ROS_ERROR("Failed to parse urdf file");
    return false;
  }

  // joint handle
  for (int i = 0; i < _n_joints; i++)
  {
    try
    {
      _joints.push_back(hw->getHandle(_joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException &e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }

    urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(_joint_names[i]);
    if (!joint_urdf)
    {
      ROS_ERROR("Could not find joint '%s' in urdf", _joint_names[i].c_str());
      return false;
    }
    _joint_urdfs.push_back(joint_urdf);
  }

  // kdl parser
  if (!kdl_parser::treeFromUrdfModel(urdf, _robot._kdl_tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  _robot.init();

  // command and state (12x1)
  _tau_d.data = Eigen::VectorXd::Zero(_n_joints);
  _tau_fric.data = Eigen::VectorXd::Zero(_n_joints);
  _qdot_d.data = Eigen::VectorXd::Zero(_n_joints);
  _qddot_d.data = Eigen::VectorXd::Zero(_n_joints);
  _q_d_old.data = Eigen::VectorXd::Zero(_n_joints);
  _qdot_d_old.data = Eigen::VectorXd::Zero(_n_joints);

  _q_error.data = Eigen::VectorXd::Zero(_n_joints);
  _qdot_error.data = Eigen::VectorXd::Zero(_n_joints);

  // gains
  _kp.resize(_n_joints);
  _kd.resize(_n_joints);

  // service ui command
  _ui_command_srv = n.advertiseService("ui_command", &MainController::srvUICommand, this);

  // service updating gain
  _gains_kp_buffer.writeFromNonRT(std::vector<double>(_n_joints, 0.0));
  _gains_kd_buffer.writeFromNonRT(std::vector<double>(_n_joints, 0.0));
  _update_gain_srv = n.advertiseService("update_gain", &MainController::updateGain, this);

  updateGain();

  // command subscriber
  _commands_buffer.writeFromNonRT(std::vector<double>(_n_joints, 0.0));
  _commands_sub = n.subscribe<std_msgs::Float64MultiArray>("command", 1, &MainController::subscribeCommand, this);

  // state subscriber
  _trunk_state_buffer.writeFromNonRT(Trunk());
  _link_states_sub = n.subscribe("/gazebo/link_states", 1, &MainController::subscribeTrunkState, this);
  for (int i = 0; i < 4; i++)
    _contact_states_buffer[i].writeFromNonRT(0);

  _contact_states_sub[0] = n.subscribe("/hyq/lf_foot_bumper", 1, &MainController::subscribeLFContactState, this);
  _contact_states_sub[1] = n.subscribe("/hyq/rf_foot_bumper", 1, &MainController::subscribeRFContactState, this);
  _contact_states_sub[2] = n.subscribe("/hyq/lh_foot_bumper", 1, &MainController::subscribeLHContactState, this);
  _contact_states_sub[3] = n.subscribe("/hyq/rh_foot_bumper", 1, &MainController::subscribeRHContactState, this);

  // start realtime state publisher
  _controller_state_pub.reset(
    new realtime_tools::RealtimePublisher<legged_robot_msgs::ControllerJointState>(n, "state", 1));

  _controller_state_pub->msg_.header.stamp = ros::Time::now();
  for (size_t i = 0; i < _n_joints; i++)
  {
    _controller_state_pub->msg_.name.push_back(_joint_names[i]);
    _controller_state_pub->msg_.command.push_back(0.0);
    _controller_state_pub->msg_.command_dot.push_back(0.0);
    _controller_state_pub->msg_.state.push_back(0.0);
    _controller_state_pub->msg_.state_dot.push_back(0.0);
    _controller_state_pub->msg_.q_error.push_back(0.0);
    _controller_state_pub->msg_.qdot_error.push_back(0.0);
    _controller_state_pub->msg_.effort_command.push_back(0.0);
    _controller_state_pub->msg_.effort_feedforward.push_back(0.0);
    _controller_state_pub->msg_.effort_feedback.push_back(0.0);
  }

  // start realtime ui state publisher
  _ui_state_pub.reset(
    new realtime_tools::RealtimePublisher<legged_robot_msgs::UIState>(n, "ui_state", 1));

  _ui_state_pub->msg_.header.stamp = ros::Time::now();
  for (size_t i = 0; i < _n_joints; i++)
  {
    _ui_state_pub->msg_.controller_name.push_back("");
  }

  // For balance controller and mpc controller
  _robot._m_body = 83.282; //60.96, 71.72,
  _robot._mu_foot = 0.6;   // TO DO: get this value from robot model
  _robot._I_com_body = Eigen::Matrix3d::Zero();
  _robot._I_com_body.diagonal() << 1.5725937, 8.5015928, 9.1954911;
  _robot._p_body2com = Eigen::Vector3d(0.056, 0.0215, 0.00358);

  // Controllers
  _virtual_spring_damper_controller.init();
  _balance_controller.init();
  _mpc_controller.init();
  _mpc_controller._step = 0;

  // first controller
  _robot.setController(4, quadruped_robot::controllers::VirtualSpringDamper);
  for (size_t i = 0; i < 4; i++)
    _robot._p_body2leg_d[i] = Vector3d(0, 0, -0.4);

  // trajectory control points
  std::vector<Vector2d> pnts(4);
  pnts[0](0) = -0.3; pnts[0](1) = -0.5;
  pnts[1](0) = -0.3; pnts[1](1) = -0.3;
  pnts[2](0) = 0.3; pnts[2](1) = -0.3;
  pnts[3](0) = 0.3; pnts[3](1) = -0.5;
  _swing_traj.setPoints(pnts);

  pnts.resize(2);
  pnts[0](0) = 0.3; pnts[0](1) = -0.5;
  pnts[1](0) = -0.3; pnts[1](1) = -0.5;
  _stance_traj.setPoints(pnts);
  
  return true;
}

void MainController::starting(const ros::Time &time)
{
  _t = 0;

  ROS_INFO("Starting Leg Controller");
}

void MainController::subscribeCommand(const std_msgs::Float64MultiArrayConstPtr &msg)
{
  if (msg->data.size() != _n_joints)
  {
    ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << _n_joints << ")! Not executing!");
    return;
  }
  _commands_buffer.writeFromNonRT(msg->data);
}

void MainController::subscribeTrunkState(const gazebo_msgs::LinkStatesConstPtr &msg)
{
  Trunk trunk;

  trunk._p = Eigen::Vector3d(msg->pose[1].position.x, msg->pose[1].position.y, msg->pose[1].position.z);
  trunk._v = Eigen::Vector3d(msg->twist[1].linear.x, msg->twist[1].linear.y, msg->twist[1].linear.z);
  trunk._o = Eigen::Quaterniond(Eigen::Quaterniond(msg->pose[1].orientation.w,
                           msg->pose[1].orientation.x,
                           msg->pose[1].orientation.y,
                           msg->pose[1].orientation.z));
  trunk._w = Eigen::Vector3d(msg->twist[1].angular.x,
                 msg->twist[1].angular.y, msg->twist[1].angular.z);

  _trunk_state_buffer.writeFromNonRT(trunk);
}

void MainController::subscribeLFContactState(const gazebo_msgs::ContactsStateConstPtr &msg)
{
  Eigen::Vector3d contact_force;

  if (msg->states.size() > 0)
    contact_force << msg->states[0].total_wrench.force.x, msg->states[0].total_wrench.force.y, msg->states[0].total_wrench.force.z;

  if (contact_force.norm() > 10)
    _contact_states_buffer[0].writeFromNonRT(1);
  else
    _contact_states_buffer[0].writeFromNonRT(0);
}

void MainController::subscribeRFContactState(const gazebo_msgs::ContactsStateConstPtr &msg)
{
  Eigen::Vector3d contact_force;

  if (msg->states.size() > 0)
    contact_force << msg->states[0].total_wrench.force.x, msg->states[0].total_wrench.force.y, msg->states[0].total_wrench.force.z;

  if (contact_force.norm() > 10)
    _contact_states_buffer[1].writeFromNonRT(1);
  else
    _contact_states_buffer[1].writeFromNonRT(0);
}

void MainController::subscribeLHContactState(const gazebo_msgs::ContactsStateConstPtr &msg)
{
  Eigen::Vector3d contact_force;

  if (msg->states.size() > 0)
    contact_force << msg->states[0].total_wrench.force.x, msg->states[0].total_wrench.force.y, msg->states[0].total_wrench.force.z;

  if (contact_force.norm() > 10)
    _contact_states_buffer[2].writeFromNonRT(1);
  else
    _contact_states_buffer[2].writeFromNonRT(0);
}

void MainController::subscribeRHContactState(const gazebo_msgs::ContactsStateConstPtr &msg)
{
  Eigen::Vector3d contact_force;

  if (msg->states.size() > 0)
    contact_force << msg->states[0].total_wrench.force.x, msg->states[0].total_wrench.force.y, msg->states[0].total_wrench.force.z;

  if (contact_force.norm() > 10)
    _contact_states_buffer[3].writeFromNonRT(1);
  else
    _contact_states_buffer[3].writeFromNonRT(0);
}

bool MainController::updateGain(legged_robot_msgs::UpdateGain::Request &request, legged_robot_msgs::UpdateGain::Response &response)
{
  updateGain();
}

bool MainController::updateGain()
{
  std::vector<double> kp(_n_joints), kd(_n_joints);
  std::string gain_name;

  for (size_t i = 0; i < _n_joints; i++)
  {
    // FIXME. generalize name of robot
    gain_name = "/hyq/main_controller/gains/" + _joint_names[i] + "/p";
    if (_node_ptr->getParam(gain_name, kp[i]))
    {
      ROS_INFO("Update gain %s = %.2f", gain_name.c_str(), kp[i]);
    }
    else
    {
      ROS_ERROR("Cannot find %s gain", gain_name.c_str());
      return false;
    }

    gain_name = "/hyq/main_controller/gains/" + _joint_names[i] + "/d";
    if (_node_ptr->getParam(gain_name, kd[i]))
    {
      ROS_INFO("Update gain %s = %.2f", gain_name.c_str(), kd[i]);
    }
    else
    {
      ROS_ERROR("Cannot find %s gain", gain_name.c_str());
      return false;
    }
  }

  _gains_kp_buffer.writeFromNonRT(kp);
  _gains_kd_buffer.writeFromNonRT(kd);

  return true;
}

bool MainController::srvMoveBodyCB(legged_robot_msgs::MoveBody::Request &request, legged_robot_msgs::MoveBody::Response &response)
{
  // Eigen::Vector6d delta_pose = Map<Eigen::Vector6d>(request.delta_pose);

  // p_body_d = p_body_d + delta_pose.head(3) * MM2M;

  // AngleAxisd request.delta_pose[3] * D2R;

  // _minjerk_traj.setTrajInput(request.delta_pose, request.duration);
}

bool MainController::srvUICommand(legged_robot_msgs::UICommand::Request &request, legged_robot_msgs::UICommand::Response &response)
{
  std::string mainCommand = request.main_command;
  std::string subCommand = request.sub_command;
  long int intParam = request.param_int64;
  double floatParam = request.param_float64;
  if (mainCommand == "ChgCtrl")
  {
    _robot._pose_body_d._pos = _robot._pose_body._pos;
      _robot._pose_body_d._rot_quat.setIdentity();
       _robot._pose_body_d._pos(2) += 300 * MM2M;
    
    if (subCommand == "VSD")
      _robot.setController(4, quadruped_robot::controllers::VirtualSpringDamper);
    else if (subCommand == "BalQP")
      _robot.setController(4, quadruped_robot::controllers::BalancingQP);
    else if (subCommand == "BalMPC")
      _robot.setController(4, quadruped_robot::controllers::BalancingMPC);
    else if (subCommand == "BalMPCWB")
      _robot.setController(4, quadruped_robot::controllers::BalancingMPCWholeBody);

    response.result = true;
    return true;
  }
  else if (mainCommand == "Body")
  {
    // Turn Left/Right
    if (intParam == 0)
    {
      _robot._pose_body_d._rot_quat = AngleAxisd(15*D2R, Vector3d::UnitZ()) * _robot._pose_body_d._rot_quat;
    }
    else if (intParam == 2)
    {
      _robot._pose_body_d._rot_quat = AngleAxisd(-15*D2R, Vector3d::UnitZ()) * _robot._pose_body_d._rot_quat;
    }
    // X
    else if (intParam == 1)
    {
        _robot._pose_body_d._pos(0) += 50 * MM2M;
    }
    else if (intParam == 7)
    {
        _robot._pose_body_d._pos(0) -= 50 * MM2M;
    }
    // Y
    else if (intParam == 3)
    {
        _robot._pose_body_d._pos(1) += 50 * MM2M;
    }
    else if (intParam == 5)
    {
        _robot._pose_body_d._pos(1) -= 50 * MM2M;
    }
    // Z
    else if (intParam == 6)
    {
        _robot._pose_body_d._pos(2) += 50 * MM2M;
    }
    else if (intParam == 8)
    {
        _robot._pose_body_d._pos(2) -= 50 * MM2M;
    }
    // Zero
    else if (intParam == 4)
    {
      _robot._pose_body_d._pos(0) = 0.0;
      _robot._pose_body_d._pos(1) = 0.0;
      _robot._pose_body_d._pos(2) = 500.0 * MM2M;
      _robot._pose_body_d._rot_quat.setIdentity();
    }

    response.result = true;
    return true;
  }
  else if (mainCommand == "Order")
  {
    if (intParam == 0)
    {
      _robot._p_body2leg_d[0](2) = -0.2;
      _robot.setController(0, quadruped_robot::controllers::VirtualSpringDamper);
      _robot.setController(1, quadruped_robot::controllers::BalancingQP);
      _robot.setController(2, quadruped_robot::controllers::BalancingQP);
      _robot.setController(3, quadruped_robot::controllers::BalancingQP);
    }
    else if (intParam == 1)
    {
      _robot._p_body2leg_d[0](2) = -0.4;
      _robot.setController(4, quadruped_robot::controllers::BalancingQP);
    }
    else if (intParam == 2)
    {
      // Order2
    }
    else if (intParam == 3)
    {
      // Please add the proper codes that restart the simulator
    }
    response.result = true;
    return true;
  }

  response.result = false;
  return true;
}

void MainController::update(const ros::Time &time, const ros::Duration &period)
{
  // Update from real-time buffer
  std::vector<double> &commands = *_commands_buffer.readFromRT();
  std::vector<double> &kp = *_gains_kp_buffer.readFromRT();
  std::vector<double> &kd = *_gains_kd_buffer.readFromRT();
  Trunk &trunk_state = *_trunk_state_buffer.readFromRT();
  std::array<int, 4> contact_states;

  for (size_t i = 0; i < 4; i++)
  {
    contact_states[i] = *_contact_states_buffer[i].readFromRT();
  }

  double dt = period.toSec();
  _t += dt;

  // update state from tree (12x1) to each leg (4x3)
  std::array<Eigen::Vector3d, 4> q_leg, qdot_leg;
  for (size_t i = 0; i < _n_joints; i++)
  {
    if (i < 3)
    {
      q_leg[0](i) = _joints[i].getPosition();
      qdot_leg[0](i) = _joints[i].getVelocity();
    }
    else if (i < 6)
    {
      q_leg[1](i - 3) = _joints[i].getPosition();
      qdot_leg[1](i - 3) = _joints[i].getVelocity();
    }
    else if (i < 9)
    {
      q_leg[2](i - 6) = _joints[i].getPosition();
      qdot_leg[2](i - 6) = _joints[i].getVelocity();
    }
    else
    {
      q_leg[3](i - 9) = _joints[i].getPosition();
      qdot_leg[3](i - 9) = _joints[i].getVelocity();
    }
  }

  // Sensor Data - continuosly update, subscribe from gazebo for now, @TODO: get this as raw sensor data
  _robot.updateSensorData(q_leg, qdot_leg, Pose(trunk_state._p, trunk_state._o), PoseVel(trunk_state._v, trunk_state._w), contact_states);

  // @TODO: State Estimation, For now, use gazebo data
  //  _state_estimation.update(_robot);

  // @TODO: Trajectory Generation, update trajectory - get from this initial state(temporary)
  // _trajectory_generator.update(_robot);


#define SWING_CONTROL_TEST
#ifdef SWING_CONTROL_TEST
  static int td = 0;
  static double s = 0;
  double s_, s__;
  static double t_touchdown = 0;
  std::array<double, 4> gait_phase_lag;
  std::array<double, 4> trot_gait_phase_lag = {0, 0.5, 0.5, 0};
  std::array<double, 4> gallop_gait_phase_lag = {0, 0.2, 0.55, 0.75};
  double T_stance = 1;
  double T_swing = 0.25;
  double T_stride = T_stance + T_swing;

  if (td > 5000)
  {
    _robot._t_leg[0] = _t - t_touchdown;

    if (_robot._t_leg[0] > T_stride)
    {
      t_touchdown = _t;
      _robot._t_leg[0] = T_stride;
    }

    for (int i=0; i<4; i++)
    {
      _robot._t_leg[i] = _robot._t_leg[0] - trot_gait_phase_lag[i] * T_stride;

      if (_robot._t_leg[i] < -T_stride)
      {
        _robot._contact_state[i] = 0;
        _robot._S_swing[i] = 1;
      }
      else if (_robot._t_leg[i] < -T_swing)
      {
        _robot._contact_state[i] = 1;
        _robot._S_stance[i] = (_robot._t_leg[i] + T_stride) / T_stance;
      }
      else if (_robot._t_leg[i] < 0)
      {
        _robot._contact_state[i] = 0;
        _robot._S_swing[i] = (_robot._t_leg[i] + T_swing) / T_swing;
      }
      else if (_robot._t_leg[i] < T_stance)
      {
        _robot._contact_state[i] = 1;
        _robot._S_stance[i] = _robot._t_leg[i]/T_stance;
      }
      else if (_robot._t_leg[i] < T_stride)
      {
        _robot._contact_state[i] = 0;
        _robot._S_swing[i] = (_robot._t_leg[i] - T_stance) / T_swing;
      }
      else
      {
        _robot._contact_state[i] = 0;
        _robot._S_swing[i] = 1;
      }

      if (_robot._contact_state[i] == 0)
      {
        _robot._p_body2leg_d[i](0) = _swing_traj.getPoint(_robot._S_swing[i])(0);
        _robot._p_body2leg_d[i](2) = _swing_traj.getPoint(_robot._S_swing[i])(1);
      }
      else
      {
        _robot._p_body2leg_d[i](0) = _stance_traj.getPoint(_robot._S_stance[i])(0);
        _robot._p_body2leg_d[i](2) = _stance_traj.getPoint(_robot._S_stance[i])(1);
      }
    }

    _robot.setController(4, quadruped_robot::controllers::VirtualSpringDamper);
  }
  td++;  
#endif
// comment out to test the gui plugin
  /*
#ifdef MPC_Debugging
  static int td = 0;
  if (td++ == 3000)
  {
    _robot._pose_com_d._pos = _robot._pose_com._pos;
    _robot._pose_body_d._pos = _robot._pose_body._pos;
    _robot._pose_body_d._rot_quat = _robot._pose_body._rot_quat;
    _robot._p_world2leg_d = _robot._p_world2leg;
    _robot._pose_vel_com_d._linear.setZero();
    _robot._pose_vel_body_d._angular.setZero();

    _robot.setController(4, quadruped_robot::controllers::BalancingMPC);
    ROS_INFO("Change Controller from virtual spring damper to MPC Controller");
  }
  if (td == 4000)
  {
    _robot._pose_body_d._pos(2) += 200 * MM2M;
  }

  if (td == 6000)
  {
    _robot._p_body2leg_d[0](2) = -0.2;

    ROS_INFO("Change Controller to 3 leg balancing mode.");
    _robot.setController(0, quadruped_robot::controllers::VirtualSpringDamper);
    _robot.setController(1, quadruped_robot::controllers::BalancingMPC);
    _robot.setController(2, quadruped_robot::controllers::BalancingMPC);
    _robot.setController(3, quadruped_robot::controllers::BalancingMPC);
  }

  if(td >= 6000)
  {
    _robot._contact_states[1] = true;
    _robot._contact_states[2] = true;
    _robot._contact_states[3] = true;
  }  
#else
  static int td = 0;
  if (td++ == 5000)
  {
    _robot._pose_body_d._pos = _robot._pose_body._pos;
    _robot._pose_body_d._rot_quat.setIdentity();
    _robot._pose_body_d._pos(2) += 300 * MM2M;
    _robot.setController(4, quadruped_robot::controllers::BalancingQP);

    ROS_INFO("Change Controller from virtual spring damper to qp balance");
  }

#define TUNING_BALANCE_QP
#ifdef TUNING_BALANCE_QP
  if (td == 6000)
  {
    _robot._pose_body_d._pos(2) -= 200 * MM2M;
  }

  if (td == 7000)
  {
    _robot._pose_body_d._pos(0) += 200 * MM2M;
  }

  if (td == 8000)
  {
    _robot._pose_body_d._pos(0) -= 200 * MM2M;
  }

//  if (td == 9000)
//  {
//    _robot._pose_body_d._pos(1) += 50 * MM2M;
//  }

//  if (td == 10000)
//  {
//    _robot._pose_body_d._pos(1) -= 50 * MM2M;
//  }

  if (td == 9000)
  {
    _robot._pose_body_d._rot_quat = AngleAxisd(25*D2R, Vector3d::UnitZ()) * _robot._pose_body_d._rot_quat;
  }

  if (td == 10000)
  {
    _robot._pose_body_d._rot_quat = AngleAxisd(-25*D2R, Vector3d::UnitZ()) * _robot._pose_body_d._rot_quat;
  }

  if (td == 11000)
  {
    _robot._pose_body_d._rot_quat = AngleAxisd(20*D2R, Vector3d::UnitX()) * _robot._pose_body_d._rot_quat;
  }

  if (td == 12000)
  {
    _robot._pose_body_d._rot_quat = AngleAxisd(-20*D2R, Vector3d::UnitX()) * _robot._pose_body_d._rot_quat;
  }

  if (td == 13000)
  {
    _robot._pose_body_d._rot_quat = AngleAxisd(20*D2R, Vector3d::UnitY()) * _robot._pose_body_d._rot_quat;
  }

  if (td == 14000)
  {
    _robot._pose_body_d._rot_quat = AngleAxisd(-20*D2R, Vector3d::UnitY()) * _robot._pose_body_d._rot_quat;
  }

#endif

#ifdef TUNING_BALANCE_QP
  if (td == 15000)
#else
  if (td == 6000)
#endif
  {
    _robot._pose_body_d._pos(0) -= 100 * MM2M;
    _robot._pose_body_d._pos(1) -= 50 * MM2M;
    _robot.setController(4, quadruped_robot::controllers::BalancingQP);

  }

#ifdef TUNING_BALANCE_QP
  if (td == 15500)
#else
  if (td == 6500)
#endif
  {
    _robot._p_body2leg_d[0](2) = -0.2;

    ROS_INFO("Change Controller to 3 leg balancing mode.");
    _robot.setController(0, quadruped_robot::controllers::VirtualSpringDamper);
    _robot.setController(1, quadruped_robot::controllers::BalancingQP);
    _robot.setController(2, quadruped_robot::controllers::BalancingQP);
    _robot.setController(3, quadruped_robot::controllers::BalancingQP);
  }

//  _robot._pose_body_d._rot_quat.setIdentity();
  _robot._pose_vel_body_d._angular.setZero();
#endif
*/

  // Kinematics, Dynamics
  _robot.calKinematicsDynamics();
  _virtual_spring_damper_controller.update(_robot, _F_leg);

#ifdef MPC_Debugging

  if (_robot.getController(1) == quadruped_robot::controllers::BalancingMPC)
  {    
    if (_mpc_controller._step++ == 0)
    {
      _mpc_controller.setControlData(_robot);
      _mpc_controller.calControlInput();
      _mpc_controller.getControlInput(_robot, _F_leg);
    }    

    if (_mpc_controller._step == Control_Step)
    {
      _mpc_controller._step = 0;
    }
  }

#else
  _balance_controller.update(_robot, _F_leg);
#endif

  // Convert force to torque
  for (size_t i = 0; i < 4; i++)
  {
    _tau_leg[i] = _robot._Jv_leg[i].transpose() * _F_leg[i];
  }

  // torque command
  for (int i = 0; i < _n_joints; i++)
  {
    if (i < 3)
      _tau_d(i) = _tau_leg[0](i);
    else if (i < 6)
      _tau_d(i) = _tau_leg[1](i - 3);
    else if (i < 9)
      _tau_d(i) = _tau_leg[2](i - 6);
    else
      _tau_d(i) = _tau_leg[3](i - 9);
  }

  for (int i = 0; i < _n_joints; i++)
  {
    // effort saturation
    if (_tau_d(i) >= _joint_urdfs[i]->limits->effort)
    {
      //       ROS_INFO("effort saturation + %d", i);
      _tau_d(i) = _joint_urdfs[i]->limits->effort;
    }

    if (_tau_d(i) <= -_joint_urdfs[i]->limits->effort)
    {
      //      ROS_INFO("effort saturation - %d", i);
      _tau_d(i) = -_joint_urdfs[i]->limits->effort;
    }

    _joints[i].setCommand(_tau_d(i));
  }

  // publish
  if (_loop_count % 10 == 0)
  {
    if (_controller_state_pub->trylock())
    {
      _controller_state_pub->msg_.header.stamp = time;
      for (int i = 0; i < _n_joints; i++)
      {
        //        _controller_state_pub->msg_.command[i] = R2D*_q_d(i);
        //        _controller_state_pub->msg_.command_dot[i] = R2D*_qdot_d(i);
        //        _controller_state_pub->msg_.state[i] = R2D*_q(i);
        //        _controller_state_pub->msg_.state_dot[i] = R2D*_qdot(i);
        //        _controller_state_pub->msg_.q_error[i] = R2D*_q_error(i);
        //        _controller_state_pub->msg_.qdot_error[i] = R2D*_qdot_error(i);
        // FIXME. temporarily
        // _controller_state_pub->msg_.effort_command[i] = _tau_d(i);
        // _controller_state_pub->msg_.effort_feedback[i] = _tau_d(i) - _controller_state_pub->msg_.effort_feedforward[i];
      }

      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          _controller_state_pub->msg_.effort_command[i * 3 + j] = _F_leg[i](j);
          _controller_state_pub->msg_.effort_feedback[i * 3 + j] = _F_leg_balance[i](j);
        }
      }
      _controller_state_pub->unlockAndPublish();
    }
  }
  if (_loop_count % 20 == 0)
  {
    // realtime ui state publisher
    if (_ui_state_pub->trylock())
    {
      _ui_state_pub->msg_.header.stamp = time;
      for (int i = 0; i < 4; i++)
      {
        _ui_state_pub->msg_.controller_name[i] = _robot.getControllerName(i);
      }
      _ui_state_pub->unlockAndPublish();
    }
  }

  // ********* printf state *********
  //printState();
}

void MainController::enforceJointLimits(double &command, unsigned int index)
{
  // Check that this joint has applicable limits
  if (_joint_urdfs[index]->type == urdf::Joint::REVOLUTE || _joint_urdfs[index]->type == urdf::Joint::PRISMATIC)
  {
    if (command > _joint_urdfs[index]->limits->upper) // above upper limnit
    {
      command = _joint_urdfs[index]->limits->upper;
    }
    else if (command < _joint_urdfs[index]->limits->lower) // below lower limit
    {
      command = _joint_urdfs[index]->limits->lower;
    }
  }
}

void MainController::printState()
{
  static int count = 0;
  if (count > 30)
  {
    printf("*********************************************************\n\n");
    printf("*** Simulation Time (unit: sec)  ***\n");
    printf("t = %f\n", _t);
    printf("\n");

    printf("*********************** Left Front Leg **********************************\n\n");

    //printf("*** Actual Position in Joint Space (unit: deg) ***\n");
    //printf("q(1): %f, ", _q_leg[0](0) * R2D);
    //printf("q(2): %f, ", _q_leg[0](1) * R2D);
    //printf("q(3): %f, ", _q_leg[0](2) * R2D);
    //printf("\n");
    //printf("\n");

    //printf("*** Actual Position in Task Space (unit: m) ***\n");
    //printf("x(1): %f, ", _p_leg[0](0) * 1);
    //printf("x(2): %f, ", _p_leg[0](1) * 1);
    //printf("x(3): %f, ", _p_leg[0](2) * 1);
    //printf("\n");
    //printf("\n");

    //printf("*** Virtual Leg Forces (unit: N) ***\n");
    //printf("X Force Input: %f, ", _F_leg[0](0));
    //printf("Y Force Input: %f, ", _F_leg[0](1));
    //printf("Z Force Input: %f, ", _F_leg[0](2));
    //printf("\n");
    //printf("\n");

    printf("*** Balance Leg Forces (unit: N) ***\n");
    printf("X Force Input: %f, ", _F_leg[0](0));
    printf("Y Force Input: %f, ", _F_leg[0](1));
    printf("Z Force Input: %f, ", _F_leg[0](2));
    printf("\n");
    printf("\n");

    printf("*** Torque Input (unit: Nm) ***\n");
    printf("Hip AA Input: %f, ", _tau_leg[0](0));
    printf("Hip FE Input: %f, ", _tau_leg[0](1));
    printf("Knee FE Input: %f, ", _tau_leg[0](2));
    printf("\n");
    printf("\n");

    // printf("*** Gravity Compensation Torque Input(unit: Nm) ***\n");
    // printf("Hip AA Input: %f, ", _G_leg[0](0));
    // printf("Hip FE Input: %f, ", _G_leg[0](1));
    // printf("Knee FE Input: %f, ", _G_leg[0](2));
    // printf("\n");
    // printf("\n");

    printf("*********************** Right Front Leg **********************************\n\n");
    //printf("*** Actual Position in Joint Space (unit: deg) ***\n");
    //printf("q(1): %f, ", _q_leg[1](0) * R2D);
    //printf("q(2): %f, ", _q_leg[1](1) * R2D);
    //printf("q(3): %f, ", _q_leg[1](2) * R2D);
    //printf("\n");
    //printf("\n");

    //printf("*** Actual Position in Task Space (unit: m) ***\n");
    //printf("x(1): %f, ", _p_leg[1](0) * 1);
    //printf("x(2): %f, ", _p_leg[1](1) * 1);
    //printf("x(3): %f, ", _p_leg[1](2) * 1);
    //printf("\n");
    //printf("\n");

    //printf("*** Virtual Leg Forces (unit: N) ***\n");
    //printf("X Force Input: %f, ", _F_leg[1](0));
    //printf("Y Force Input: %f, ", _F_leg[1](1));
    //printf("Z Force Input: %f, ", _F_leg[1](2));
    //printf("\n");
    //printf("\n");

    printf("*** Balance Leg Forces (unit: N) ***\n");
    printf("X Force Input: %f, ", _F_leg[1](0));
    printf("Y Force Input: %f, ", _F_leg[1](1));
    printf("Z Force Input: %f, ", _F_leg[1](2));
    printf("\n");
    printf("\n");

    printf("*** Torque Input (unit: Nm) ***\n");
    printf("Hip AA Input: %f, ", _tau_leg[1](0));
    printf("Hip FE Input: %f, ", _tau_leg[1](1));
    printf("Knee FE Input: %f, ", _tau_leg[1](2));
    printf("\n");
    printf("\n");

    // printf("*** Gravity Compensation Torque Input(unit: Nm) ***\n");
    // printf("Hip AA Input: %f, ", _G_leg[1](0));
    // printf("Hip FE Input: %f, ", _G_leg[1](1));
    // printf("Knee FE Input: %f, ", _G_leg[1](2));
    // printf("\n");
    // printf("\n");

    printf("*********************** Left Hind Leg **********************************\n\n");
    //printf("*** Actual Position in Joint Space (unit: deg) ***\n");
    //printf("q(1): %f, ", _q_leg[2](0) * R2D);
    //printf("q(2): %f, ", _q_leg[2](1) * R2D);
    //printf("q(3): %f, ", _q_leg[2](2) * R2D);
    //printf("\n");
    //printf("\n");

    //printf("*** Actual Position in Task Space (unit: m) ***\n");
    //printf("x(1): %f, ", _p_leg[2](0) * 1);
    //printf("x(2): %f, ", _p_leg[2](1) * 1);
    //printf("x(3): %f, ", _p_leg[2](2) * 1);
    //printf("\n");
    //printf("\n");

    //printf("*** Virtual Leg Forces (unit: N) ***\n");
    //printf("X Force Input: %f, ", _F_leg[2](0));
    //printf("Y Force Input: %f, ", _F_leg[2](1));
    //printf("Z Force Input: %f, ", _F_leg[2](2));
    //printf("\n");
    //printf("\n");

    printf("*** Balance Leg Forces (unit: N) ***\n");
    printf("X Force Input: %f, ", _F_leg[2](0));
    printf("Y Force Input: %f, ", _F_leg[2](1));
    printf("Z Force Input: %f, ", _F_leg[2](2));
    printf("\n");
    printf("\n");

    printf("*** Torque Input (unit: Nm) ***\n");
    printf("Hip AA Input: %f, ", _tau_leg[2](0));
    printf("Hip FE Input: %f, ", _tau_leg[2](1));
    printf("Knee FE Input: %f, ", _tau_leg[2](2));
    printf("\n");
    printf("\n");

    // printf("*** Gravity Compensation Torque Input(unit: Nm) ***\n");
    // printf("Hip AA Input: %f, ", _G_leg[2](0));
    // printf("Hip FE Input: %f, ", _G_leg[2](1));
    // printf("Knee FE Input: %f, ", _G_leg[2](2));
    // printf("\n");
    // printf("\n");

    printf("*********************** Right Hind Leg **********************************\n\n");
    //printf("*** Actual Position in Joint Space (unit: deg) ***\n");
    //printf("q(1): %f, ", _q_leg[3](0) * R2D);
    //printf("q(2): %f, ", _q_leg[3](1) * R2D);
    //printf("q(3): %f, ", _q_leg[3](2) * R2D);
    //printf("\n");
    //printf("\n");

    //printf("*** Actual Position in Task Space (unit: m) ***\n");
    //printf("x(1): %f, ", _p_leg[3](0) * 1);
    //printf("x(2): %f, ", _p_leg[3](1) * 1);
    //printf("x(3): %f, ", _p_leg[3](2) * 1);
    //printf("\n");
    //printf("\n");

    //printf("*** Virtual Leg Forces (unit: N) ***\n");
    //printf("X Force Input: %f, ", _F_leg[3](0));
    //printf("Y Force Input: %f, ", _F_leg[3](1));
    //printf("Z Force Input: %f, ", _F_leg[3](2));
    //printf("\n");
    //printf("\n");

    printf("*** Balance Leg Forces (unit: N) ***\n");
    printf("X Force Input: %f, ", _F_leg[3](0));
    printf("Y Force Input: %f, ", _F_leg[3](1));
    printf("Z Force Input: %f, ", _F_leg[3](2));
    printf("\n");
    printf("\n");

    printf("*** Torque Input (unit: Nm) ***\n");
    printf("Hip AA Input: %f, ", _tau_leg[3](0));
    printf("Hip FE Input: %f, ", _tau_leg[3](1));
    printf("Knee FE Input: %f, ", _tau_leg[3](2));
    printf("\n");
    printf("\n");

    // printf("*** Gravity Compensation Torque Input(unit: Nm) ***\n");
    // printf("Hip AA Input: %f, ", _G_leg[3](0));
    // printf("Hip FE Input: %f, ", _G_leg[3](1));
    // printf("Knee FE Input: %f, ", _G_leg[3](2));
    // printf("\n");
    // printf("\n");

    count = 0;
  }
  count++;
}

} // namespace legged_robot_controller
PLUGINLIB_EXPORT_CLASS(legged_robot_controller::MainController, controller_interface::ControllerBase)
