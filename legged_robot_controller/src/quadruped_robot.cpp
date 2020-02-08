/*
  Author: Modulabs
  File Name: quadruped_robot.cpp
*/

#include "legged_robot_controller/quadruped_robot.h"


namespace quadruped_robot
{
QuadrupedRobot::QuadrupedRobot()
{
  // command and state divided into each legs (4x3)
  for (size_t i=0; i<4; i++)
  {
    // joint state
    _kdl_q_leg[i].resize(3);
    _kdl_qdot_leg[i].resize(3);

    // Jacobian
    _kdl_J_leg[i].resize(3);

    // Dynamics
    _kdl_inertia_mat_leg[i].resize(3);
    _kdl_trq_coriolis_leg[i].resize(3);
    _kdl_trq_grav_leg[i].resize(3);
  }
}

controllers::Controller QuadrupedRobot::getController(size_t i)
{
  if (i == 4)
    return _controller[0];  // representative
  else
    return _controller[i];
}

std::string QuadrupedRobot::getControllerName(size_t i)
{
  return controllers::ControllerToString(getController(i));
}

void QuadrupedRobot::setController(size_t i, controllers::Controller controller)
{
  if (i == 4)  {
    _controller.fill(controller);
  }
  else {
    _controller[i] = controller;
  }
}

int QuadrupedRobot::init()
{
  // kdl chain
  std::string root_name, tip_name[4];

  root_name = "trunk";
  tip_name[0] = "lf_foot"; tip_name[1] = "rf_foot"; tip_name[2] = "lh_foot"; tip_name[3] = "rh_foot";

  _kdl_gravity = KDL::Vector(0.0, 0.0, -9.81);

  for (size_t i=0; i<4; i++)
  {
    if(!_kdl_tree.getChain(root_name, tip_name[i], _kdl_chain[i]))
    {
      return -1;
    }
    else
    {
      _kdl_fkin_solver[i].reset(new KDL::ChainFkSolverPos_recursive(_kdl_chain[i]));
      _kdl_jacobian_solver[i].reset(new KDL::ChainJntToJacSolver(_kdl_chain[i]));
      // _jnt_to_jac_dot_solver[i].reset(new KDL::ChainJntToJacDotSolver(_kdl_chain[i])); @ To do
      _kdl_idyn_solver[i].reset(new KDL::ChainDynParam(_kdl_chain[i],_kdl_gravity));
    }
  }
}

void QuadrupedRobot::updateSensorData(const std::array<Eigen::Vector3d, 4>& q_leg, const std::array<Eigen::Vector3d, 4>& qdot_leg,
                        const Pose& pose_body, const PoseVel& pose_vel_body,
                        const std::array<int, 4>& contact_states)
{
  _q_leg = q_leg;
  _qdot_leg = qdot_leg;
  for (size_t i=0; i<4; i++)
  {
    for (size_t j=0; j<3; j++)
    {
      _kdl_q_leg[i](j) = _q_leg[i][j];
      _kdl_qdot_leg[i](j) = _qdot_leg[i][j];
    }
  }
  _pose_body = pose_body;
  _pose_vel_body = pose_vel_body;
  _contact_states = contact_states;
}

void QuadrupedRobot::calKinematicsDynamics()
{
  // previous variable
  _F_world2leg_prev = _F_world2leg;

  //
  std::array<KDL::Frame,4> frame_leg;

  for (size_t i=0; i<4; i++)
  {
    _kdl_fkin_solver[i]->JntToCart(_kdl_q_leg[i], frame_leg[i]);
    _kdl_jacobian_solver[i]->JntToJac(_kdl_q_leg[i], _kdl_J_leg[i]);

    _Jv_leg[i] = _kdl_J_leg[i].data.block(0,0,3,3);
    _p_body2leg[i] = Eigen::Vector3d(frame_leg[i].p.data);
    _v_body2leg[i] = _Jv_leg[i]*_kdl_qdot_leg[i].data;

    // _jnt_to_jac_dot_solver[i]->JntToJacDot(); // @ To do: Jacobian Dot Calculation

    _kdl_idyn_solver[i]->JntToMass(_kdl_q_leg[i], _kdl_inertia_mat_leg[i]);
    _kdl_idyn_solver[i]->JntToCoriolis(_kdl_q_leg[i], _kdl_qdot_leg[i], _kdl_trq_coriolis_leg[i]);
    _kdl_idyn_solver[i]->JntToGravity(_kdl_q_leg[i], _kdl_trq_grav_leg[i]);

    for (size_t j=0; j<3; j++)
    {
      for (size_t k=0; k<3; k++)
      {
        _inertia_mat_leg[i](j,k) = _kdl_inertia_mat_leg[i](j,k);
      }

      _trq_coriolis_leg[i](j) = _kdl_trq_coriolis_leg[i](j);
      _trq_grav_leg[i](j) = _kdl_trq_grav_leg[i](j);
    }
  }

  // to world coordinates
  for (size_t i=0; i<4; i++)
      _p_world2leg[i] = _pose_body * _p_body2leg[i];

  // body to com
  _pose_com_d._pos = _pose_body_d * _p_body2com;
  _pose_com_d._rot_quat = _pose_body_d._rot_quat;

  _pose_com._pos = _pose_body * _p_body2com;
  _pose_com._rot_quat = _pose_body._rot_quat;

  _pose_vel_com_d._linear = _pose_vel_body_d._linear + skew(_pose_vel_body_d._linear) * _pose_body_d._rot_quat.toRotationMatrix() * _p_body2com;
  _pose_vel_com_d._angular = _pose_vel_body_d._angular;

  _pose_vel_com._linear = _pose_vel_body._linear + skew(_pose_vel_body._linear) * _pose_body._rot_quat.toRotationMatrix() * _p_body2com;
  _pose_vel_com._angular = _pose_vel_body._angular;
}
}
