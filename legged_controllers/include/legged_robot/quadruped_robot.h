#pragma once

#include <utility/math_func.h>
#include <array>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>

// ros
#include <ros/console.h>

// kdl
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics: position
#include <kdl/chainfksolvervel_recursive.hpp> // forward kinematics: velocity
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
// #include <kdl/chainjnttojacdotsolver.hpp>     // @ To do: jacobian derivative
#include <kdl/chaindynparam.hpp>              // inverse dynamics

namespace quadruped_robot
{

namespace controllers
{
  enum Controller
  {
    VirtualSpringDamper,
    BalancingQP,
    BalancingMPC,
    BalancingMPCWholeBody,
    Swing
  };
  inline const char* ControllerToString(Controller controller)
  {
      switch (controller)
      {
          case VirtualSpringDamper:   return "VSD";
          case BalancingQP:           return "BalQP";
          case BalancingMPC:          return "BalMPC";
          case BalancingMPCWholeBody: return "BalMPCWB";
          case Swing:                 return "Swing";
          default:                    return "---";
      }
  }  
}




class QuadrupedRobot
{
public:
  QuadrupedRobot();
  ~QuadrupedRobot();

  // get function
  controllers::Controller getController(size_t i);
  std::string getControllerName(size_t i);

  // set function
  void setController(size_t i, controllers::Controller controller);

  // main routine
  int init();
  void updateSensorData(const std::array<Eigen::Vector3d, 4>& q, const std::array<Eigen::Vector3d, 4>& q_dot,
                        const Pose& pose_body, const PoseVel& pose_vel_body,
                        const std::array<int, 4>& contact_states);
  void calKinematicsDynamics();



  // parameter
  double          _m_body;
  double          _mu_foot;     // between foot and ground
  Eigen::Matrix3d _I_com_body;
  Eigen::Vector3d _p_body2com;

  // controller
  std::array<controllers::Controller, 4> _controller;

  // state
  std::array<int, 4> _contact_states;

  // joint space
  std::array<KDL::JntArray,4> _kdl_q_leg, _kdl_qdot_leg;
  std::array<Eigen::Vector3d, 4> _q_leg, _q_leg_d, _qdot_leg, _qdot_leg_d, _qddot_leg_d;
  std::array<Eigen::Matrix3d, 4> _inertia_mat_leg, _coriolis_mat_leg;
  std::array<Eigen::Vector3d, 4> _trq_leg, _trq_idyn_leg, _trq_inertia_leg, _trq_coriolis_leg, _trq_grav_leg;

  // leg
  std::array<Eigen::Vector3d, 4> _p_body2leg, _p_world2leg, _p_world2leg_d;
  std::array<Eigen::Vector3d, 4> _v_body2leg, _v_world2leg;
  std::array<Eigen::Vector3d, 4> _F_body2leg, _F_world2leg, _F_world2leg_prev;

  std::array<Eigen::Vector3d, 4> _p_body2leg_d;

  std::array<double, 4> _T_stance;
  std::array<double, 4> _T_swing;
  std::array<double, 4> _S_stance;
  std::array<double, 4> _S_swing;
  std::array<double, 4> _t_leg;
  std::array<int, 4> _contact_state;

  // body
  Pose  _pose_body, _pose_body_d;         // world to body
  Pose  _pose_com, _pose_com_d;           // world to com
  PoseVel _pose_vel_body, _pose_vel_body_d; // world to body
  PoseVel _pose_vel_com, _pose_vel_com_d;   // world to COM
  PoseAcc _pose_acc_body_d;                 // world to body

  // kinematics, dynamics
  // kdl
  KDL::Vector _kdl_gravity;
  KDL::Tree 	_kdl_tree;
  std::array<KDL::Chain, 4>	_kdl_chain;
  std::array<boost::shared_ptr<KDL::ChainFkSolverPos_recursive>, 4> _kdl_fkin_solver;
  std::array<boost::shared_ptr<KDL::ChainJntToJacSolver>, 4> _kdl_jacobian_solver;
  // std::array<boost::scoped_ptr<KDL::ChainJntToJacDotSolver>, 4> _jnt_to_jac_dot_solver;  @ To do
  std::array<boost::shared_ptr<KDL::ChainDynParam>, 4> _kdl_idyn_solver;

  std::array<KDL::Jacobian, 4> _kdl_J_leg;
  std::array<Eigen::MatrixXd, 4> _Jv_leg;

  //
  std::array<KDL::JntSpaceInertiaMatrix, 4> _kdl_inertia_mat_leg; // joint space intertia matrix
  std::array<KDL::JntArray, 4> _kdl_trq_coriolis_leg;              // coriolis vector
  std::array<KDL::JntArray, 4> _kdl_trq_grav_leg;              // gravity torque vector
};

}
