#pragma once

#include <utility/math_func.h>
#include <array>
#include <boost/scoped_ptr.hpp>

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


typedef Twist PoseVel;
typedef Twist PoseAcc;

class QuadrupedRobot
{
public:
  QuadrupedRobot();
  ~QuadrupedRobot();

  int init();
  void updateSensorData(const std::array<Eigen::Vector3d, 4>& q, const std::array<Eigen::Vector3d, 4>& q_dot,
                        const Pose& pose_body, const PoseVel& pose_vel_body);
  void calKinematics();


  // parameter
  double          _m_body;
  double          _mu_foot;     // between foot and ground
  Eigen::Matrix3d _I_com_body;
  Eigen::Vector3d _p_body2com;


  // state
  std::array<bool, 4> _contact_state;

  // joint space
  std::array<KDL::JntArray,4> _q_leg_kdl, _qdot_leg_kdl;
  std::array<Eigen::Vector3d, 4> _q_leg, _q_leg_d, _qdot_leg, _qdot_leg_d, _qddot_leg_d;
  std::array<Eigen::Vector3d, 4> _trq_leg, _trq_idyn_leg, _trq_grav_leg;

  // leg
  std::array<Eigen::Vector3d, 4> _p_body2leg, _p_world2leg;  // body to leg
  std::array<Eigen::Vector3d, 4> _v_body2leg, _v_world2leg;  // body to leg
  std::array<Eigen::Vector3d, 4> _F_body2leg, _F_world2leg;  // body to leg

  // body
  Pose  _pose_body, _pose_body_d;         // world to body
  Pose  _pose_com, _pose_com_d;           // world to com
  PoseVel _pose_vel_body, _pose_vel_body_d; // world to body
  PoseVel _pose_vel_com, _pose_vel_com_d;   // world to COM
  PoseAcc _pose_acc_body_d;                 // world to body

  // kinematics, dynamics
  // kdl
  boost::scoped_ptr<KDL::Vector> _gravity;
  KDL::Tree 	_kdl_tree;
  std::array<KDL::Chain, 4>	_kdl_chain;
  std::array<boost::scoped_ptr<KDL::ChainFkSolverPos_recursive>, 4> _fk_pos_solver;
  std::array<boost::scoped_ptr<KDL::ChainJntToJacSolver>, 4> _jnt_to_jac_solver;
  // std::array<boost::scoped_ptr<KDL::ChainJntToJacDotSolver>, 4> _jnt_to_jac_dot_solver;  @ To do
  std::array<boost::scoped_ptr<KDL::ChainDynParam>, 4> _id_solver;

  std::array<KDL::Jacobian, 4> _J_leg;
  std::array<Eigen::MatrixXd, 4> _Jv_leg;

  //
  std::array<KDL::JntSpaceInertiaMatrix, 4> _M_leg; // joint space intertia matrix
  std::array<KDL::JntArray, 4> _C_leg;              // coriolis vector
  std::array<KDL::JntArray, 4> _G_leg;              // gravity torque vector
};
