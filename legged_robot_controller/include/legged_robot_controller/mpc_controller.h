/*
  Author: Modulabs
  File Name: mpc_controller.h
*/

#pragma once

#include <array>
#include <fstream>
#include <iostream>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <qpOASES/qpOASES.hpp>

#include "legged_robot_math/math_func.h"
#include "legged_robot_controller/quadruped_robot.h"

#undef MPC_Debugging

#define SamplingTime 0.01
#define MPC_Step 3
#define Control_Step 6

#define L_00_gain 1.0
#define L_11_gain_x 1.0
#define L_11_gain_y 1.0
#define L_11_gain_z 50.0
#define L_22_gain_wx 1.0
#define L_22_gain_wy 1.0
#define L_22_gain_wz 1.0
#define L_33_gain_vx 1.0
#define L_33_gain_vy 1.0
#define L_33_gain_vz 1.0
#define L_44_gain 0.0

#define K_gain 0.000000000001 

#define Force_min 10
#define Force_max 666

#define Gravity 9.81

using namespace std;


struct LegContactState
{
  int ContactTotalNum;
  bool LegState[4];
};

class MPCController
{
public:
  MPCController() {}

  void init();

  void setControlData(quadruped_robot::QuadrupedRobot &robot);
  void calControlInput();
  void getControlInput(quadruped_robot::QuadrupedRobot &robot, std::array<Eigen::Vector3d, 4> &F_leg);
  void cal_A_d();
  void cal_B_d_and_B_d_d();
  
public:
  bool _start;
  volatile bool _update;
  int _step;
      
  // parameter
  double _m_body;
  Eigen::Matrix3d _I_com;

  double _mu; // friction coefficient on ground

  LegContactState _LegContactState;

  // command and state
  Eigen::Vector3d _p_com_d, _p_com_dot_d, _p_com, _p_com_dot;
  std::array<Eigen::Vector3d, 4> _p_leg, _p_leg_d, _F_leg;

  Eigen::Matrix3d _R_body_d, _R_body;

  Eigen::Vector3d _w_body_d, _w_body;

  // optimization output
  std::array<Eigen::Vector3d, 4> _F;

  // Define Parameter For MPC Controller
  Eigen::MatrixXd _A_d, _B_d, _B_d_d;
  Eigen::MatrixXd _Rz, _Rz_d;
  Eigen::MatrixXd _R1, _R2, _R3, _R4;
  Eigen::MatrixXd _R1_d, _R2_d, _R3_d, _R4_d;
  Eigen::MatrixXd _I3x3, _I15x15;
  Eigen::MatrixXd _I_hat, _I_hat_d;
  Eigen::MatrixXd _p_leg_1_skew, _p_leg_2_skew, _p_leg_3_skew, _p_leg_4_skew;
  Eigen::MatrixXd _p_leg_1_skew_d, _p_leg_2_skew_d, _p_leg_3_skew_d, _p_leg_4_skew_d;
  Eigen::MatrixXd _A_qp, _B_qp, _Temp;
  Eigen::MatrixXd _L_d, _K_d;
  Eigen::MatrixXd _H_qp, _L_qp, _K_qp;
  Eigen::MatrixXd _g_qp, _x0, _xref, _xref_qp;

  Eigen::MatrixXd _C_1leg, _C_totalleg, _C_qp;
  Eigen::MatrixXd _lbC_1leg, _lbC_totalleg, _lbC_qp;
  Eigen::MatrixXd _ub_1leg, _ub_totalleg, _ub_qp;
  Eigen::MatrixXd _lb_1leg, _lb_totalleg, _lb_qp;
};
