#pragma once

// for debugging
#include <fstream>
#include <iostream>
using namespace std;

#define MPC_Debugging
//#define MPC_Thread

#include <array>
#include <utility/math_func.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// kdl
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

// qpOASES
#include <qpOASES.hpp>

// Information of quadruped robot's kinematics, dynamics and sensor data
#include <legged_robot/quadruped_robot.h>
/*
#define SamplingTime 0.001
#define MPC_Step 3
#define Control_Step 3

#define L_00_gain 1.0
#define L_11_gain_x 1.0
#define L_11_gain_y 1.0
#define L_11_gain_z 1.0
#define L_22_gain_wx 1.0
#define L_22_gain_wy 1.0
#define L_22_gain_wz 1.0
#define L_33_gain_vx 1.0
#define L_33_gain_vy 1.0
#define L_33_gain_vz 1.0
#define L_44_gain 0.0

#define K_00_gain 0.000000001
#define K_11_gain 0.000000001
#define K_22_gain 0.000000001
#define K_33_gain 0.000000001
*/

#define SamplingTime 0.03
#define MPC_Step 3
#define Control_Step 12

#define L_00_gain 1.0
#define L_11_gain_x 1.0
#define L_11_gain_y 1.0
#define L_11_gain_z 1.0
#define L_22_gain_wx 1.0
#define L_22_gain_wy 1.0
#define L_22_gain_wz 1.0
#define L_33_gain_vx 1.0
#define L_33_gain_vy 1.0
#define L_33_gain_vz 1.0
#define L_44_gain 0.0

#define K_00_gain 0.000000000001
#define K_11_gain 0.000000000001
#define K_22_gain 0.000000000001
#define K_33_gain 0.000000000001

#define Force_min 10
#define Force_max 666

#define Gravity 9.81

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

    // command and state
    Eigen::Vector3d _p_com_d, _p_com_dot_d, _p_com, _p_com_dot;
    std::array<Eigen::Vector3d, 4> _p_leg, _p_leg_d, _F_leg;

    Eigen::Matrix3d _R_body_d, _R_body;

    Eigen::Vector3d _w_body_d, _w_body;

    // optimization output
    Eigen::Matrix<double, 12, 1> _F;

    // Define Parameter For MPC Controller
    Eigen::MatrixXd _A_d, _B_d, _B_d_d;
    Eigen::MatrixXd _Rz, _Rz_d;
    Eigen::MatrixXd _R1, _R2, _R3, _R4;
    Eigen::MatrixXd _R1_d, _R2_d, _R3_d, _R4_d;
    Eigen::MatrixXd _I3x3, _I15x15;
    Eigen::MatrixXd _I_hat, _I_hat_d;
    Eigen::MatrixXd _p_leg_1_skew, _p_leg_2_skew, _p_leg_3_skew, _p_leg_4_skew;
    Eigen::MatrixXd _p_leg_1_skew_d, _p_leg_2_skew_d, _p_leg_3_skew_d, _p_leg_4_skew_d;
    Eigen::MatrixXd _T15X15, _T12X12;
    Eigen::MatrixXd _A_qp, _B_qp, _Temp;
    Eigen::MatrixXd _L_d, _K_d;
    Eigen::MatrixXd _H_qp, _L_qp, _K_qp;
    Eigen::MatrixXd _g_qp, _x0, _xref, _xref_qp;

    Eigen::MatrixXd _C_1leg, _C_4leg, _C_qp;
    Eigen::MatrixXd _lbC_1leg, _lbC_4leg, _lbC_qp;
    Eigen::MatrixXd _ub_1leg, _ub_4leg, _ub_qp;
    Eigen::MatrixXd _lb_1leg, _lb_4leg, _lb_qp;
};