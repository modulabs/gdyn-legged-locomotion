#pragma once

#include <array>
#include <numeric>
#include <utility/math_func.h>

// kdl
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

// qpOASES
#include <qpOASES.hpp>
#define GRAVITY_CONSTANT 9.81

#include <ros/console.h>
//
#include <legged_robot/quadruped_robot.h>

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Matrix;
using Eigen::Dynamic;
using Eigen::ColMajor;
using Eigen::RowMajor;
using Eigen::NoChange;

class BalanceController
{
public:
    BalanceController() {}

    void init();

//    void setControlInput(const Eigen::Vector3d& p_body_d,
//            const Eigen::Vector3d& p_body_dot_d,
//            const Eigen::Matrix3d& R_body_d,
//            const Eigen::Vector3d& w_body_d,
//			const Eigen::Vector3d& p_body,
//            const Eigen::Vector3d& p_body_dot,
//            const Eigen::Matrix3d& R_body,
//            const Eigen::Vector3d w_body,
//            const std::array<Eigen::Vector3d,4>& p_body2leg);

//    void getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg);

    void update(quadruped_robot::QuadrupedRobot& robot, std::array<Vector3d, 4>& F_leg);

public:
    // gain
    Eigen::Vector3d _kp_p, _kd_p, _kp_w, _kd_w;

    // Legs to optimize
    std::vector<size_t> _legs;

    // Optimization
    Matrix<double, 6, Dynamic, ColMajor, 6, 12> _A;
    Matrix<double, Dynamic, 1, ColMajor, 12, 1> _F;
    Matrix<double, Dynamic, 1, ColMajor, 12, 1> _F_prev;
    Matrix<double, 6, 1> _bd;
    Matrix<double, 6, 6> _S;

    // QP optimization
    Matrix<double, Dynamic, Dynamic, RowMajor, 12, 12> _H;
    Matrix<double, Dynamic, Dynamic, ColMajor, 12, 12> _Alpha;
    Matrix<double, Dynamic, Dynamic, ColMajor, 12, 12> _Beta;
    Matrix<double, Dynamic, 1, ColMajor, 12, 1> _g;

    // Inequality constraint

    Matrix<double, Dynamic, Dynamic, RowMajor, 16, 12> _C;
    Matrix<double, 1, Dynamic, RowMajor, 1, 12> _lb, _ub;
    Matrix<double, 1, Dynamic, RowMajor, 1, 16> _ubC;
};
