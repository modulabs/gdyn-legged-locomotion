#pragma once

#include <array>
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

//
#include <legged_robot/quadruped_robot.h>

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

    void update(quadruped_robot::QuadrupedRobot& robot, std::array<Eigen::Vector3d, 4>& F_leg);

public:
//    // optimization output
//    Eigen::Matrix<double, 12, 1> _F, _F_prev;

    // gain
    Eigen::Vector3d _kp_p, _kd_p, _kp_w, _kd_w;

    // optimization variable

};
