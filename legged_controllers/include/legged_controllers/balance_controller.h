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
typedef Eigen::Matrix<double, 12, 1> Vector12d;

class BalanceController
{
public:
    BalanceController() {}

    void init(double m_body, const Eigen::Vector3d& p_com_body, const Eigen::Matrix3d& I_com, double mu);

    void setControlInput(const Eigen::Vector3d& p_body_d, 
            const Eigen::Vector3d& p_body_dot_d, 
            const Eigen::Matrix3d& R_body_d, 
            const Eigen::Vector3d& w_body_d,
			const Eigen::Vector3d& p_body, 
            const Eigen::Vector3d& p_body_dot,
            const Eigen::Matrix3d& R_body,
            const Eigen::Vector3d w_body,
            const std::array<Eigen::Vector3d,4>& p_body2leg);

    void getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg);

    void update();

public:
    // parameter
    double _m_body;
    Eigen::Vector3d _p_body2com;
    Eigen::Matrix3d _I_com;

    double _mu; // friction coefficient on ground  

    // command and state
    Eigen::Vector3d _p_com_d, _p_com_dot_d, _p_com, _p_com_dot;
    std::array<Eigen::Vector3d, 4> _p_leg, _F_leg;

    Eigen::Matrix3d _R_body_d, _R_body;   // have to change angle-axis or quaternion

    Eigen::Vector3d _w_body_d, _w_body;

    // optimization output
    Eigen::Matrix<double, 12, 1> _F, _F_prev;

    // gain
    Eigen::Vector3d _kp_p, _kd_p, _kp_w, _kd_w;
};