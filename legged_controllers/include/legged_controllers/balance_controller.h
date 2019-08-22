#pragma once

#include <array>

// kdl
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

// qpOASES
#include <qpOASES.hpp>

class BalanceController
{
public:
    BalanceController() {}

    void init(double m_body, const KDL::RotationalInertia& I_com, double mu);

    void setControlInput(const KDL::Vector& p_com_d, 
            const KDL::Vector& p_com_dot_d, 
            const KDL::Rotation& R_body_d, 
            const KDL::Vector& w_body_d,
			const KDL::Vector& p_com, 
            const std::array<KDL::Vector,4>& p_leg,
            const KDL::Rotation& R_body,
            const KDL::Vector w_body);

    void getControlOutput(std::array<KDL::Vector, 4>& F_leg);

    void update();

public:
    KDL::Vector _p_com_d, _p_com_dot_d, _p_com_ddot_d, _p_com;
    std::array<KDL::Vector, 4> _p_leg, _F_leg;

    KDL::Rotation _R_body_d, _R_body;   // have to change angle-axis or quaternion

    KDL::Vector _w_body_d, _w_body_dot_d, _w_body;

    double _m_body;
    KDL::RotationalInertia _I_com;

    double _mu; // friction coefficient on ground  
};