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

#define SamplingTime 0.001
#define MPC_Step 40

#define L_00_gain 1.0
#define L_11_gain 50.0
#define L_22_gain 1.0
#define L_33_gain 1.0
#define L_44_gain 0.0

#define K_00_gain 0.000001
#define K_11_gain 0.000001
#define K_22_gain 0.000001
#define K_33_gain 0.000001
#define K_44_gain 0.0

#define Force_min 10
#define Force_max 666

class MPCController
{
public:
    MPCController() {}

    void init(double m_body, const KDL::RotationalInertia& I_com, double mu);

    void setControlInput(const KDL::Vector& p_com_d, 
            const KDL::Vector& p_com_dot_d, 
            const KDL::Rotation& R_body_d, 
            const KDL::Vector& w_body_d,
			const KDL::Vector& p_com,
            const KDL::Vector& p_com_dot, 
            const std::array<KDL::Vector,4>& p_leg,
            const KDL::Rotation& R_body,
            const KDL::Vector w_body);

    void getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg);

    void update();

    void skew_symmetric(KDL::Vector &v_, Eigen::MatrixXf skew_mat_);

public:
    KDL::Vector _p_com_d, _p_com_dot_d, _p_com_ddot_d, _p_com, _p_com_dot;
    std::array<KDL::Vector, 4> _p_leg, _F_leg;

    KDL::Rotation _R_body_d, _R_body; 

    KDL::Vector _w_body_d, _w_body_dot_d, _w_body;

    double _m_body;
    KDL::RotationalInertia _I_com;

    double _mu; // friction coefficient on ground  
};