//#pragma once

#include <array>

// kdl
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

class SwingController
{
public:
    SwingController() {}

    void init();

    void setControlInput(const std::array<Eigen::Vector3d, 4>& p_leg, 
                        const std::array<Eigen::Vector3d, 4>& v_leg,
                        const std::array<Eigen::MatrixXd, 4>& Jv_leg,
                        const std::array<KDL::JntSpaceInertiaMatrix, 4>& M_leg,
                        const std::array<KDL::JntArray, 4>& C_leg, 
                        const std::array<KDL::JntArray, 4>& G_leg);

    void getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg);

    void compute();

public:
    std::array<KDL::JntArray, 4> _kp_leg;
    std::array<KDL::JntArray, 4> _kd_leg;
    std::array<KDL::Frame, 4> _x_offset;
    std::array<KDL::Frame, 4> _xd;
    std::array<KDL::Twist, 4> _xd_dot;
    
    std::array<Eigen::Vector3d, 4> _p_leg;
    std::array<Eigen::Vector3d, 4> _v_leg;
    std::array<Eigen::Vector3d, 4> _F_leg;

    std::array<Eigen::MatrixXd, 4> _Jv_leg;

    std::array<KDL::JntSpaceInertiaMatrix, 4> _M_leg;
    std::array<KDL::JntArray, 4> _C_leg;
    std::array<KDL::JntArray, 4> _G_leg;
};