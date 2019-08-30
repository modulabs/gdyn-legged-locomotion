#include <legged_controllers/swing_controller.h>

void SwingController::init()
{
    for (int i=0; i<4; i++)
	{
        // gains: kp, kd
        _kp_leg[i].resize(3);
        _kd_leg[i].resize(3);

        _kp_leg[i](0) = 0.0;
        _kp_leg[i](1) = 0.0;
        _kp_leg[i](2) = 0.0;

        _kd_leg[i](0) = 0.0;
        _kd_leg[i](1) = 0.0;
        _kd_leg[i](2) = 0.0;
    }
}

void SwingController::setControlInput(const std::array<KDL::Vector, 4>& p_leg, 
                                        const std::array<Eigen::Vector3d, 4>& v_leg,
                                        const std::array<Eigen::MatrixXd, 4>& Jv_leg,
                                        const std::array<KDL::JntSpaceInertiaMatrix, 4>& M_leg,
                                        const std::array<KDL::JntArray, 4>& C_leg,
                                        const std::array<KDL::JntArray, 4>& G_leg)
{
    _p_leg = p_leg;
    _v_leg = v_leg;
    _M_leg = M_leg;
    _C_leg = C_leg
    _G_leg = G_leg;
}

void SwingController::getControlOutput(std::array<Eigen::Vector3d, 4>& F_leg)
{
    F_leg = _F_leg;
}

void SwingController::compute()
{
    for (int i = 0; i < 4; i++)
    {
        // desired foot position/velocity form hip joint
        _xd[i].p(0) = 0.0; 
        _xd[i].p(1) = 0.0; 
        _xd[i].p(2) = -0.4;

        _xd_dot[i].vel(0) = 0.0;
        _xd_dot[i].vel(1) = 0.0;
        _xd_dot[i].vel(2) = 0.0;

        // swing controller
        _F_leg[i](0) = 0.0;
        _F_leg[i](1) = 0.0;
        _F_leg[i](2) = 0.0;
    }
}

