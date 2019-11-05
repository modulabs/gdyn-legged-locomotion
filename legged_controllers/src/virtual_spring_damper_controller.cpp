#include <legged_controllers/virtual_spring_damper_controller.h>

void VirtualSpringDamperController::init()
{
    for (int i=0; i<4; i++)
	{
        // leg stiffness and damping
        _kp_leg[i](0) = 5000.0;
        _kp_leg[i](1) = 5000.0;
        _kp_leg[i](2) = 5000.0;

        _kd_leg[i](0) = 300.0;
        _kd_leg[i](1) = 300.0;
        _kd_leg[i](2) = 300.0;
    }

    // hip position offset from base to hip joint
    _x_offset[0].p(0) = 0.3735;  
    _x_offset[0].p(1) = 0.207;

    _x_offset[1].p(0) = 0.3735;  
    _x_offset[1].p(1) = -0.207;

    _x_offset[2].p(0) = -0.3735;  
    _x_offset[2].p(1) = 0.207;

    _x_offset[3].p(0) = -0.3735;  
    _x_offset[3].p(1) = -0.207;  
  
}

void VirtualSpringDamperController::update(quadruped_robot::QuadrupedRobot& robot, std::array<Eigen::Vector3d, 4>& F_leg)
{

  // Set Input
  _p_leg = robot._p_body2leg;
  _v_leg = robot._v_body2leg;
  _G_leg = robot._trq_grav_leg;

  // Calculate
  for (size_t i=0; i<4; i++)
  {
    if (robot.getController(i) == quadruped_robot::controllers::VirtualSpringDamper)
    {
      // desired foot position/velocity form hip joint
      _xd[i].p(0) = robot._p_body2leg_d[i][0];
      _xd[i].p(1) = robot._p_body2leg_d[i][1];
      _xd[i].p(2) = robot._p_body2leg_d[i][2];

      _xd_dot[i].vel(0) = 0.0;
      _xd_dot[i].vel(1) = 0.0;
      _xd_dot[i].vel(2) = 0.0;

      // convert hip position decription from base to hip joint
      _p_leg[i](0) = _p_leg[i](0) - _x_offset[i].p(0);
      _p_leg[i](1) = _p_leg[i](1) - _x_offset[i].p(1);

      // virtual spring-damper controller
      _F_leg[i](0) = _kp_leg[i](0)*(_xd[i].p(0) - _p_leg[i](0)) + _kd_leg[i](0)*(_xd_dot[i].vel(0) - _v_leg[i](0)) + _G_leg[i](0);
      _F_leg[i](1) = _kp_leg[i](1)*(_xd[i].p(1) - _p_leg[i](1)) + _kd_leg[i](1)*(_xd_dot[i].vel(1) - _v_leg[i](1)) + _G_leg[i](1);
      _F_leg[i](2) = _kp_leg[i](2)*(_xd[i].p(2) - _p_leg[i](2)) + _kd_leg[i](2)*(_xd_dot[i].vel(2) - _v_leg[i](2)) + _G_leg[i](2);

      F_leg[i] = _F_leg[i];
    }
  }
}

