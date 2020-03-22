#include "legged_robot_controller/motion_planner.h"


void MotionPlanner::setGaitPattern(quadruped_robot::gait_patterns::GaitPattern gait_pattern, int int_param)
{
  _robot->_gait_pattern_old = _robot->_gait_pattern;
  _robot->_gait_pattern = gait_pattern;
  _robot->_gait_pattern_start = true;
}

void MotionPlanner::update()
{
  // to do: make below structure to behavior tree
  switch(_robot->_gait_pattern)
  {
  case quadruped_robot::gait_patterns::Falling:
    updateFalling();
    break;
  case quadruped_robot::gait_patterns::Standing:
    if (_robot->_gait_pattern_start)
    {
      startStanding();
      _robot->_gait_pattern_start = false;
    }
    else
      updateStanding();
    break;
  case quadruped_robot::gait_patterns::Manipulation:
    break;
  case quadruped_robot::gait_patterns::Walking:
  case quadruped_robot::gait_patterns::Pacing:
  case quadruped_robot::gait_patterns::Trotting:
  case quadruped_robot::gait_patterns::Bounding:
  case quadruped_robot::gait_patterns::Galloping:
  case quadruped_robot::gait_patterns::Pronking:
    updateMoving();
    break;
  default:
    break;
  }
}

void MotionPlanner::updateFalling()
{
  _robot->setController(4, quadruped_robot::controllers::VirtualSpringDamper);

  _robot->_p_body2leg_d[0] = Vector3d(0, 0, -0.5);
  _robot->_p_body2leg_d[1] = Vector3d(0, 0, -0.5);
  _robot->_p_body2leg_d[2] = Vector3d(0, 0, -0.5);
  _robot->_p_body2leg_d[3] = Vector3d(0, 0, -0.5);

}

void MotionPlanner::startStanding()
{
  ROS_INFO("[Motion Planner] Start Standing");

  _robot->_pose_body_d._pos = _robot->_pose_body._pos;
  _robot->_pose_body_d._rot_quat.setIdentity();

  _robot->setController(4, quadruped_robot::controllers::BalancingQP);
//  _robot->setController(4, quadruped_robot::controllers::BalancingMPC);
//  _robot->setController(4, quadruped_robot::controllers::BalancingMPCWholeBody);
}

void MotionPlanner::updateStanding()
{
  // Limit body posture so as to enable balancing

}

void MotionPlanner::updateMoving()
{
  // gait pattern modulator
}
