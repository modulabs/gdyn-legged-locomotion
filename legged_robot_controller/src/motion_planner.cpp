#include "legged_robot_controller/motion_planner.h"


void MotionPlanner::setGaitPattern(quadruped_robot::gait_patterns::GaitPattern gait_pattern, int int_param)
{
  _robot->_gait_pattern_old = _robot->_gait_pattern;
  _robot->_gait_pattern = gait_pattern;
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
    if (_robot->_gait_pattern_old != quadruped_robot::gait_patterns::Standing)
      startStanding();
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
  for (size_t i = 0; i < 4; i++)
    _robot->_p_body2leg_d[i] = Vector3d(0, 0, -0.4);
}

void MotionPlanner::startStanding()
{
  ROS_INFO("[Motion Planner] Start Standing");

  _robot->_pose_com_d = _robot->_pose_body;

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
