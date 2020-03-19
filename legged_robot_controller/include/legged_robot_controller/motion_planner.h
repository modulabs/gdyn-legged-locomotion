#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include "legged_robot_controller/quadruped_robot.h"

class MotionPlanner
{
public:
  void init(quadruped_robot::QuadrupedRobot* robot) {_robot = robot;}

  void setGaitPattern(quadruped_robot::gait_patterns::GaitPattern gait_pattern, int int_param=0);

  void update();
  void updateFalling();
  void startStanding();
  void updateStanding();
  void updateMoving();

  quadruped_robot::QuadrupedRobot* _robot;
};



#endif // MOTION_PLANNER_H

