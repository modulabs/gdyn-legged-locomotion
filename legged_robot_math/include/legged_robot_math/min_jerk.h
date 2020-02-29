/*
  Author: Modulabs
  File Name: min_jerk.h
*/

#pragma once


namespace trajectory
{
class MinJerk
{
public:
    MinJerk();

    double setTrajInput(double t, double start, double end, double duration);

    int getTrajOutput(double t, double& pos, double& vel, double& acc);

    double getPos(double dt);
    double getVel(double dt);
    double getAcc(double dt);

private:
    int _state; // -1: before start, 0: set moving, 1: moving, 2: finish moving
    double _start_pos, _a3, _a4, _a5, _end_pos;
    double _duration, _t;
};
}