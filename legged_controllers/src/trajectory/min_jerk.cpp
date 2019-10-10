#include <trajectory/min_jerk.h>

namespace trajectory
{

MinJerk::MinJerk()
{
    _state = -1;
    _t = 0.0;
}

double MinJerk::setTrajInput(double t, double start_pos, double end_pos, double duration)
{
    _state = 0;
    _t = t;
    _start_pos = start_pos;
    _duration = duration;
    _a3 = (20.0*end_pos - 20.0*start_pos) / (2.0*duration*duration*duration);
    _a4 = (30.0*start_pos - 30*end_pos) / (2.0*duration*duration*duration*duration);
    _a5 = (12.0*end_pos - 12.0*start_pos) / (2.0*duration*duration*duration*duration*duration);
    _end_pos = end_pos;
}

int MinJerk::getTrajOutput(double t, double& pos, double& vel, double& acc)
{
    double dt = t - _t;

    if (dt < 0)
    {
        _state = -1;
        pos = _start_pos;
        vel = 0;
        acc = 0;
    }
    else if (dt <_duration)
    {
        _state = 1;
        pos = getPos(dt);
        vel = getVel(dt);
        acc = getAcc(dt);
    }
    else
    {
        _state = 2;
        pos = _end_pos;
        vel = 0;
        acc = 0;
    }

    return _state;
}

double MinJerk::getPos(double dt)
{
    return _start_pos + _a3*dt*dt*dt + _a4*dt*dt*dt*dt + _a5*dt*dt*dt*dt*dt;
}

double MinJerk::getVel(double dt)
{
    return 3.0*_a3*dt*dt + 4.0*_a4*dt*dt*dt + 5.0*_a5*dt*dt*dt*dt;
}

double MinJerk::getAcc(double dt)
{
    return 6.0*_a3*dt + 12.0*_a4*dt*dt + 20.0*_a5*dt*dt*dt;
}

}
    