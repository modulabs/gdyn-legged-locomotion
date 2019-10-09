#pragma once

#include <legged_robot/leg.h>
#include <legged_robot/body.h>
#include <array>

class LeggedRobot
{
public:
    Body _body;
    std::array<Leg, 4> _legs;
};
