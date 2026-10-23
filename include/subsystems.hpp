#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor Channel(9);
inline pros::MotorGroup FourBar({13,-20});
inline ez::Piston MatchLoader('C');
inline ez::Piston ScoreSwitcher('A');
inline ez::Piston Hood('B');
inline ez::Piston DescoreWing('H');