#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor ClawRotation(21);

inline pros::MotorGroup dr4b({-1, 13});
inline pros::Rotation dr4bRotation(5);
inline ez::PID dr4bPID{1, 0, 4, 100, "dr4b"};

inline ez::Piston Claw('A');
