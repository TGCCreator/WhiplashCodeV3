#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor ClawRotation(2);

inline pros::MotorGroup FourBar({-1,13});

inline ez::Piston Claw('A');
