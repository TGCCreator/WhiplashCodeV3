#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor ClawRotation(9);

inline pros::MotorGroup FourBar({-1,10});

inline ez::Piston Claw('C');
