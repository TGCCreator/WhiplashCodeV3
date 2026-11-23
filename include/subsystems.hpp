#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor ClawRotation(21);

inline pros::MotorGroup dr4b({-1, 13});
inline pros::Rotation dr4bRotation(5);
inline ez::PID dr4bPID{25, 0, 10, 100, "dr4b"};

inline ez::Piston Claw('A');

void dr4b_set(double target);
void dr4b_wait(double tolerance = 3, int timeout_ms = 2000);
double dr4b_pos();