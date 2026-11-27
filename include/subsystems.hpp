#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

// ---------------------------------------------------------------------------
// Chassis (defined in main.cpp)
// ---------------------------------------------------------------------------
extern ez::Drive chassis;

// ---------------------------------------------------------------------------
// Claw
// ---------------------------------------------------------------------------
inline pros::Motor ClawRotation(21);
inline ez::Piston Claw('A');

// ---------------------------------------------------------------------------
// dr4b lift
// ---------------------------------------------------------------------------
// Sanity checks on the robot:
//  - Hold L1 (up): the "p" value on the controller must INCREASE. If it decreases,
//    reverse the sensor with dr4bRotation(-5).
//  - Both motors must raise the lift in the same direction. If one fights the other,
//    flip the sign of that port.
inline pros::MotorGroup dr4b({-1, 13});
inline pros::Rotation dr4bRotation(5);


inline ez::PID dr4bPID{4, 0, 0, 5, "dr4b"};

void dr4b_set(double target);  // set lift target (degrees) and enable the PID
void dr4b_wait(double tolerance = 3, int timeout_ms = 2000);  // block until at target or timeout
double dr4b_pos();  // lift position in degrees (last good value if the sensor read fails)