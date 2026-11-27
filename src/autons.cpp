#include "main.h"
#include "EZ-Template/util.hpp"

// These are out of 127
const int DRIVE_SPEED = 90;
const int TURN_SPEED = 110;
const int SWING_SPEED = 127;

// ===========================================================================
// Constants
// ===========================================================================
void default_constants() {
  // P, I, D, and Start I
  // Remember to change brainscreen.cpp for these values
  chassis.pid_drive_constants_set(7, 0.0, 0.0);            // Fwd/rev, used for odom and non-odom motions
  chassis.pid_heading_constants_set(6.5, 0.0, 0.0);        // Holds the robot straight going forward without odom
  chassis.pid_turn_constants_set(1.54, 0.0, 0.0);          // Turn in place
  chassis.pid_swing_constants_set(1.0, 0.0, 12.0);         // Swing
  chassis.pid_odom_angular_constants_set(0.1, 0.0, 0.0);   // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(1, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // How much turns are prioritized over driving in odom motions. 1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // How far ahead in the path the robot looks
  chassis.odom_boomerang_distance_set(16_in);  // Max distance from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // How aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Default turning behavior: shortest path

  // Motor brake modes (dr4b and ClawRotation) are set in initialize() in main.cpp
}

// ===========================================================================
// Odom tuning helper
// ===========================================================================

// Spins the robot and measures each tracker's distance to center.
void measure_offsets() {
  int iterations = 10;
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();

  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Alternate the turn target every run

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// ===========================================================================
// Autons (empty stubs: fill in, then add to the selector in initialize())
// ===========================================================================
void LeftSideStrip() {}
void LeftSideWithoutStrip() {}
void RightSideStrip() {}
void RightSideWithoutStrip() {}
void AWPAuton() {}
void TeammateAWP() {}

void PIDTest() {
  chassis.pid_turn_set(90_deg, 60);
  chassis.pid_wait();
}

// ---------------------------------------------------------------------------
// Skills
// ---------------------------------------------------------------------------
// Drive and turn timing now uses pid_wait() instead of fixed delays, so it no longer
// depends on battery voltage or field friction. Delays remain only where something
// physical needs time (the claw closing). The lift uses dr4b_set()/dr4b_wait() so it
// shares one position source (the rotation sensor) with the lift PID task.
void Skills() {
  ClawRotation.tare_position();
  dr4b_set(0);  // make sure the lift PID is holding the bottom

  // Drive to the first goal
  chassis.pid_drive_set(16_in, 127);
  chassis.pid_wait();

  ClawRotation.move_absolute(740, 127);  // non-blocking: rotates while we turn
  chassis.pid_turn_set(88_deg, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(9.5_in, 127);
  chassis.pid_wait();

  Claw.set(true);
  pros::delay(600);  // let the claw close

  // Small turn wiggle to seat the goal (kept from the original; remove if unneeded)
  for (double angle : {86.0, 88.0, 90.0, 88.0}) {
    chassis.pid_turn_set(angle, 127);
    chassis.pid_wait();
  }

  chassis.pid_drive_set(-10_in, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(176_deg, 127);
  chassis.pid_wait();

  dr4b_set(20);
  dr4b_wait(2, 800);

  chassis.pid_drive_set(17_in, 127);
  chassis.pid_wait();

  dr4b_set(15);
  dr4b_wait(2, 700);

  chassis.pid_drive_set(-15_in, 127);
  chassis.pid_wait();

  dr4b_set(0);
  dr4b_wait(2, 800);

  chassis.pid_turn_set(43.5_deg, 127);
  chassis.pid_wait();

  chassis.pid_drive_set(18_in, 80);
  chassis.pid_wait();

  Claw.set(false);

  // ---- Planned (not yet converted) ----
  // chassis.pid_turn_set(-45, 127);
  // (scoring code, drive forward 2 inches)
  // chassis.pid_drive_set(-2, 127);
  // chassis.pid_turn_set(-82, 127);
  // chassis.pid_drive_set(21, 127);
  // chassis.pid_turn_set(-47, 127);
  // chassis.pid_drive_set(2, 127);
  // (clamp code)
  // chassis.pid_turn_set(50, 127);
  // (scoring code)
  // chassis.pid_turn_set(0, 127);
  // chassis.pid_drive_set(18, 127);
  // chassis.pid_turn_set(55, 127);
  // chassis.pid_drive_set(2, 127);
  // (clamp code)
  // chassis.pid_turn_set(130, 127);
  // (scoring code)
}

// ---------------------------------------------------------------------------
// CAIMAN alpha test
// ---------------------------------------------------------------------------
// Assumed intent: after starting at (24, 24), if the robot is still within `radius`
// of its start point at the 8th tick, drive to (10, 10) facing 90 degrees.
void CAIMANAlphaTest() {
  const double start_x = 24.0, start_y = 24.0;
  const double radius = 6.0;  // inches

  chassis.odom_xyt_set(24_in, 24_in, 0_deg);

  for (int tick = 1; tick <= 10; tick++) {
    pros::delay(900);

    // Re-check position every tick (the old version checked once, before the loop)
    double dx = chassis.odom_x_get() - start_x;
    double dy = chassis.odom_y_get() - start_y;
    bool closeEnough = sqrt(dx * dx + dy * dy) <= radius;  // compare distance to radius, not radius squared

    if (tick == 8 && closeEnough) {
      chassis.pid_odom_set({{10_in, 10_in, 90_deg}, fwd, DRIVE_SPEED});
      chassis.pid_wait();
    }
  }
}
