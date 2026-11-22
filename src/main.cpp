#include "main.h"

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
int dr4bstages = 0;
bool robotmanual = true;

// DR4B PID state
bool dr4bPIDEnabled = false;
bool dr4bManualHeld = false;
bool clawManualHeld = false;

// Stage heights in DEGREES of the rotation sensor (0 = fully down).
// MEASURE THESE: move the lift by hand to each height and read dr4b_pos().
const double DR4B_STAGES[] = {0, 60, 110, 150};
const int DR4B_NUM_STAGES = sizeof(DR4B_STAGES) / sizeof(DR4B_STAGES[0]);

ez::Drive chassis(
    {-10, -11},  // Left
    {9, 12},     // Right

    6,     // IMU
    2.75,  // Wheel Diameter
    450);  // Wheel RPM = cartridge * (motor gear / wheel gear)

// Making 8 and 9 negative will reverse the sensor
// Port, Wheel Diameter, Distance to Center
ez::tracking_wheel horiz_tracker(8, 2, 2.375);  // Perpendicular
ez::tracking_wheel vert_tracker(7, 2, 3.625);   // Parallel

// ---------------------------------------------------------------------------
// DR4B helper functions (declared in subsystems.hpp so autons.cpp can use them)
// ---------------------------------------------------------------------------

// Rotation sensor reports centidegrees -> convert to degrees
double dr4b_pos() {
  return dr4bRotation.get_position() / 100.0;
}

// Set a new lift target (in degrees) and turn the PID on
void dr4b_set(double target) {
  dr4bPID.target_set(target);
  dr4bPIDEnabled = true;
}

// Block until the lift is within `tolerance` degrees of target, or the timeout hits
void dr4b_wait(double tolerance, int timeout_ms) {
  int waited = 0;
  while (fabs(dr4bPID.target_get() - dr4b_pos()) > tolerance && waited < timeout_ms) {
    pros::delay(ez::util::DELAY_TIME);
    waited += ez::util::DELAY_TIME;
  }
}

// Background task: runs the PID whenever it is enabled
void dr4bPIDTask() {
  pros::delay(2000);  // let everything finish initializing
  while (true) {
    if (dr4bPIDEnabled) {
      double out = dr4bPID.compute(dr4b_pos());

      // Optional gravity feedforward: uncomment and tune if the lift is slow going up
      // if (dr4bPID.target_get() > dr4b_pos()) out += 8;

      out = std::clamp(out, -127.0, 127.0);
      dr4b.move(out);
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task dr4bTask(dr4bPIDTask);

// ---------------------------------------------------------------------------
// initialize
// ---------------------------------------------------------------------------
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Odom trackers
  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // ---- DR4B + claw setup ----
  dr4b.set_brake_mode(MOTOR_BRAKE_HOLD);
  ClawRotation.set_brake_mode(MOTOR_BRAKE_HOLD);

  // IMPORTANT: the lift must be fully down when the code starts so 0 = bottom.
  // If positive motor power makes dr4b_pos() go DOWN, uncomment the next line (before the reset).
  // dr4bRotation.set_reversed(true);
  dr4bRotation.reset_position();

  // dr4bPID constants live in subsystems.hpp ({kP, kI, kD, start_I}).
  // To change them here instead: dr4bPID.constants_set(1.0, 0.0, 4.0, 5.0);
  dr4b_set(0);  // hold the bottom position at startup

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      //{"LeftStrip", LeftSideStrip},
      //{"LeftNoStrip", LeftSideWithoutStrip},
      //{"RightStrip", RightSideStrip},
      //{"RightNoStrip", RightSideWithoutStrip},
      {"Skills", Skills},
      //{"SoloAWP", SoloAWP},
      //{"AWP Teammate", TeammateAWP},
      //{"PIDTest", PIDTest},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

pros::Task ezScreenTask(ez_screen_task);

void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Trigger the selected autonomous routine.
    // Changed from Left+Y (Y is your claw toggle, Left is a lift stage button) to X+A.
    if (master.get_digital(DIGITAL_X) and master.get_digital(DIGITAL_A)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

// ---------------------------------------------------------------------------
// opcontrol
// ---------------------------------------------------------------------------
void opcontrol() {
  int printCounter = 0;
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    ez_template_extras();

    // chassis.opcontrol_tank();  // Tank
    chassis.opcontrol_arcade_standard(ez::SPLIT);  // Arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Halo
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped Arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped Halo

    // ---- DR4B: toggle manual / staged mode ----
    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      robotmanual = !robotmanual;
      dr4bManualHeld = false;
      dr4b_set(dr4b_pos());  // hold where we are so nothing jumps
      // Re-sync the stage counter to the closest stage
      double best = fabs(dr4b_pos() - DR4B_STAGES[0]);
      dr4bstages = 0;
      for (int i = 1; i < DR4B_NUM_STAGES; i++) {
        if (fabs(dr4b_pos() - DR4B_STAGES[i]) < best) {
          best = fabs(dr4b_pos() - DR4B_STAGES[i]);
          dr4bstages = i;
        }
      }
    }

    if (robotmanual) {
      // Manual control: PID is off while a button is held, then holds the new height on release
      if (master.get_digital(DIGITAL_L1)) {
        dr4bPIDEnabled = false;
        dr4b.move(-127);
        dr4bManualHeld = true;
      } else if (master.get_digital(DIGITAL_L2)) {
        dr4bPIDEnabled = false;
        dr4b.move(127);
        dr4bManualHeld = true;
      } else if (dr4bManualHeld) {
        dr4bManualHeld = false;
        dr4b_set(dr4b_pos());
      }
    } else {
      // Staged control: Right = next stage up, Left = next stage down
      if (master.get_digital_new_press(DIGITAL_RIGHT) && dr4bstages < DR4B_NUM_STAGES - 1) {
        dr4bstages++;
        dr4b_set(DR4B_STAGES[dr4bstages]);
      }
      if (master.get_digital_new_press(DIGITAL_LEFT) && dr4bstages > 0) {
        dr4bstages--;
        dr4b_set(DR4B_STAGES[dr4bstages]);
      }
    }

    // ---- Claw ----
    Claw.button_toggle(master.get_digital(DIGITAL_Y));

    // ---- Claw rotation ----
    if (master.get_digital(DIGITAL_R1)) {
      ClawRotation.move(127);
      clawManualHeld = true;
    } else if (master.get_digital(DIGITAL_R2)) {
      ClawRotation.move(-127);
      clawManualHeld = true;
    } else if (master.get_digital_new_press(DIGITAL_UP)) {
      ClawRotation.move_absolute(650, 127);
      clawManualHeld = false;
    } else if (clawManualHeld) {
      // Only stop the motor right when R1/R2 is released, so move_absolute isn't cancelled
      ClawRotation.move(0);
      clawManualHeld = false;
    }

    // ---- Controller debug print (every ~200 ms, only off-field) ----
    if (!pros::competition::is_connected() && ++printCounter % 20 == 0) {
      master.print(0, 0, "dr4b %.1f tgt %.1f   ", dr4b_pos(), dr4bPID.target_get());
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
