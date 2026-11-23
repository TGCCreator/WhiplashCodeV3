#include "main.h"

#include <algorithm>
#include <cmath>

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
int dr4bstages = 0;
bool robotmanual = true;
const bool dr4b_sensor_test = false; //testing variable

const double KG_HEIGHTS[] = {0, 15, 30, 45, 60, 70};
const double KG_VALUES[] = {-8, -6, -4, -3, -2, -1};
const int KG_N = sizeof(KG_HEIGHTS) / sizeof(KG_HEIGHTS[0]);

double dr4b_kg(double pos) {
  if (pos <= KG_HEIGHTS[0]) return KG_VALUES[0];
  if (pos >= KG_HEIGHTS[KG_N - 1]) return KG_VALUES[KG_N - 1];
  for (int i = 0; i < KG_N - 1; i++) {
    if (pos <= KG_HEIGHTS[i + 1]) {
      double t = (pos - KG_HEIGHTS[i]) / (KG_HEIGHTS[i + 1] - KG_HEIGHTS[i]);
      return KG_VALUES[i] + t * (KG_VALUES[i + 1] - KG_VALUES[i]);
    }
  }
  return KG_VALUES[KG_N - 1];
}

// dr4b PID state
bool dr4bPIDEnabled = false;
bool dr4bManualHeld = false;
bool dr4bSettling = false;
int dr4bSettleTimer = 0;
bool clawManualHeld = false;

// Sensor health
bool dr4bSensorOK = true;
double dr4bLastPos = 0;

// Stage heights in DEGREES of the rotation sensor (0 = fully down).
// MEASURE THESE: move the lift by hand to each height and read dr4b_pos().
const double dr4b_STAGES[] = {0, 15, 30, 45, 60, 70};
const int dr4b_NUM_STAGES = sizeof(dr4b_STAGES) / sizeof(dr4b_STAGES[0]);

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
// dr4b helper functions (declared in subsystems.hpp so autons.cpp can use them)
// ---------------------------------------------------------------------------

// Rotation sensor reports centidegrees -> convert to degrees.
// If the sensor read FAILS, PROS returns PROS_ERR (about 2.1 billion). Dividing that by 100
// gives ~21474836, which the PID would treat as a gigantic error and drive the lift at full
// speed. So we reject bad readings and remember the last good one.
double dr4b_pos() {
  int32_t raw = dr4bRotation.get_position();
  if (raw == PROS_ERR || std::abs(raw) > 100000) {  // 100000 centidegrees = 1000 degrees, far past the lift's range
    dr4bSensorOK = false;
    return dr4bLastPos;
  }
  dr4bSensorOK = true;
  dr4bLastPos = raw / 100.0;
  return dr4bLastPos;
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
  pros::delay(1000);  // let everything finish initializing
  while (true) {
    if (dr4bPIDEnabled && !dr4b_sensor_test) {
      double pos = dr4b_pos();

      if (!dr4bSensorOK) {
        // Lost the sensor: don't guess, just hold
        dr4b.brake();
        dr4bPIDEnabled = false;
      } else if (dr4bPID.target_get() <= 2 && pos <= 4) {
        // Parked at the bottom: stop pushing into the hard stop
        dr4b.brake();
      } else {
        double kg = dr4b_kg(pos);

        // Gravity helps on the way down, so don't fight the descent with full kG
        if (dr4bPID.target_get() < pos - 3) kg *= 0.3;

        double out = dr4bPID.compute(pos) + kg;
        out = std::clamp(out, -127.0, 127.0);
        dr4b.move(-out);  // negative = up on this robot
      }
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

  // ---- dr4b + claw setup ----
  dr4b.set_brake_mode(MOTOR_BRAKE_HOLD);
  ClawRotation.set_brake_mode(MOTOR_BRAKE_HOLD);

  // IMPORTANT: the lift must be fully down on its hard stop when the code starts so 0 = bottom.
  // (You can also re-zero any time in driver control: rest the lift on the stop and press B.)
  dr4bRotation.reset_position();

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
    //fortnite.Pirate_Token:Ahhhh,almost...almost
    // chassis.opcontrol_tank();  // Tank
    chassis.opcontrol_arcade_standard(ez::SPLIT);  // Arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Halo
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped Arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped Halo

    // ---- dr4b: re-zero (rest the lift fully down on its stop, then press B) ----
    if (master.get_digital_new_press(DIGITAL_B)) {
      dr4bRotation.reset_position();
      dr4bLastPos = 0;
      dr4bstages = 0;
      dr4bSettling = false;
      dr4bManualHeld = false;
      dr4bSensorOK = true;
      dr4b_set(0);
      master.rumble(".");
    }

    // ---- dr4b: toggle manual / staged mode ----
    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      robotmanual = !robotmanual;
      dr4bManualHeld = false;
      dr4bSettling = false;
      dr4b_set(dr4b_pos());  // hold where we are so nothing jumps
      // Re-sync the stage counter to the closest stage
      double best = fabs(dr4b_pos() - dr4b_STAGES[0]);
      dr4bstages = 0;
      for (int i = 1; i < dr4b_NUM_STAGES; i++) {
        if (fabs(dr4b_pos() - dr4b_STAGES[i]) < best) {
          best = fabs(dr4b_pos() - dr4b_STAGES[i]);
          dr4bstages = i;
        }
      }
    }

    if (robotmanual) {
      if (master.get_digital(DIGITAL_L1)) {
        dr4bPIDEnabled = false;
        dr4bSettling = false;
        dr4b.move(-127);
        dr4bManualHeld = true;
      } else if (master.get_digital(DIGITAL_L2)) {
        dr4bPIDEnabled = false;
        dr4bSettling = false;
        dr4b.move(127);
        dr4bManualHeld = true;
      } else if (dr4bManualHeld) {
        // Just let go: brake and wait for the lift to stop before the PID takes over
        dr4bManualHeld = false;
        dr4bPIDEnabled = false;
        dr4b.brake();
        dr4bSettling = true;
        dr4bSettleTimer = 0;
      }

      if (dr4bSettling) {
        dr4bSettleTimer += ez::util::DELAY_TIME;
        // velocity is in centidegrees/sec, so 500 = 5 deg/sec
        if (fabs(dr4bRotation.get_velocity()) < 500 || dr4bSettleTimer >= 300) {
        dr4bSettling = false;
        double p = dr4b_pos();
        if (dr4bSensorOK) dr4b_set(p);
  }
}
    } else {
      // Staged control: Right = next stage up, Left = next stage down
      if (master.get_digital_new_press(DIGITAL_RIGHT) && dr4bstages < dr4b_NUM_STAGES - 1) {
        dr4bstages++;
        dr4b_set(dr4b_STAGES[dr4bstages]);
      }
      if (master.get_digital_new_press(DIGITAL_LEFT) && dr4bstages > 0) {
        dr4bstages--;
        dr4b_set(dr4b_STAGES[dr4bstages]);
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
    // p = lift position, t = target, ERR = the rotation sensor read failed.
    // Kept under 15 characters because the controller screen truncates longer lines.
    if (!pros::competition::is_connected() && ++printCounter % 20 == 0) {
      master.print(0, 0, "p%.1f t%.1f %s    ", dr4b_pos(), dr4bPID.target_get(), dr4bSensorOK ? "" : "ERR");
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}