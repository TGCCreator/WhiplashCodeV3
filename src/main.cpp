#include "main.h"

#include <algorithm>
#include <cmath>
#include <mutex>

// ===========================================================================
// Globals
// ===========================================================================

// ---- Chassis & odom --------------------------------------------------------
ez::Drive chassis(
    {-10, -11},  // Left
    {9, 12},     // Right

    6,     // IMU
    2.75,  // Wheel Diameter
    450);  // Wheel RPM = cartridge * (motor gear / wheel gear)

// Making the port negative will reverse the sensor
// Port, Wheel Diameter, Distance to Center
ez::tracking_wheel horiz_tracker(8, 2, 2.375);  // Perpendicular
ez::tracking_wheel vert_tracker(7, 2, 3.625);   // Parallel

// ---- dr4b state ------------------------------------------------------------
int dr4bstages = 0;
bool robotmanual = true;
const bool dr4b_sensor_test = false;  // true = PID task never drives the motors

volatile bool dr4bPIDEnabled = false;  // shared between opcontrol and the PID task
bool dr4bManualHeld = false;
bool dr4bSettling = false;
int dr4bSettleTimer = 0;

volatile bool dr4bSensorOK = true;
double dr4bLastPos = 0;

bool clawManualHeld = false;

// Stage heights in DEGREES of the rotation sensor (0 = fully down).
// MEASURE THESE: move the lift by hand to each height and read dr4b_pos().
const double dr4b_STAGES[] = {0, 15, 30, 45, 60, 70};
const int dr4b_NUM_STAGES = sizeof(dr4b_STAGES) / sizeof(dr4b_STAGES[0]);

// Gravity feedforward (kG) vs. lift height. Should be POSITIVE (pushes the lift up).
// Start at 0 and raise values slowly until the lift holds without the P term working.
const double KG_HEIGHTS[] = {0, 15, 30, 45, 60, 70};
const double KG_VALUES[] = {0, 0, 0, 0, 0, 0};
const int KG_N = sizeof(KG_HEIGHTS) / sizeof(KG_HEIGHTS[0]);

// ---- dr4b tuning constants -------------------------------------------------
namespace {
constexpr double BOTTOM_TARGET_MAX = 2;       // target at/below this counts as "park at bottom"
constexpr double BOTTOM_POS_MAX = 4;          // and position at/below this counts as parked
constexpr double DESCENT_THRESHOLD = 3;       // target this far below pos = descending
constexpr double DESCENT_KG_SCALE = 0.3;      // kG multiplier while descending
constexpr double SENSOR_MAX_CENTIDEG = 100000;  // 1000 deg, far past the lift's range

constexpr double SETTLE_VELOCITY_MAX = 500;  // centidegrees/sec (500 = 5 deg/sec)
constexpr int SETTLE_SLOW_LOOPS = 5;         // consecutive slow loops needed to hand off
constexpr int SETTLE_TIMEOUT_MS = 500;       // hand off anyway after this long

constexpr double CLAW_ROTATION_PRESET = 650;  // UP button target

// Guards the lift PID object and enable flag, which both opcontrol and the PID task touch.
pros::Mutex dr4bMutex;

pros::Task* dr4bTask = nullptr;
pros::Task* ezScreenTask = nullptr;
}  // namespace

// ===========================================================================
// dr4b helpers (declared in subsystems.hpp so autons.cpp can use them)
// ===========================================================================

// Piecewise-linear kG lookup
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

// Rotation sensor reports centidegrees -> convert to degrees.
// A failed read returns PROS_ERR (~2.1 billion); the PID would treat that as a gigantic
// error and run the lift at full speed. So bad readings are rejected and the last good
// one is returned instead.
double dr4b_pos() {
  int32_t raw = dr4bRotation.get_position();
  if (raw == PROS_ERR || std::abs(raw) > SENSOR_MAX_CENTIDEG) {
    dr4bSensorOK = false;
    return dr4bLastPos;
  }
  dr4bSensorOK = true;
  dr4bLastPos = raw / 100.0;
  return dr4bLastPos;
}

// Set a new lift target (degrees) and turn the PID on
void dr4b_set(double target) {
  std::lock_guard<pros::Mutex> lock(dr4bMutex);
  if (!dr4bPIDEnabled) {
    // PID was off: flush stale error/integral by running one zero-error compute
    double p = dr4b_pos();
    dr4bPID.target_set(p);
    dr4bPID.compute(p);  // result is thrown away
  }
  dr4bPID.target_set(target);
  dr4bPIDEnabled = true;
}

// Turn the PID off (optionally driving the motor directly) without racing the PID task
static void dr4b_pid_off_and_move(int speed) {
  std::lock_guard<pros::Mutex> lock(dr4bMutex);
  dr4bPIDEnabled = false;
  dr4b.move(speed);
}

static void dr4b_pid_off_and_brake() {
  std::lock_guard<pros::Mutex> lock(dr4bMutex);
  dr4bPIDEnabled = false;
  dr4b.brake();
}

// Block until the lift is within `tolerance` degrees of target, or the timeout hits
void dr4b_wait(double tolerance, int timeout_ms) {
  int waited = 0;
  while (fabs(dr4bPID.target_get() - dr4b_pos()) > tolerance && waited < timeout_ms) {
    pros::delay(ez::util::DELAY_TIME);
    waited += ez::util::DELAY_TIME;
  }
}

// One PID iteration. Caller must hold dr4bMutex.
static void dr4b_run_pid() {
  double pos = dr4b_pos();

  if (!dr4bSensorOK) {
    // Lost the sensor: don't guess, just hold
    dr4b.brake();
    dr4bPIDEnabled = false;
    return;
  }

  if (dr4bPID.target_get() <= BOTTOM_TARGET_MAX && pos <= BOTTOM_POS_MAX) {
    // Parked at the bottom: stop pushing into the hard stop
    dr4b.brake();
    return;
  }

  double kg = dr4b_kg(pos);

  // Gravity helps on the way down, so don't fight the descent with full kG
  if (dr4bPID.target_get() < pos - DESCENT_THRESHOLD) kg *= DESCENT_KG_SCALE;

  double out = std::clamp(dr4bPID.compute(pos) + kg, -127.0, 127.0);
  dr4b.move(-out);  // negative = up on this robot
}

// Background task: runs the PID whenever it is enabled
void dr4bPIDTask() {
  while (true) {
    if (dr4bPIDEnabled && !dr4b_sensor_test) {
      std::lock_guard<pros::Mutex> lock(dr4bMutex);
      if (dr4bPIDEnabled) dr4b_run_pid();  // re-check: opcontrol may have disabled it while we waited
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

// Index of the stage closest to `pos`
static int dr4b_nearest_stage(double pos) {
  int best = 0;
  for (int i = 1; i < dr4b_NUM_STAGES; i++) {
    if (fabs(pos - dr4b_STAGES[i]) < fabs(pos - dr4b_STAGES[best])) best = i;
  }
  return best;
}

// ===========================================================================
// Brain screen / odom debug
// ===========================================================================

// Simplifies printing tracker values to the brain screen
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  ez::screen_print(tracker_value + tracker_width, line);
}

void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled() && ez::as::page_blank_is_on(0)) {
        ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                             "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                             "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                         1);  // Don't override the top Page line

        screen_print_tracker(chassis.odom_tracker_left, "l", 4);
        screen_print_tracker(chassis.odom_tracker_right, "r", 5);
        screen_print_tracker(chassis.odom_tracker_back, "b", 6);
        screen_print_tracker(chassis.odom_tracker_front, "f", 7);
      }
    } else if (ez::as::page_blank_amount() > 0) {
      // Remove all blank pages when connected to a comp switch
      ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

// ===========================================================================
// Competition lifecycle
// ===========================================================================

void initialize() {
  ez::ez_template_print();
  pros::delay(500);  // Let legacy ports configure

  // Odom trackers
  chassis.odom_tracker_back_set(&horiz_tracker);
  chassis.odom_tracker_left_set(&vert_tracker);

  // Chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_drive_activebrake_set(0.0);   // Active brake kP. ~2 recommended; 0 disables.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Comment out if you have an SD card

  // Drive constants from autons.cpp
  default_constants();

  // ---- dr4b + claw setup ----
  dr4b.set_brake_mode(MOTOR_BRAKE_HOLD);
  ClawRotation.set_brake_mode(MOTOR_BRAKE_HOLD);

  // The lift must be fully down on its hard stop at startup so 0 = bottom.
  // (Re-zero any time in driver control: rest the lift on the stop and press B.)
  dr4bRotation.reset_position();

  // Autonomous selector
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

  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

  // Start background tasks only after everything above exists
  dr4bTask = new pros::Task(dr4bPIDTask);
  ezScreenTask = new pros::Task(ez_screen_task);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();
}

// ===========================================================================
// opcontrol helpers
// ===========================================================================

// Off-field only: X+A runs the selected auton, and the PID tuner iterates
void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    if (master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_A)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }
    chassis.pid_tuner_iterate();
  } else if (chassis.pid_tuner_enabled()) {
    chassis.pid_tuner_disable();
  }
}

// B: re-zero the lift (rest it fully down on its stop first)
static void handle_lift_rezero() {
  if (!master.get_digital_new_press(DIGITAL_B)) return;

  dr4bRotation.reset_position();
  dr4bLastPos = 0;
  dr4bstages = 0;
  dr4bSettling = false;
  dr4bManualHeld = false;
  dr4bSensorOK = true;
  dr4b_set(0);
  master.rumble(".");
}

// DOWN: toggle manual / staged mode, holding the lift where it is
static void handle_lift_mode_toggle() {
  if (!master.get_digital_new_press(DIGITAL_DOWN)) return;

  robotmanual = !robotmanual;
  dr4bManualHeld = false;
  dr4bSettling = false;

  double pos = dr4b_pos();
  dr4b_set(pos);
  dr4bstages = dr4b_nearest_stage(pos);  // re-sync stage counter
}

// Manual: L1 = up, L2 = down. On release, brake, wait for the lift to stop, then hold.
static void handle_lift_manual() {
  static int slowLoops = 0;

  if (master.get_digital(DIGITAL_L1) || master.get_digital(DIGITAL_L2)) {
    dr4b_pid_off_and_move(master.get_digital(DIGITAL_L1) ? -127 : 127);
    dr4bSettling = false;
    dr4bManualHeld = true;
    return;
  }

  if (dr4bManualHeld) {
    // Just let go
    dr4bManualHeld = false;
    dr4b_pid_off_and_brake();
    dr4bSettling = true;
    dr4bSettleTimer = 0;
    slowLoops = 0;
  }

  if (dr4bSettling) {
    dr4bSettleTimer += ez::util::DELAY_TIME;
    slowLoops = (fabs(dr4bRotation.get_velocity()) < SETTLE_VELOCITY_MAX) ? slowLoops + 1 : 0;

    if (slowLoops >= SETTLE_SLOW_LOOPS || dr4bSettleTimer >= SETTLE_TIMEOUT_MS) {
      dr4bSettling = false;
      slowLoops = 0;
      double p = dr4b_pos();
      if (dr4bSensorOK) dr4b_set(p);
    }
  }
}

// Staged: RIGHT = next stage up, LEFT = next stage down
static void handle_lift_staged() {
  if (master.get_digital_new_press(DIGITAL_RIGHT) && dr4bstages < dr4b_NUM_STAGES - 1) {
    dr4b_set(dr4b_STAGES[++dr4bstages]);
  }
  if (master.get_digital_new_press(DIGITAL_LEFT) && dr4bstages > 0) {
    dr4b_set(dr4b_STAGES[--dr4bstages]);
  }
}

// Y toggles the claw; R1/R2 rotate it; UP jumps to the preset
static void handle_claw() {
  Claw.button_toggle(master.get_digital(DIGITAL_Y));

  // Read once so a press made while R1/R2 is held isn't missed
  bool upPressed = master.get_digital_new_press(DIGITAL_UP);

  if (master.get_digital(DIGITAL_R1)) {
    ClawRotation.move(127);
    clawManualHeld = true;
  } else if (master.get_digital(DIGITAL_R2)) {
    ClawRotation.move(-127);
    clawManualHeld = true;
  } else if (upPressed) {
    ClawRotation.move_absolute(CLAW_ROTATION_PRESET, 127);
    clawManualHeld = false;
  } else if (clawManualHeld) {
    // Stop only right when R1/R2 is released, so move_absolute isn't cancelled
    ClawRotation.move(0);
    clawManualHeld = false;
  }
}

// Controller line (about every 200 ms, off-field only): p = position, t = target, ERR = sensor failed
static void print_lift_debug(int& counter) {
  if (pros::competition::is_connected() || ++counter % 20 != 0) return;
  master.print(0, 0, "p%.1f t%.1f %s    ", dr4b_pos(), dr4bPID.target_get(), dr4bSensorOK ? "" : "ERR");
}

// ===========================================================================
// opcontrol
// ===========================================================================

void opcontrol() {
  int printCounter = 0;
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    ez_template_extras();
    chassis.opcontrol_arcade_standard(ez::SPLIT);  // Other options: opcontrol_tank(), SINGLE (halo), arcade_flipped()

    handle_lift_rezero();
    handle_lift_mode_toggle();
    if (robotmanual) handle_lift_manual();
    else handle_lift_staged();

    handle_claw();
    print_lift_debug(printCounter);

    pros::delay(ez::util::DELAY_TIME);  // Used for timer calculations! Keep this ez::util::DELAY_TIME
  }
}