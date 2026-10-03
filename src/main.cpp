
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////
#include "main.h"
#include "ui_engine.hpp"

#include <algorithm>
#include <cmath>

// Forward declarations — defined in src/user_screen.cpp
void build_screens();
int  get_selected_auton();
void handle_ctrl_input();
/////
// MOTOR PORTS - CHANGE THESE
// - put a minus in front of a port to reverse that motor (ex. -11)
/////
// ── L1 / L2 pair - two motors, always spinning opposite each other ──────────
// TODO: set your real ports
constexpr int8_t L_MOTOR_A_PORT = 13;
constexpr int8_t L_MOTOR_B_PORT = 6;

// ── R1 / R2 group - four  motors, three one way and the fourth the other ─────
// TODO: set your real ports.  R_MOTOR_D is the odd one out - it always runs
// opposite to the other three.
constexpr int8_t FIN_1_PORT = 1;
constexpr int8_t DROPDOWN_PORT = 4;   // drop-down intake
constexpr int8_t UPPER_ROLLER_PORT = 19;
constexpr int8_t FIN_2_PORT = 11;  // the one that spins opposite the other two


/////
// MOTOR SPEEDS - CHANGE THESE
// - range is 0 to 127, where 127 is full power
/////
constexpr int L_SPEED    = 127;      // L1 - full power
constexpr int L2_SPEED   = L_SPEED * 50 / 100;  // L2 - 50% of L_SPEED (= 63)
constexpr int R_SPEED = 127;         // R1 / R2 group
constexpr int DRIVE_SPEED = 127;     // caps how much power the joysticks can ask for

// How close to the target heading drive_arc() calls it arrived.
constexpr double ARC_TOL_DEG = 2;

// Fin sync - keeps the two fins at the same relative position while they spin.
// The fin that gets ahead is slowed by KP power per degree, up to MAX.
constexpr double FIN_SYNC_KP  = 1.0;
constexpr int    FIN_SYNC_MAX = 40;

/////
// CASCADE HEIGHTS - CHANGE THESE
/////
// Rotation sensor readings - the "p" value on the controller's middle row.
// Drive the cascade where you want it, read p off the controller, put the
// number here.  The macro and the L1 travel limit both use these.
double CASCADE_LOW     = 190;   // bottom / travel
double CASCADE_COLLECT = 300;   // where the macro parks, waiting for press 2
double CASCADE_FLIP    = 360;   // press 1 rises to here first
double CASCADE_OUT     = 430;   // press 2 rises to here
int CASCADE_DROP_DELAY_MS = 350;  // press 1: ms after claw+flip drop before lowering to collect
double CASCADE_MAX     = 1000;  // L1 won't raise past this

// ---- Press 2 ----
// How far into the rise to CASCADE_OUT the flip piston fires.  0 fires it as
// the cascade starts moving; longer than the rise and it fires on arrival.
int CASCADE_FLIP_DELAY_MS = 300;
// How long after the flip fires before the cascade may move again.  Counted
// from the flip, so time spent still rising is already used up.
int CASCADE_AFTER_FLIP_MS = 200;

// Power for downward macro moves.  At the same power the cascade was slower
// going down than up, so down moves get their own number.  80% of full.
int CASCADE_DOWN_SPEED = 127 * 95 / 100;

/////
// CASCADE HOLD (currently unused - kept for the macro work)
/////
// These belong to a shared PD hold that drove both motors from one position
// reading.  It is not wired up: the cascade currently uses the motors' own
// BRAKE_HOLD (see opcontrol()).  Left here because the macro system will want
// a position controller, and this is most of one.
constexpr double CASCADE_HOLD_KP       = 2.0;
constexpr double CASCADE_HOLD_KD       = 0.0;
constexpr int    CASCADE_HOLD_MAX      = 60;  // power cap
constexpr double CASCADE_HOLD_DEADBAND = 2.0; // degrees of slop before correcting

// L1 / L2 pair - these two ALWAYS spin opposite each other.
// The opposite direction is commanded in code (one gets +speed, the other
// -speed), not baked into a negative port, so the relationship is visible
// where you read it.
pros::Motor l_motor_a(L_MOTOR_A_PORT);
pros::Motor l_motor_b(L_MOTOR_B_PORT);

// R1 / R2 group - A, B and C run together; D always runs opposite to them.
pros::Motor fin_1(FIN_1_PORT);
pros::Motor dropdown(DROPDOWN_PORT);
pros::Motor upper_roller(UPPER_ROLLER_PORT);
pros::Motor fin_2(FIN_2_PORT); 

/////
// PNEUMATICS - ADI (3-wire) ports, letters A-H
/////
constexpr char INTAKE_PISTON_PORT = 'F';  // 25 mm, UP <-> MIDDLE - toggled by Y
constexpr char CLAW_PORT          = 'B';  // toggled by DOWN
constexpr char C_FLIP_PORT        = 'A';  // toggled by LEFT

// Single-acting solenoids.  true = extended, false = retracted.
// If your pistons turn out to behave backwards, swap these two values - that is
// the only place the sense of "extended" is defined.
constexpr bool PISTON_EXTENDED  = true;
constexpr bool PISTON_RETRACTED = false;

// Starts EXTENDED (intake UP).  The second constructor argument is the
// power-on state, so it is already up before opcontrol runs.
pros::adi::DigitalOut intake_piston(INTAKE_PISTON_PORT, PISTON_EXTENDED);

// Claw starts CLAW_ON (gripping) and the C-flip starts FLIP_ON.  The second
// constructor argument is the power-on state, so they are set before the match
// starts.
pros::adi::DigitalOut claw(CLAW_PORT,     CLAW_ON);
pros::adi::DigitalOut c_flip(C_FLIP_PORT, FLIP_ON);

// Software mirror of what each solenoid was last told to do.  A DigitalOut
// cannot be read back, so this is the only record of piston state.
bool intake_piston_extended = true;
bool claw_extended          = CLAW_ON;
bool c_flip_extended        = FLIP_ON;

// Where the intake is.  One 25 mm piston moves it between the two positions.
//   HIGH   piston extended - the resting position
//   MIDDLE piston retracted
IntakePos intake_pos = INTAKE_HIGH;

/////
// CASCADE POSITION SOURCE
/////
// Single place that answers "where is the cascade", so anything reading it -
// the controller readout now, the macros later - goes through one function.
double cascade_position() {
  // /100 because Rotation::get_position() is in CENTIdegrees while
  // Motor::get_position() is in degrees.  get_position() and not get_angle():
  // get_angle() wraps at 360 and the cascade would appear to teleport.
  double deg = cascade_rot.get_position() / 100.0;

  // PROS_ERR when the sensor is missing or unplugged mid-match.  Fall back to
  // the motor encoder rather than handing the hold loop a garbage target -
  // less accurate, but it keeps holding instead of slamming to the power cap.
  if (cascade_rot.get_position() == PROS_ERR) return l_motor_a.get_position();
  return deg;
}

// Which cascade motor currently does the holding.  Only ONE holds - two motors
// in BRAKE_HOLD on the same shaft each run their own position PID against their
// own encoder, latch different targets, and fight each other for ever.  The
// holder carries the whole load, so it is the one that heats up; swapping after
// every trip back to low spreads that between the two.
int cascade_hold_motor = 0;   // 0 = l_motor_a holds, 1 = l_motor_b holds

void cascade_apply_hold_motor() {
  if (cascade_hold_motor == 0) {
    l_motor_a.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    l_motor_b.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  } else {
    l_motor_a.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    l_motor_b.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
}

void cascade_swap_hold_motor() {
  cascade_hold_motor = 1 - cascade_hold_motor;
  cascade_apply_hold_motor();
}

// Brake the holder, and leave the other at zero volts so it free-wheels rather
// than fighting.  Called every tick the cascade is idle.
void cascade_hold() {
  if (cascade_hold_motor == 0) { l_motor_a.brake(); l_motor_b.move(0); }
  else                         { l_motor_b.brake(); l_motor_a.move(0); }
}

/////
// Subsystem helpers - use these instead of driving the devices directly
/////
// Each piston helper sets the solenoid AND its software mirror together.  A
// DigitalOut cannot be read back, so those mirrors are the only record of
// piston state - the DOWN/LEFT toggles and the controller readout read them,
// and they go wrong silently if a set_value() is done without one.
void claw_set(bool on)          { claw_extended          = on; claw.set_value(on); }
void flip_set(bool on)          { c_flip_extended        = on; c_flip.set_value(on); }
void intake_piston_set(bool on) { intake_piston_extended = on; intake_piston.set_value(on); }

// Whole intake group in one call: -127 to 127, positive runs it the same way
// the R1 button does.  roller = false holds the upper roller at zero, which is
// what driver control does outside the collect height; everything else leaves
// it true so all FOUR motors turn.
void intake_set(int power, bool roller) {
  fins_set(power);
  dropdown.move(-power);
  upper_roller.move(roller ? -power : 0);
}

/////
// drive_arc - the simple version of a swing
/////
// You give it the two side speeds and the heading to stop at.  It drives both
// sides at those speeds until the robot is facing target_deg, then stops.
//
//   drive_arc(90, 100, 40);    // curve right to 90 degrees
//   drive_arc(0, 40, 100);     // curve back to 0 the other way
//   drive_arc(90, 80, -80);    // spin on the spot to 90
//
// The heading is ABSOLUTE, like pid_turn_set - 90 means "end up facing 90",
// not "turn 90 more".  Blocks until it arrives, so no pid_wait() afterwards.
//
// This is open-loop: the speeds you give are the speeds it drives at, and the
// only feedback is when to stop.  That makes it predictable, but it will not
// correct itself the way the PID motions do - expect to overshoot slightly at
// high speeds, and lower the speeds rather than fighting it.
//
// Returns false if it ran out of time instead of reaching the heading.
bool drive_arc(double target_deg, int left_speed, int right_speed, int timeout_ms) {
  drive_for_time_stop();   // cancel a push still running - see drive_for_time()
  const uint32_t start = pros::millis();

  // Stop, and tell EZ-Template which way the robot now faces.  A normal
  // pid_drive_set() holds the heading EZ last set as its target, and only EZ's
  // own turns and swings update that.  Without this, the first drive after an
  // arc steers back to the angle from BEFORE the arc.
  //
  // The target is the heading we actually ended at, not target_deg: we stop a
  // degree or two past it, and aiming at target_deg would make the next drive
  // wiggle to correct that.  drive_angle_set() also resets the IMU, but to the
  // value it already reads, so the heading itself does not change.
  auto arc_stop = [&]() {
    chassis.drive_set(0, 0);
    chassis.drive_angle_set(chassis.drive_imu_get());
  };

  // Shortest way round, so 350 -> 10 turns 20 degrees rather than 340.
  auto error_to_target = [&]() {
    double e = target_deg - chassis.drive_imu_get();
    while (e >  180) e -= 360;
    while (e < -180) e += 360;
    return e;
  };

  double first = error_to_target();
  if (fabs(first) < 1) { arc_stop(); return true; }

  while (pros::millis() - start < (uint32_t)timeout_ms) {
    double e = error_to_target();

    // Stop on arrival, or the moment we cross the target - with fixed speeds
    // there is nothing to slow us down, so the crossing is what catches it.
    if (fabs(e) <= ARC_TOL_DEG || (e > 0) != (first > 0)) {
      arc_stop();
      return true;
    }

    chassis.drive_set(left_speed, right_speed);
    pros::delay(ez::util::DELAY_TIME);
  }

  arc_stop();
  return false;
}

// Timed push: drive at a fixed power for a set time, in the background, so
// pistons, intakes and the cascade can run while the robot holds itself
// against a wall or goal.  One long-lived worker, like the intake spins -
// pros::Task has no destructor, so a task per call would leak.
//
// The push CANCELS ITSELF the moment the next drive command starts:
//  - pid_drive_set / turn / swing / odom moves take EZ-Template out of DISABLE
//    mode, and the worker sees that and steps aside.
//  - drive_arc, vision_drive and vision_align call drive_for_time_stop()
//    first, because they drive in DISABLE mode too and the worker cannot tell
//    them apart from the push.
static volatile bool _push_active = false;
static volatile int  _push_until  = 0;

// Write the drive motors directly, NOT through chassis.drive_set().  drive_set()
// forces EZ-Template into DISABLE mode, so if a pid_drive_set() landed between
// the worker's mode check and its write, drive_set() would switch that PID
// motion straight back off.  A direct write can only clobber one tick, and the
// PID task overwrites it on the next.
static void push_motors(int power) {
  for (auto& m : chassis.left_motors)  m.move(power);
  for (auto& m : chassis.right_motors) m.move(power);
}

static void push_task(void*) {
  while (true) {
    if (_push_active) {
      if (chassis.drive_mode_get() != ez::DISABLE) {
        // A PID motion owns the wheels now.  Step aside without touching them.
        _push_active = false;
      } else if ((int)pros::millis() >= _push_until) {
        push_motors(0);
        _push_active = false;
      } else {
        // Keep EZ-Template's heading target on wherever the wall has squared
        // us to, every tick - same reason as drive_arc().  Doing it throughout
        // rather than only at the end means a pid_drive_set() that cuts the
        // push short still drives off in the aligned direction, instead of
        // steering back to the heading from before the push.
        chassis.drive_angle_set(chassis.drive_imu_get());
      }
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

void drive_for_time(int ms, int speed) { drive_for_time(ms, speed, speed); }

// Timed arc: the same push, but each side at its own speed.
void drive_for_time(int ms, int left_speed, int right_speed) {
  static pros::Task worker(push_task, nullptr, "Drive Push");

  if (ms <= 0 || (left_speed == 0 && right_speed == 0)) {
    drive_for_time_stop();
    return;
  }
  // drive_set() once, here, to put EZ-Template in DISABLE mode - its task does
  // not write the motors in that mode, so they hold this power by themselves
  // and the worker only has to watch the clock.
  chassis.drive_set(left_speed, right_speed);
  _push_until  = (int)pros::millis() + ms;
  _push_active = true;
}

void drive_for_time_stop() {
  if (!_push_active) return;
  _push_active = false;
  push_motors(0);
  chassis.drive_angle_set(chassis.drive_imu_get());
}

bool drive_for_time_active() { return _push_active; }

bool drive_for_time_wait(int timeout_ms) {
  const uint32_t start = pros::millis();
  while (_push_active) {
    if ((int)(pros::millis() - start) > timeout_ms) return false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return true;
}

// Single intake motors, for when you want one on its own.  Same sign
// convention as intake_set: positive runs it the way R1 does.
void upper_roller_set(int power) { upper_roller.move(-power); }

void dropdown_set(int power) { dropdown.move(-power); }

// Just the two fins, kept in step.  fin_2 is mounted opposite, so it is always
// commanded the other way round - positive runs them the same way R1 does.  The
// dropdown and the upper roller are left alone.
//
// fins_set() only records the power.  One long-lived task writes both motors
// every tick, slowing whichever fin has got ahead of the positions recorded by
// fins_sync_zero(), so they stay at the same relative position.  Nothing else
// may write the fin motors, or it would fight this task.
static volatile int _fin_power = 0;
static double _fin1_zero = 0;
static double _fin2_zero = 0;

void fins_sync_zero() {
  _fin1_zero = fin_1.get_position();
  _fin2_zero = fin_2.get_position();
}

static void fin_sync_task(void*) {
  while (true) {
    int p = _fin_power;
    if (p == 0) {
      fin_1.move(0);
      fin_2.move(0);
    } else {
      double a = fin_1.get_position();
      double b = fin_2.get_position();
      double corr = 0;
      // A missing motor reads PROS_ERR_F - test for good values, then run
      // without correction rather than on garbage.
      if (std::isfinite(a) && std::isfinite(b)) {
        // Progress in the R1 direction.  fin_1 runs negative for R1.
        double err = -(a - _fin1_zero) - (b - _fin2_zero);   // + = fin_1 ahead
        corr = std::clamp(err * FIN_SYNC_KP, (double)-FIN_SYNC_MAX, (double)FIN_SYNC_MAX);
      }
      // Works in both directions: the one ahead gets less power, the one behind
      // more (capped at 127, so near full speed the leader does the waiting).
      int p1 = std::clamp((int)std::lround(p - corr), -127, 127);
      int p2 = std::clamp((int)std::lround(p + corr), -127, 127);
      fin_1.move(-p1);
      fin_2.move(p2);
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

void fins_set(int power) {
  static pros::Task worker(fin_sync_task, nullptr, "Fin Sync");
  _fin_power = power;
}

// Timed, non-blocking spins.  Each call returns immediately and a background
// task drives the motors, so the next drive or turn starts straight away:
//
//   upper_roller_spin(800, 127);                      // returns at once
//   chassis.pid_drive_set(24_in, DRIVE_SPEED);  // roller still spinning
//   chassis.pid_wait();
//
// ms > 0 runs for that long, ms < 0 runs until stopped, ms == 0 or speed == 0
// stops that group now.  Positive speed runs the motors the way R1 does.
//
// The three groups are INDEPENDENT and can overlap - a roller spin does not
// disturb a fins spin.  intake_spin() is shorthand for setting all three at
// once.  Starting the same group again replaces its previous spin.
struct _SpinCh {
  int  until_ms;   // absolute deadline, or -1 for "until stopped"
  int  speed;
  bool active;
};
static volatile _SpinCh _ch_fins   = {0, 0, false};
static volatile _SpinCh _ch_roller = {0, 0, false};
static volatile _SpinCh _ch_drop   = {0, 0, false};

bool intake_spin_active() {
  return _ch_fins.active || _ch_roller.active || _ch_drop.active;
}

// Returns the power this channel should be driving at now, expiring it if its
// time is up.
static int _ch_power(volatile _SpinCh& c) {
  if (!c.active) return 0;
  if (c.until_ms >= 0 && (int)pros::millis() >= c.until_ms) {
    c.active = false;
    return 0;
  }
  return c.speed;
}

static void intake_spin_task(void*) {
  bool was_driving = false;
  while (true) {
    bool driving = intake_spin_active();

    // Write while any group is running, plus one final pass on the tick
    // everything stops - so the motors are actually zeroed.  After that, stop
    // writing entirely, or this would fight opcontrol every tick.
    if (driving || was_driving) {
      fins_set(_ch_power(_ch_fins));
      upper_roller_set(_ch_power(_ch_roller));
      dropdown_set(_ch_power(_ch_drop));
    }
    was_driving = driving;
    pros::delay(ez::util::DELAY_TIME);
  }
}

static void _ch_start(volatile _SpinCh& c, int ms, int speed) {
  // One worker, created on first use and never destroyed.  pros::Task has no
  // destructor, so spawning one per call would leak a task every time.
  static pros::Task worker(intake_spin_task, nullptr, "Intake Spin");

  if (ms == 0 || speed == 0) { c.active = false; return; }
  c.speed    = speed;
  c.until_ms = (ms < 0) ? -1 : (int)pros::millis() + ms;
  c.active   = true;
}

void fins_spin(int ms, int speed)     { _ch_start(_ch_fins,   ms, speed); }
void upper_roller_spin(int ms, int speed)   { _ch_start(_ch_roller, ms, speed); }
void dropdown_spin(int ms, int speed) { _ch_start(_ch_drop,   ms, speed); }

// All three groups together.
void intake_spin(int ms, int speed) {
  fins_spin(ms, speed);
  upper_roller_spin(ms, speed);
  dropdown_spin(ms, speed);
}
void intake_spin_stop() { intake_spin(0, 0); }

// Block until every running spin has finished.  The spins are non-blocking by
// design - that is the point of them - so this is for when you want one to
// finish before the next line runs:
//
//   upper_roller_spin(800, 127);
//   spin_wait();                 // waits out the 800 ms
//   claw_set(CLAW_ON);           // only now
//
// Returns false if timeout_ms passed with something still running, which only
// happens if a spin was started with ms < 0 (run until stopped).
bool spin_wait(int timeout_ms) {
  const uint32_t start = pros::millis();
  while (intake_spin_active()) {
    if ((int)(pros::millis() - start) > timeout_ms) return false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return true;
}

// Cascade pair: -127 to 127.  The two motors always run opposite each other.
void cascade_set(int power) {
  l_motor_a.move(power);
  l_motor_b.move(-power);
}

// De-energise every solenoid, so the cylinders vent and the robot is not left
// holding pressure.  Called from disabled().
// The intake position, exactly as the Y button drives it.  Driver control and
// autons both call these, so a simulated press in an auton does the same thing
// a real one does.
//   Y  -> MIDDLE, or back to HIGH if already MIDDLE
void intake_pos_set(IntakePos pos) {
  intake_pos = pos;
  intake_piston_set(pos == INTAKE_HIGH);   // extended = HIGH
}
void press_y() { intake_pos_set(intake_pos == INTAKE_MIDDLE ? INTAKE_HIGH : INTAKE_MIDDLE); }

void release_all_pistons() {
  intake_piston_set(false);
  claw_set(false);
  flip_set(false);
}

// Cascade hold state.  cascade_last_err is read by the controller readout in
// user_screen.cpp so you can see the error while tuning KP/KD.
bool   cascade_holding  = false;
double cascade_target   = 0;
double cascade_last_err = 0;

/////
// AI VISION SENSOR - smart port (not ADI)
/////
/////
// DISTANCE SENSOR - smart port
/////
// DISTANCE_PORT is set in include/subsystems.hpp so the readout can see it.
pros::Distance distance_sensor(DISTANCE_PORT);

/////
// CASCADE ROTATION SENSOR - smart port
/////
// Put a minus in front of the port if it counts backwards (raising the cascade
// must make the number go UP, or the hold loop will drive the wrong way).
constexpr int8_t CASCADE_ROT_PORT = 14;
pros::Rotation cascade_rot(CASCADE_ROT_PORT);

constexpr int8_t AI_VISION_PORT = 15;
pros::AIVision ai_cam(AI_VISION_PORT);

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-20, -12,},     // Left Chassis Ports (negative port will reverse it!)
    {10, 3,},  // Right Chassis Ports (negative port will reverse it!)

    5,      // IMU Port
    3.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    360);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();  // required to start LVGL - do not remove, it data aborts

  // Leave LLEMU's 8 objects alone.  lv_obj_clean(lv_scr_act()) here would free
  // them all while LLEMU went on holding pointers to them, and anything that
  // later called pros::lcd::set_text() would write into freed memory and data
  // abort.  lcd_initialize() does not rebuild them either - it returns early
  // because LLEMU still believes it is initialised.
  // Spinner goes on a screen of our own, so LLEMU's is left untouched.
  // remove_style_all() strips width and height along with every other style
  // property, so set them back explicitly - a screen with no size is what made
  // an earlier attempt at this fail to cover PROS's loading bar.
  lv_obj_t* boot = lv_obj_create(nullptr);
  lv_obj_remove_style_all(boot);
  lv_obj_set_size(boot, LV_HOR_RES, LV_VER_RES);
  lv_obj_set_pos(boot, 0, 0);
  lv_obj_set_style_bg_color(boot, lv_color_hex(UI_DARK_BG), 0);
  lv_obj_set_style_bg_opa(boot,   LV_OPA_COVER,             0);
  lv_obj_clear_flag(boot, LV_OBJ_FLAG_SCROLLABLE);
  lv_scr_load(boot);

  lv_obj_t* spinner = lv_spinner_create(boot, 1200, 75);
  lv_obj_set_size(spinner, 100, 100);
  lv_obj_center(spinner);
  lv_obj_set_style_arc_color(spinner, lv_color_hex(UI_GOLD),   LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(spinner, 8,                        LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(spinner, lv_color_hex(0x3A3A3A), LV_PART_MAIN);
  lv_obj_set_style_arc_width(spinner, 8,                        LV_PART_MAIN);
  lv_task_handler();

  chassis.opcontrol_curve_buttons_toggle(false); // reclaim controller buttons for UI use

  // ── Your chassis setup — replace with your own, but do not delete ────────────
  default_constants();
  chassis.initialize();   // spinner stays visible during IMU calibration
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  // ─────────────────────────────────────────────────────────────────────────────

  // Tell the AI Vision sensor what to look for.  Harmless if it is unplugged -
  // the calls just return PROS_ERR and ai_vision_text() shows "--".
  ai_cam.enable_detection_types(pros::AivisionModeType::tags,
                                pros::AivisionModeType::colors,
                                pros::AivisionModeType::objects);
  ai_cam.set_tag_family(pros::AivisionTagFamily::tag_16H5);

  // Put the intake in the HIGH position at power-on - piston extended.
  // Commanded explicitly rather than relying on the DigitalOut constructor's
  // initial state, so the solenoid actually receives it.
  intake_pos_set(INTAKE_HIGH);

  // Fin alignment reference at power-on, for driver practice without an auton.
  // autonomous() takes it again.
  fins_sync_zero();

  EngineInit();
  build_screens();  // sets up brain screen + initial controller display
  CtrlFlush();
}






/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // Vent everything the moment the robot is disabled, so it is not left with
  // pistons held out between matches.
  release_all_pistons();
  macro_press_pending_clear();
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

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  // The fins' positions now are the alignment they are held to while spinning.
  fins_sync_zero();

  // The number in each case must match the auton_idx you gave that
  // ButtonAdd in build_screens().
  switch (get_selected_auton()) {
    case 0: sawp();    break;
    case 1: skills();  break;
    case 2: one_pin(); break;
    case 3: auto_4();  break;
    case 4: auto_5();  break;
    default:                           break;   // nothing selected
  }
}
// NOTE: EZ-Template's stock main.cpp defines screen_print_tracker() and
// ez_screen_task() here, plus the global `pros::Task ezScreenTask(ez_screen_task);`.
// All three have been REMOVED for the brain UI.
//  - They draw to the brain with ez::screen_print(), which is LLEMU.
//  - initialize() loads a screen of its own and hands the display to the UI engine.
//  - The task is a global, so it starts before initialize() even runs.
// Leaving it in means an LLEMU task and the LVGL UI engine both driving the screen.

/**
 * Simplifies printing tracker values to the brain screen
 */

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */


/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // Trigger the selected autonomous routine.
    // Gated on !DriverModeActive(): in driver mode LEFT is released back to your
    // subsystems, so without this a driver using LEFT could fire autonomous by
    // accident.  Outside driver mode LEFT belongs to the UI menu, and the menu
    // navigating while auton starts does not matter.
    // Edge-detect the combo ourselves rather than using get_digital_new_press().
    //
    // Two reasons:
    //  1. get_digital_new_press() is CONSUMED by the first caller each press, and
    //     handle_ctrl_input() already reads LEFT/RIGHT/A/B that way for the menu.
    //     It runs earlier in the loop, so it eats the press and this never fires.
    //  2. Plain get_digital() on both would re-fire forever: autonomous() blocks
    //     the loop, so the instant it returns the combo is still held.
    //
    // get_digital() does not get consumed, so tracking the edge here is safe.
    // HOLD both for AUTON_COMBO_HOLD_MS.  A tap does nothing, so brushing the
    // buttons while driving cannot start a routine.
    //
    // get_digital(), not get_digital_new_press(): a new-press is CONSUMED by the
    // first caller, and handle_ctrl_input() already reads LEFT/RIGHT/A/B that way
    // for the menu.  It runs earlier in the loop, so it would eat the press and
    // this would never fire.
    static const int AUTON_COMBO_HOLD_MS = 1000;
    static int  auton_combo_ms    = 0;
    static bool auton_combo_fired = false;

    bool auton_combo = master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_LEFT);
    bool auton_combo_pressed = false;

    if (auton_combo) {
      auton_combo_ms += ez::util::DELAY_TIME;   // one tick per opcontrol loop
      if (auton_combo_ms >= AUTON_COMBO_HOLD_MS && !auton_combo_fired) {
        auton_combo_fired   = true;             // fires once, not every tick
        auton_combo_pressed = true;
      }
    } else {
      auton_combo_ms    = 0;
      auton_combo_fired = false;
    }

    if (!DriverModeActive() && auton_combo_pressed) {
      master.rumble("-");   // long buzz so you know the combo fired
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

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  macro_press_pending_clear();   // a delayed press left over from the auton

  // Only ONE cascade motor holds - see cascade_apply_hold_motor() above for why.
  // Re-applied here so it survives an auton test, which changes brake modes.
  cascade_apply_hold_motor();
  static bool ctrl_flushed = false;

  while (true) {
    handle_ctrl_input();
    ez_template_extras();        // ← keeps the LEFT+B auton test

    chassis.opcontrol_arcade_standard(ez::SPLIT);

    if (!ctrl_flushed) { ctrl_flushed = true; CtrlFlush(); }

    // ── Add your subsystem controls here ──────────────────────────────────
    // Gated on driver mode: outside it the UI owns LEFT/RIGHT/A/B, so navigating
    // the auton menu must not be able to spin a motor or fire a piston.
    if (DriverModeActive()) {


      // ── Cascade sequence, in two halves ─────────────────────────────────
      //   press 1 -> flip height, then collect, then WAIT (roller armed)
      //   press 2 -> flip height, then back to low
      // Pressing while it is moving cancels instead.  Both halves run in their
      // own task, so the drivetrain keeps responding throughout.
      if (master.get_digital_new_press(DIGITAL_RIGHT)) macro_start();

      // Touching the cascade manually also cancels a move - the driver should
      // not have to find the right button to take back control.
      if (macro_running() &&
          (master.get_digital(DIGITAL_L1) || master.get_digital(DIGITAL_L2)))
        macro_cancel();

        // Intake piston - Y toggles between HIGH and MIDDLE.
        if (master.get_digital_new_press(DIGITAL_Y)) press_y();

      // Claw on DOWN, C-flip on LEFT - both latching toggles.  Safe to read
      // these with new_press: handle_ctrl_input() only consumes LEFT/RIGHT/A/B
      // while driver mode is OFF, and this block is driver-only.
      if (master.get_digital_new_press(DIGITAL_DOWN)) {
        claw_extended = !claw_extended;
        claw.set_value(claw_extended);
      }
      if (master.get_digital_new_press(DIGITAL_LEFT)) {
        c_flip_extended = !c_flip_extended;
        c_flip.set_value(c_flip_extended);
      }

      // Intake - four motors, R2 runs them in, R1 reverses all of them.
      //   port 1  (fin_1) fin      - always runs
      //   port 11 (fin_2) fin      - always runs, mounted opposite the rest
      //   port 4  (dropdown) dropdown - always runs
      //   port 19 (upper_roller) upper roller - runs ONLY at the collect height
      // A runs all four the R2 way, upper roller included, at any height.

      // The upper roller (port 19) only turns while the cascade is AT the
      // collect height.  Anywhere else the fins (ports 1 and 11) and the
      // dropdown still run, but the roller is held at zero.
      bool roller_enabled = cascade_at_collect();

      // Skipped while the macro drives the intake itself - otherwise the else
      // branch below writes zero to these motors every tick and the macro's
      // intake never actually spins.
      if (macro_owns_intake() || intake_spin_active()) {
        // the macro or a timed intake_spin() owns the intake
      } else if (master.get_digital(DIGITAL_R2)) {
        intake_set(-R_SPEED, roller_enabled);
      } else if (master.get_digital(DIGITAL_R1)) {
        intake_set(R_SPEED, roller_enabled);
      } else if (master.get_digital(DIGITAL_A)) {
        intake_set(-R_SPEED, true);   // all four, roller too, any cascade height
      } else {
        intake_set(0);
      }

      // L1 / L2 pair - two motors, always opposite each other
      //  - L1: A forward, B backward, at full L_SPEED
      //  - L2: both flipped from what L1 does, at the slower L2_SPEED
      // L1/L2 are locked out in two cases:
      //  - while the macro is MOVING: it owns these motors, and both writing
      //    every tick would make it lose
      //  - while it is PARKED at collect waiting for the second press: the
      //    cascade must stay exactly where the sequence left it
      // The intake, claw and drivetrain stay under driver control throughout.
      if (macro_running()) {
        // macro owns the cascade - do not touch the motors
      } else if (macro_waiting()) {
        cascade_hold();   // parked: hold position, ignore L1/L2
      } else if (master.get_digital(DIGITAL_L1)) {
        // Stop at the top limit instead of driving into the hard stop.  Hold
        // rather than coast, so it stays put while the button is still held.
        if (cascade_position() >= CASCADE_MAX) {
          cascade_hold();
        } else {
          cascade_set(L_SPEED);
        }
      } else if (master.get_digital(DIGITAL_L2)) {
        cascade_set(-L2_SPEED);
      } else {
        // Whichever motor is currently the holder brakes; the other free-wheels.
        // The macro swaps them after each trip back to low.
        cascade_hold();
      }
    } else {
      // Parked while the UI has the controller.  move(0) rather than skipping,
      // or a motor holds whatever it was last told to do.
      cascade_set(0);
      intake_set(0);
      // Pistons hold their state and are not re-commanded here.
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

