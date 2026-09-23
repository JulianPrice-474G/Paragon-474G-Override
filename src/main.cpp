
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////
#include "main.h"
#include "ui_engine.hpp"

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

// ── R1 / R2 group - four motors, three one way and the fourth the other ─────
// TODO: set your real ports.  R_MOTOR_D is the odd one out - it always runs
// opposite to the other three.
constexpr int8_t FIN_1_PORT = 1;
constexpr int8_t DROPDOWN_PORT = 4;   // drop-down intake - cut by the piston interlock
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

/////
// CASCADE HEIGHTS - CHANGE THESE
/////
// Rotation sensor readings - the "p" value on the controller's middle row.
// Drive the cascade where you want it, read p off the controller, put the
// number here.  The macro and the L1 travel limit both use these.
double CASCADE_LOW     = 190;   // bottom / travel
double CASCADE_COLLECT = 290;   // intake height, where the macro parks
double CASCADE_FLIP    = 320;   // above collect, where the flip piston fires
double CASCADE_OUT     = 400;   // phase 2 raises to here on the second click

// How long after the cascade STARTS its second-click rise the flip piston
// extends.  0 fires it the instant the cascade begins moving; raise it to let
// the cascade get further up first.  If it is longer than the move takes, the
// piston fires as the move finishes.
int CASCADE_FLIP_DELAY_MS = 300;

// First click: how long to wait at flip height AFTER the flip piston releases,
// before the cascade starts down to collect.  Gives the piston time to finish
// moving while the cascade is still still.
int CASCADE_FLIP_RELEASE_MS = 500;
double CASCADE_MAX     = 1000;  // L1 stops raising here

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
constexpr char HIGH_INTAKE_PORT   = 'F';
constexpr char MIDDLE_INTAKE_PORT = 'E';
constexpr char CLAW_PORT          = 'B';  // toggled by DOWN
constexpr char C_FLIP_PORT        = 'A';  // toggled by LEFT

// Single-acting solenoids.  true = extended, false = retracted.
// If your pistons turn out to behave backwards, swap these two values - that is
// the only place the sense of "extended" is defined.
constexpr bool PISTON_EXTENDED  = true;
constexpr bool PISTON_RETRACTED = false;

// Both start EXTENDED.  The second constructor argument is the power-on state,
// so they are already up before opcontrol runs.
pros::adi::DigitalOut high_intake(HIGH_INTAKE_PORT,   PISTON_EXTENDED);
pros::adi::DigitalOut middle_intake(MIDDLE_INTAKE_PORT, PISTON_EXTENDED);

// Claw and C-flip also start EXTENDED.  The second constructor argument is the
// power-on state, so they are out before the match starts.
pros::adi::DigitalOut claw(CLAW_PORT,     PISTON_RETRACTED);
pros::adi::DigitalOut c_flip(C_FLIP_PORT, PISTON_RETRACTED);

// Software mirror of what each solenoid was last told to do.  A DigitalOut
// cannot be read back, so this is the only record of piston state - and the
// dropdown interlock below depends on it.
bool high_intake_extended   = true;
bool middle_intake_extended = true;
bool claw_extended          = false;
bool c_flip_extended        = false;

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
// piston state - the dropdown interlock and the DOWN/LEFT toggles both read
// them, and they go wrong silently if a set_value() is done without one.
void claw_set(bool on)          { claw_extended          = on; claw.set_value(on); }
void flip_set(bool on)          { c_flip_extended        = on; c_flip.set_value(on); }
void high_intake_set(bool on)   { high_intake_extended   = on; high_intake.set_value(on); }
void middle_intake_set(bool on) { middle_intake_extended = on; middle_intake.set_value(on); }

// Whole intake group in one call: -127 to 127, positive runs it the same way
// the R1 button does.  roller = false holds the upper roller at zero, which is
// what driver control does outside the collect height; everything else leaves
// it true so all FOUR motors turn.
void intake_set(int power, bool roller) {
  // Dropdown interlock: port 4 stays stopped while BOTH intake pistons are
  // extended, and runs in every other position.  Same rule as R1/R2.
  bool dropdown_enabled = !(high_intake_extended && middle_intake_extended);

  fin_1.move(-power);
  fin_2.move(power);                                  // mounted opposite
  dropdown.move(dropdown_enabled ? -power : 0);
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
  const uint32_t start = pros::millis();

  // Shortest way round, so 350 -> 10 turns 20 degrees rather than 340.
  auto error_to_target = [&]() {
    double e = target_deg - chassis.drive_imu_get();
    while (e >  180) e -= 360;
    while (e < -180) e += 360;
    return e;
  };

  double first = error_to_target();
  if (fabs(first) < 1) { chassis.drive_set(0, 0); return true; }

  while (pros::millis() - start < (uint32_t)timeout_ms) {
    double e = error_to_target();

    // Stop on arrival, or the moment we cross the target - with fixed speeds
    // there is nothing to slow us down, so the crossing is what catches it.
    if (fabs(e) <= ARC_TOL_DEG || (e > 0) != (first > 0)) {
      chassis.drive_set(0, 0);
      return true;
    }

    chassis.drive_set(left_speed, right_speed);
    pros::delay(ez::util::DELAY_TIME);
  }

  chassis.drive_set(0, 0);
  return false;
}

// Single intake motors, for when you want one on its own.  Same sign
// convention as intake_set: positive runs it the way R1 does.
void upper_roller_set(int power) { upper_roller.move(-power); }

// The dropdown keeps its interlock even when driven on its own - it must not
// run while both intake pistons are extended.
void dropdown_set(int power) {
  bool enabled = !(high_intake_extended && middle_intake_extended);
  dropdown.move(enabled ? -power : 0);
}

// Just the two fins.  fin_2 is mounted opposite, so it is always commanded the
// other way round - positive runs them the same way R1 does.  The dropdown and
// the upper roller are left alone.
void fins_set(int power) {
  fin_1.move(-power);
  fin_2.move(power);
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

// Cascade pair: -127 to 127.  The two motors always run opposite each other.
void cascade_set(int power) {
  l_motor_a.move(power);
  l_motor_b.move(-power);
}

// De-energise every solenoid, so the cylinders vent and the robot is not left
// holding pressure.  Called from disabled().
void release_all_pistons() {
  high_intake_set(false);
  middle_intake_set(false);
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

      // B and Y are TOGGLES, not holds - each press flips state and it stays.
      //  - B toggles high_intake on its own
      //  - Y toggles BOTH together, matching what holding Y used to do
      // Y decides from high_intake's state so the pair cannot drift apart: if B
      // has left them disagreeing, the first Y press lines them both up.
      if (master.get_digital_new_press(DIGITAL_B)) {
        high_intake_extended = !high_intake_extended;
        high_intake.set_value(high_intake_extended);
      }
      if (master.get_digital_new_press(DIGITAL_Y)) {
        bool want = !high_intake_extended;
        high_intake_extended   = want;
        middle_intake_extended = want;
        high_intake.set_value(want);
        middle_intake.set_value(want);
      }

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
      //   port 4  (dropdown) dropdown - runs unless BOTH intake pistons are out
      //   port 19 (upper_roller) upper roller - runs ONLY at the collect height
      //
      // Dropdown interlock, from the two intake piston toggles (B and Y):
      //        both extended   (high state)   -> stopped
      //        high retracted  (middle state) -> runs with the group
      //        both retracted  (low state)    -> runs with the group
      bool dropdown_enabled = !(high_intake_extended && middle_intake_extended);

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

