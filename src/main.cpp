
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
constexpr int8_t L_MOTOR_A_PORT = 11;
constexpr int8_t L_MOTOR_B_PORT = 2;

// ── R1 / R2 group - four motors, three one way and the fourth the other ─────
// TODO: set your real ports.  R_MOTOR_D is the odd one out - it always runs
// opposite to the other three.
constexpr int8_t R_MOTOR_A_PORT = -16;
constexpr int8_t R_MOTOR_B_PORT = -1;   // negative = this motor is mounted backwards
constexpr int8_t R_MOTOR_C_PORT = 13;  // negative = this motor is mounted backwards
constexpr int8_t R_MOTOR_D_PORT = 4;


/////
// MOTOR SPEEDS - CHANGE THESE
// - range is 0 to 127, where 127 is full power
/////
constexpr int L_SPEED    = 127;      // L1 - full power
constexpr int L2_SPEED   = L_SPEED * 80 / 100;  // L2 - 80% of L_SPEED (= 101)
constexpr int R_SPEED = 127;         // R1 / R2 group
constexpr int DRIVE_SPEED = 127;     // caps how much power the joysticks can ask for

// L1 / L2 pair - these two ALWAYS spin opposite each other.
// The opposite direction is commanded in code (one gets +speed, the other
// -speed), not baked into a negative port, so the relationship is visible
// where you read it.
pros::Motor l_motor_a(L_MOTOR_A_PORT);
pros::Motor l_motor_b(L_MOTOR_B_PORT);

// R1 / R2 group - A, B and C run together; D always runs opposite to them.
pros::Motor r_motor_a(R_MOTOR_A_PORT);
pros::Motor r_motor_b(R_MOTOR_B_PORT);
pros::Motor r_motor_c(R_MOTOR_C_PORT);
pros::Motor r_motor_d(R_MOTOR_D_PORT);

/////
// PNEUMATICS - ADI (3-wire) ports, letters A-H
/////
constexpr char HIGH_INTAKE_PORT   = 'F';
constexpr char MIDDLE_INTAKE_PORT = 'D';
constexpr char CLAW_PORT          = 'A';  // TODO: set your real ADI port (A-H, D and F are taken)

// Single-acting solenoids.  true = extended, false = retracted.
// If your pistons turn out to behave backwards, swap these two values - that is
// the only place the sense of "extended" is defined.
constexpr bool PISTON_EXTENDED  = true;
constexpr bool PISTON_RETRACTED = false;

// Both start EXTENDED.  The second constructor argument is the power-on state,
// so they are already up before opcontrol runs.
pros::adi::DigitalOut high_intake(HIGH_INTAKE_PORT,   PISTON_EXTENDED);
pros::adi::DigitalOut middle_intake(MIDDLE_INTAKE_PORT, PISTON_EXTENDED);

// Claw starts RETRACTED.  You did not ask for it to default extended like the
// other two, so this is the odd one out on purpose - flip to PISTON_EXTENDED
// if it should start out.
pros::adi::DigitalOut claw(CLAW_PORT, PISTON_RETRACTED);

// Software mirror of what each solenoid was last told to do.  A DigitalOut
// cannot be read back, so this is the only record of piston state - and the
// r_motor_b interlock below depends on it.
bool high_intake_extended   = true;
bool middle_intake_extended = true;
bool claw_extended          = false;

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-20, -12,},     // Left Chassis Ports (negative port will reverse it!)
    {10, 3,},  // Right Chassis Ports (negative port will reverse it!)

    19,      // IMU Port
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
  // lv_obj_clean() below replaces pros::lcd::shutdown().  lcd_initialize() builds
  // 8 objects; lcd_shutdown() deletes exactly one and leaks the other seven.

  // Spinner — drawn on the screen LVGL already owns.
  // Do NOT do lv_obj_create(nullptr) + lv_obj_remove_style_all() + lv_scr_load()
  // here: remove_style_all() strips the screen's width and height along with
  // everything else, so that screen never covers the panel.  PROS's loading bar
  // stays visible underneath it and only the spinner's own pixels paint on top.
  // lv_scr_act() is already display-sized, so styling it directly works.
  lv_obj_clean(lv_scr_act());
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(UI_DARK_BG), 0);
  lv_obj_set_style_bg_opa(lv_scr_act(),   LV_OPA_COVER,             0);

  lv_obj_t* spinner = lv_spinner_create(lv_scr_act(), 1200, 75);
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
    case 0: drive_example();           break;   // Auto 1
    case 1: turn_example();            break;   // Auto 2
    case 2: drive_and_turn();          break;   // Auto 3
    case 3: wait_until_change_speed(); break;   // Auto 4
    case 4: swing_example();           break;   // Auto 5
    default:                           break;   // nothing selected
  }
}
// NOTE: EZ-Template's stock main.cpp defines screen_print_tracker() and
// ez_screen_task() here, plus the global `pros::Task ezScreenTask(ez_screen_task);`.
// All three have been REMOVED for the brain UI.
//  - They draw to the brain with ez::screen_print(), which is LLEMU.
//  - initialize() calls pros::lcd::shutdown() and hands the display to the UI engine.
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
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    // X is half of the UI's UP+X driver-mode combo, so arming driver mode would
    // otherwise toggle the PID tuner at the same time.  That is fatal: the tuner
    // calls pros::lcd::shutdown() and builds its own LLEMU display over the UI
    // engine's screens, while the engine's background tasks are still writing to
    // them.  Require X on its own, with UP up.
    if (master.get_digital_new_press(DIGITAL_X) && !master.get_digital(DIGITAL_UP))
      chassis.pid_tuner_toggle();

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
  static bool ctrl_flushed = false;

  while (true) {
    handle_ctrl_input();
    ez_template_extras();        // ← keeps DOWN+B auton test and X PID tuner

    chassis.opcontrol_arcade_standard(ez::SPLIT);

    if (!ctrl_flushed) { ctrl_flushed = true; CtrlFlush(); }

    // ── Add your subsystem controls here ──────────────────────────────────
    // Gated on driver mode: outside it the UI owns LEFT/RIGHT/A/B, so navigating
    // the auton menu must not be able to spin a motor or fire a piston.
    if (DriverModeActive()) {


      // Pistons - HOLD to retract, release to extend.  Not a toggle.
      //  - hold B: HIGH intake retracts  (this is the "middle" position)
      //  - hold Y: BOTH retract           (the "low" position)
      //  - release: whatever you were holding down goes back up ("high" position)
      bool high_want   = !(master.get_digital(DIGITAL_B) || master.get_digital(DIGITAL_Y));
      bool middle_want = !master.get_digital(DIGITAL_Y);

      // Only write to the solenoid when the state actually changes, rather than
      // re-sending the same value every 10ms tick.
      if (high_want != high_intake_extended) {
        high_intake_extended = high_want;
        high_intake.set_value(high_want);
      }
      if (middle_want != middle_intake_extended) {
        middle_intake_extended = middle_want;
        middle_intake.set_value(middle_want);
      }

      // Claw - DOWN arrow TOGGLES it.  Unlike the two above, this one latches:
      // press once to extend, press again to retract.
      if (master.get_digital_new_press(DIGITAL_DOWN)) {
        claw_extended = !claw_extended;
        claw.set_value(claw_extended);
      }

      // R1 / R2 group - four motors
      //  - R1: A, B, C forward and D backward
      //  - R2: every one of them reversed from what R1 does
      //  - r_motor_b (port 1) is INTERLOCKED.  It runs in every position EXCEPT
      //    the high state, which is both pistons extended:
      //        HIGH   - both extended (nothing held) -> stopped
      //        MIDDLE - high retracted (B held)      -> runs with the group
      //        LOW    - both retracted (Y held)      -> runs with the group
      bool port1_enabled = !(high_intake_extended && middle_intake_extended);

      if (master.get_digital(DIGITAL_R1)) {
        r_motor_a.move(R_SPEED);
        r_motor_b.move(port1_enabled ? R_SPEED : 0);
        r_motor_c.move(R_SPEED);
        r_motor_d.move(-R_SPEED);
      } else if (master.get_digital(DIGITAL_R2)) {
        r_motor_a.move(-R_SPEED);
        r_motor_b.move(port1_enabled ? -R_SPEED : 0);
        r_motor_c.move(-R_SPEED);
        r_motor_d.move(R_SPEED);
      } else {
        r_motor_a.move(0);
        r_motor_b.move(0);
        r_motor_c.move(0);
        r_motor_d.move(0);
      }

      // L1 / L2 pair - two motors, always opposite each other
      //  - L1: A forward, B backward, at full L_SPEED
      //  - L2: both flipped from what L1 does, at the slower L2_SPEED
      if (master.get_digital(DIGITAL_L1)) {
        l_motor_a.move(L_SPEED);
        l_motor_b.move(-L_SPEED);
      } else if (master.get_digital(DIGITAL_L2)) {
        l_motor_a.move(-L2_SPEED);
        l_motor_b.move(L2_SPEED);
      } else {
        l_motor_a.move(0);
        l_motor_b.move(0);
      }

    } else {
      // Parked while the UI has the controller.  move(0) rather than skipping,
      // or a motor holds whatever it was last told to do.
      l_motor_a.move(0);
      l_motor_b.move(0);
      r_motor_a.move(0);
      r_motor_b.move(0);
      r_motor_c.move(0);
      r_motor_d.move(0);
      // Pistons hold their state and are not re-commanded here.
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

