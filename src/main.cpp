#include "main.h"
#include "ui_engine.hpp"

// Forward declarations — defined in src/user_screen.cpp
void build_screens();
int  get_selected_auton();
void handle_ctrl_input();
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

/////
// MOTOR PORTS - CHANGE THESE
// - put a minus in front of a port to reverse that motor (ex. -11)
/////
constexpr int8_t INTAKE_LEFT_PORT = 16;
constexpr int8_t INTAKE_RIGHT_PORT = 7;
constexpr int8_t CASCADE_PORT = 11;    // the 11W motor
constexpr int8_t CASCADE_2_PORT = -2;  // the 5.5W on the other end of the shaft.
                                        // Negative because it faces the opposite way - if it
                                        // fights the first motor instead of helping, flip this sign.

constexpr int8_t INTAKE_2_PORT = 1;    // the 5.5W on LEFT / RIGHT arrows
constexpr int8_t ARM_PORT = 6;         // TODO: set your real port - UP / DOWN arrows

/////
// MOTOR SPEEDS - CHANGE THESE
// - range is 0 to 127, where 127 is full power
/////
constexpr int INTAKE_SPEED = 127;    // R1 / R2
constexpr int INTAKE_2_SPEED = 127;  // LEFT / RIGHT arrows
constexpr int CASCADE_SPEED = 127;   // L1 / L2
constexpr int ARM_SPEED = 127;       // UP / DOWN arrows
constexpr int DRIVE_SPEED = 127;     // caps how much power the joysticks can ask for

// Intake - R1 runs it one way, R2 runs it the other way.
// These two motors always spin opposite each other.
pros::Motor intake_left(INTAKE_LEFT_PORT);
pros::Motor intake_right(INTAKE_RIGHT_PORT);

// Second intake (5.5W) - LEFT arrow runs it forward, RIGHT arrow runs it reverse
pros::Motor intake_2(INTAKE_2_PORT);

// Arm - UP arrow runs it forward, DOWN arrow runs it back
pros::Motor arm(ARM_PORT);

// Cascade - L1 runs it backward, L2 runs it forward.
// Two separate motors instead of a MotorGroup.  A group assumes its members are
// interchangeable, which an 11W and a 5.5W aren't, and it silently swallows a
// failure on one member.  Commanding them separately can't hide a dead motor.
pros::Motor cascade(CASCADE_PORT);
pros::Motor cascade_2(CASCADE_2_PORT);

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
  pros::lcd::initialize();  // required to start LVGL
  pros::lcd::shutdown();    // remove PROS text overlay immediately

  // Spinner — visible while build_screens() runs
  lv_obj_t* startup_scr = lv_obj_create(nullptr);
  lv_obj_remove_style_all(startup_scr);
  lv_obj_set_style_bg_color(startup_scr, lv_color_hex(UI_DARK_BG), 0);
  lv_obj_set_style_bg_opa(startup_scr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(startup_scr, LV_OBJ_FLAG_SCROLLABLE);
  lv_scr_load(startup_scr);

  lv_obj_t* spinner = lv_spinner_create(startup_scr, 1200, 75);
  lv_obj_set_size(spinner, 100, 100);
  lv_obj_center(spinner);
  lv_obj_set_style_arc_color(spinner, lv_color_hex(UI_GOLD),   LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(spinner, 8,                        LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(spinner, lv_color_hex(0x3A3A3A), LV_PART_MAIN);
  lv_obj_set_style_arc_width(spinner, 8,                        LV_PART_MAIN);
  lv_task_handler();

  chassis.opcontrol_curve_buttons_toggle(false); // reclaim controller buttons for UI use

  // ── Add your chassis setup here ──────────────────────────────────────────────
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  // The brain UI replaces EZ-Template's LLEMU selector.  get_selected_auton()
  // returns the index of whichever auton button was tapped (or -1 if none).
  // The number in each case must match the auton_idx given to that ButtonAdd.
  switch (get_selected_auton()) {
    // case 0: your_left_auton();   break;
    // case 1: your_right_auton();  break;
    // case 2: your_skills_route(); break;
    default: break;
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
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    // !!! WARNING: DOWN now runs the arm backward.  Holding DOWN and pressing B fires your
    // !!! whole autonomous routine.  Change this combo if that bites you.
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
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

    chassis.opcontrol_arcade_standard(ez::SPLIT);

    if (!ctrl_flushed) { ctrl_flushed = true; CtrlFlush(); }

    // ── Add your subsystem controls here ──────────────────────────────────

    // Intake
    //  - hold R1 to run it one direction, hold R2 to run it the other
    //  - the two motors always spin opposite each other
    if (master.get_digital(DIGITAL_R1)) {
      intake_left.move(INTAKE_SPEED);
      intake_right.move(-INTAKE_SPEED);
    } else if (master.get_digital(DIGITAL_R2)) {
      intake_left.move(-INTAKE_SPEED);
      intake_right.move(INTAKE_SPEED);
    } else {
      intake_left.move(0);
      intake_right.move(0);
    }

    // Second intake
    //  - hold LEFT arrow to run it forward, hold RIGHT arrow to run it reverse
    if (master.get_digital(DIGITAL_LEFT)) {
      intake_2.move(INTAKE_2_SPEED);
    } else if (master.get_digital(DIGITAL_RIGHT)) {
      intake_2.move(-INTAKE_2_SPEED);
    } else {
      intake_2.move(0);
    }

    // Arm
    //  - hold UP arrow to run it forward, hold DOWN arrow to run it back
    if (master.get_digital(DIGITAL_UP)) {
      arm.move(ARM_SPEED);
    } else if (master.get_digital(DIGITAL_DOWN)) {
      arm.move(-ARM_SPEED);
    } else {
      arm.move(0);
    }

    // Cascade
    //  - hold L1 to run it backward, hold L2 to run it forward
    //  - releasing both lets it coast
    if (master.get_digital(DIGITAL_L1)) {
      cascade.move(-CASCADE_SPEED);
      cascade_2.move(-CASCADE_SPEED);
    } else if (master.get_digital(DIGITAL_L2)) {
      cascade.move(CASCADE_SPEED);
      cascade_2.move(CASCADE_SPEED);
    } else {
      cascade.move(0);
      cascade_2.move(0);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}