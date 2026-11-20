#pragma once

// Same PROS settings main.h makes, in case this header is included BEFORE
// main.h (an editor auto-include put it above main.h more than once).  PROS
// only reads them the first time its headers load, so without this the short
// names (MOTOR_BRAKE_HOLD ...) vanish for the whole file.
#ifndef PROS_USE_SIMPLE_NAMES
#define PROS_USE_SIMPLE_NAMES
#endif
#ifndef PROS_USE_LITERALS
#define PROS_USE_LITERALS
#endif

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Motors - the ports for these are set at the top of main.cpp
extern pros::Motor l_motor_a;  // L1 / L2 pair - always opposite l_motor_b
extern pros::Motor l_motor_b;
extern pros::Motor fin_1;  // R1 / R2 intake - with dropdown and fin_2
extern pros::Motor dropdown;  // front intake; the back roller is chained to it
extern pros::Motor fin_2;  // always opposite fin_1
extern pros::Motor side_toggles;  // port 19 - B held in driver control only

// Pneumatics - single-acting solenoids
extern pros::adi::DigitalOut claw;    // port A - toggled by DOWN
extern pros::adi::DigitalOut c_flip;  // port B - toggled by LEFT
extern pros::adi::DigitalOut piston_h;  // port H - extended at power-on, Y retracts
extern bool claw_extended;
extern bool c_flip_extended;
extern bool piston_h_extended;

/////
// Subsystem helpers - prefer these to touching the devices directly
/////
// Set a piston and its state mirror together.  Never call .set_value() on a
// solenoid directly: the mirrors are the only record of piston state, and the
// DOWN/LEFT toggles and the controller readout read them.
void claw_set(bool on);
void flip_set(bool on);
void piston_h_set(bool on);   // true = extended (PISTON_H_ON in main.cpp)

// Whole intake, -127 to 127.  Positive runs it the same way the R1 button
// does.  fin_2 is commanded opposite the others because it is mounted that
// way.  The back roller is chained to the front intake, so this turns it too.
void intake_set(int power);

// Side toggles (port 19), -127 to 127.  Driver control runs them while B is
// held; the autons and the macro never do.
void side_toggles_set(int power);

// Timed, non-blocking spins.  Each returns immediately and a background task
// drives the motors, so the next drive or turn starts straight away:
//
//   intake_spin(800, 127);                      // returns at once
//   chassis.pid_drive_set(24_in, DRIVE_SPEED);  // intake still spinning
//   chassis.pid_wait();
//
//   ms > 0   run for that long, then stop
//   ms < 0   run until stopped
//   ms == 0  stop that group now (so does speed == 0)
//
// Positive speed runs the motors the way R1 does.  The two groups (fins,
// dropdown) are INDEPENDENT and can overlap.  intake_spin() is shorthand for
// both at once.  Starting the same group again replaces its previous spin.
// Simple arc: give it the two side speeds and the heading to stop at, and it
// drives until the robot faces that heading.  Blocks, so no pid_wait() after.
//
//   drive_arc(90, 100, 40);    // curve right to 90 degrees
//   drive_arc(0, 40, 100);     // curve back to 0
//   drive_arc(90, 80, -80);    // spin on the spot
//
// Heading is ABSOLUTE, like pid_turn_set.  Open-loop - the speeds you give are
// the speeds it drives at - so it will not self-correct like the PID motions.
// Returns false on timeout.
bool drive_arc(double target_deg, int left_speed, int right_speed, int timeout_ms = 3000);

// Drive for a set time WITHOUT blocking - for pushing into a wall or goal and
// staying square while you score.  Returns at once; pistons, intakes and the
// cascade can all run while the robot keeps pushing:
//
//   drive_for_time(1500);            // back into the goal for 1.5 s
//   pros::delay(200);
//   claw_set(CLAW_OFF);              // score while still pushed in
//   intake_spin(800, 127);
//   drive_for_time_wait();           // let the push finish
//   chassis.pid_drive_set(10_in, DRIVE_SPEED);
//
// speed is -127 to 127, negative = backwards.  Leave it out for a medium
// backwards push (-60) - raise it if the robot gets shoved off the wall, lower
// it if the wheels slip.  Calling it again replaces
// the push in progress.
//
// The next drive command cancels the push automatically - pid_drive_set,
// turns, swings, odom moves, drive_arc, vision_drive and vision_align all take
// the wheels straight back, and the robot drives off in the direction the wall
// squared it to.  Only call drive_for_time_wait() if you want the FULL time.
void drive_for_time(int ms, int speed = -60);

// Timed arc: each side at its own speed for a set time.  Same background
// worker as above - non-blocking, cancelled by the next drive command, and
// drive_for_time_wait() / _stop() work on it the same way.
//
//   drive_for_time(800, 100, 40);    // curve right for 0.8 s
//   drive_for_time_wait();           // block until it finishes
void drive_for_time(int ms, int left_speed, int right_speed);
void drive_for_time_stop();          // end the push early
bool drive_for_time_active();        // true while pushing
bool drive_for_time_wait(int timeout_ms = 5000);  // block until the push ends

// Drive one part of the intake directly - it keeps running until you set it
// again.  Positive runs them the way R1 does; fin_2 is commanded opposite
// automatically.
void fins_set(int power);      // fin_1 + fin_2, kept in step

// Records where both fins are now as their alignment.  While they spin, the
// one that gets ahead of this is slowed until they match.  Called at the start
// of every auton and at power-on - align the fins by hand before then.
void fins_sync_zero();

// Fin jam guard - AUTONS ONLY.  autonomous() turns it on and opcontrol() /
// disabled() turn it off.  If a fin feels a big load (FIN_JAM_NM in main.cpp)
// the fins stop - only the fins - until the next intake_spin(), fins_spin(),
// fins_set() or intake_set().  fins_jammed() is true while they are stopped.
void fins_jam_guard(bool on);
bool fins_jammed();
void dropdown_set(int power);  // port 4

void fins_spin(int ms, int speed);
void upper_roller_spin(int ms, int speed);   // = intake_spin(): the roller is chained to it now
void dropdown_spin(int ms, int speed);
void intake_spin(int ms, int speed);
void intake_spin_stop();
bool intake_spin_active();

// Block until every running spin has finished - use when you want a timed spin
// to complete before the next line runs.  Returns false on timeout, which only
// happens if something was started with ms < 0 (run until stopped).
bool spin_wait(int timeout_ms = 5000);

// Cascade pair, -127 to 127.  The two motors always run opposite each other.
void cascade_set(int power);

// De-energise every solenoid so the cylinders vent.
void release_all_pistons();

// Which cascade motor holds: 0 = l_motor_a, 1 = l_motor_b.  Only one holds at
// a time, and the macro swaps them after each trip back to low so the heat of
// carrying the cascade is shared.
extern int cascade_hold_motor;
void cascade_apply_hold_motor();  // push the current choice to the motors
void cascade_swap_hold_motor();   // change holder, then apply
void cascade_hold();              // brake the holder, free-wheel the other

// Cascade hold - see the CASCADE_HOLD_* constants at the top of main.cpp
double cascade_position();  // cascade position in degrees, from the sensor
constexpr int8_t DISTANCE_PORT = 16;
extern pros::Distance distance_sensor;
extern pros::Rotation cascade_rot;  // port 14 - cascade position
extern bool   cascade_holding;
extern double cascade_target;
extern double cascade_last_err;

// AI Vision sensor - smart port 15
extern pros::AIVision ai_cam;

// Your other motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');