#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Motors - the ports for these are set at the top of main.cpp
extern pros::Motor l_motor_a;  // L1 / L2 pair - always opposite l_motor_b
extern pros::Motor l_motor_b;
extern pros::Motor fin_1;  // R1 / R2 group - a, b, c together
extern pros::Motor dropdown;
extern pros::Motor upper_roller;
extern pros::Motor fin_2;  // always opposite a, b, c

// Pneumatics - single-acting solenoids, both default to extended
extern pros::adi::DigitalOut high_intake;
extern pros::adi::DigitalOut middle_intake;
extern pros::adi::DigitalOut claw;    // port C - toggled by RIGHT
extern pros::adi::DigitalOut c_flip;  // port D - toggled by DOWN
extern bool high_intake_extended;
extern bool middle_intake_extended;
extern bool claw_extended;
extern bool c_flip_extended;

/////
// Subsystem helpers - prefer these to touching the devices directly
/////
// Set a piston and its state mirror together.  Never call .set_value() on a
// solenoid directly: the mirrors are the only record of piston state, and the
// dropdown interlock and the DOWN/LEFT toggles both read them.
void claw_set(bool on);
void flip_set(bool on);
void high_intake_set(bool on);
void middle_intake_set(bool on);

// Whole intake group, -127 to 127.  Positive runs it the same way the R1
// button does.  fin_2 is commanded opposite the others because it is mounted
// that way, and the dropdown keeps its piston interlock - stopped only while
// BOTH intake pistons are extended.
//
// roller = false holds the upper roller at zero, which is what driver control
// does outside the collect height.  Autons and intake_spin() leave it true, so
// all four motors turn.
void intake_set(int power, bool roller = true);

// Run the intake WITHOUT blocking.  Returns immediately and a background task
// drives the motors, so the intake keeps turning through a drive or turn:
//
//   intake_spin(3000, 127);                      // 3 seconds, then stops itself
//   chassis.pid_drive_set(24_in, DRIVE_SPEED);   // starts straight away
//   chassis.pid_wait();
//
//   intake_spin(-1, 127);                        // run until stopped
//   ...
//   intake_spin_stop();
//
// speed is -127 to 127; positive runs it the same way R1 does.  Calling it
// again replaces the running spin rather than queueing one.  ms = 0 or
// speed = 0 stops it, same as intake_spin_stop().
void intake_spin(int ms, int speed);

// Just the two fins - the dropdown and the upper roller are left alone.
// fins_set() drives them directly; fins_spin() takes the same ms/speed as
// intake_spin() and runs in the background the same way.
//
// intake_spin() and fins_spin() share one worker, so starting either replaces
// whatever was running - they cannot fight over the fin motors.
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

void fins_set(int power);
void fins_spin(int ms, int speed);
void intake_spin_stop();
bool intake_spin_active();

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