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

// Timed, non-blocking spins.  Each returns immediately and a background task
// drives the motors, so the next drive or turn starts straight away:
//
//   roller_spin(800, 127);                      // returns at once
//   chassis.pid_drive_set(24_in, DRIVE_SPEED);  // roller still spinning
//   chassis.pid_wait();
//
//   ms > 0   run for that long, then stop
//   ms < 0   run until stopped
//   ms == 0  stop that group now (so does speed == 0)
//
// Positive speed runs the motors the way R1 does.  The three groups are
// INDEPENDENT and can overlap - a roller spin does not disturb a fins spin.
// intake_spin() is shorthand for all three at once.  Starting the same group
// again replaces its previous spin.
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

// Drive one part of the intake directly - it keeps running until you set it
// again.  Positive runs them the way R1 does; fin_2 is commanded opposite
// automatically, and dropdown_set keeps the piston interlock.
void fins_set(int power);      // fin_1 + fin_2
void roller_set(int power);    // port 19, the upper roller
void dropdown_set(int power);  // port 4

void fins_spin(int ms, int speed);
void roller_spin(int ms, int speed);
void dropdown_spin(int ms, int speed);
void intake_spin(int ms, int speed);
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