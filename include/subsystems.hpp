#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Motors - the ports for these are set at the top of main.cpp
extern pros::Motor l_motor_a;  // L1 / L2 pair - always opposite l_motor_b
extern pros::Motor l_motor_b;
extern pros::Motor r_motor_a;  // R1 / R2 group - a, b, c together
extern pros::Motor r_motor_b;
extern pros::Motor r_motor_c;
extern pros::Motor r_motor_d;  // always opposite a, b, c

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

// Whole intake group, -127 to 127, positive collects.  Handles r_motor_d
// running opposite and keeps the dropdown's piston interlock.
void intake_set(int power);

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