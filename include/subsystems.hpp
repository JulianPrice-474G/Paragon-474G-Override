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
extern pros::adi::DigitalOut claw;
extern bool high_intake_extended;
extern bool middle_intake_extended;
extern bool claw_extended;

// Your other motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');