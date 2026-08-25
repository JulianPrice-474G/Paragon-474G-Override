#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Motors - the ports for these are set at the top of main.cpp
extern pros::Motor intake_left;
extern pros::Motor intake_right;
extern pros::Motor intake_2;
extern pros::Motor arm;
extern pros::Motor cascade;    // 11W, one end of the shaft
extern pros::Motor cascade_2;  // 5.5W, other end of the shaft

// Your other motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');