#pragma once

#include "api.h"

/////
// CASCADE HEIGHTS - CHANGE THESE
/////
// Rotation sensor readings, the "p" value on the controller's middle row.
// Drive the cascade where you want it, read p, put the number here.
constexpr double CASCADE_LOW     = 49;   // bottom / travel
constexpr double CASCADE_COLLECT = 144;  // intake height
constexpr double CASCADE_FLIP    = 201;  // where the flip piston fires

/////
// CASCADE MOVEMENT - CHANGE THESE
/////
constexpr int    CASCADE_MOVE_SPEED   = 90;   // 0-127, how hard it drives to a height
constexpr double CASCADE_MOVE_TOL     = 5;    // degrees; close enough to call it arrived
constexpr double CASCADE_MOVE_SLOW    = 30;   // degrees out, start easing off
constexpr int    CASCADE_MOVE_MIN     = 25;   // floor power, or it stalls short of target
constexpr int    CASCADE_MOVE_TIMEOUT = 2500; // ms before a move gives up

// If the cascade has not moved CASCADE_STALL_DEG in CASCADE_STALL_MS, something
// is wrong - it is jammed, or the direction constant below is inverted - so the
// move aborts instead of driving into a hard stop until the timeout expires.
constexpr double CASCADE_STALL_DEG = 2;
constexpr int    CASCADE_STALL_MS  = 350;

// Set to -1 if positive motor power LOWERS the cascade instead of raising it.
// Symptom of getting it wrong: the macro immediately stalls and aborts.
constexpr int CASCADE_RAISE_SIGN = 1;

/////
// MACRO TIMINGS - CHANGE THESE
/////
constexpr int MACRO_INTAKE_SPEED  = 127;   // 0-127, intake power during the macro
constexpr int MACRO_INTAKE_MS     = 3000;  // how long the intake runs at collect height
constexpr int MACRO_PISTON_SETTLE = 300;   // ms to let a piston finish moving
constexpr int MACRO_CLAW_SETTLE   = 400;   // ms to let the claw close before lifting

/////
// Running the macro
/////
// Starts the sequence in its own task, so the drivetrain never stops
// responding.  Does nothing if one is already running.
void macro_start();

// Asks a running macro to stop at its next check.  It will release the cascade
// and intake; pistons stay wherever they got to.
void macro_cancel();

// True while a macro owns the cascade, intake and claw.  opcontrol() checks
// this and leaves those subsystems alone - otherwise both write every tick and
// the macro loses.  The drivetrain is never owned.
bool macro_running();

// One line of status for the controller screen.
const char* macro_status_text();
