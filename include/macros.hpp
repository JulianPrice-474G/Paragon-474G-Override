#pragma once

#include "api.h"

/////
// CASCADE HEIGHTS - CHANGE THESE
/////
// Rotation sensor readings, the "p" value on the controller's middle row.
// Drive the cascade where you want it, read p, put the number here.
// Defined at the top of src/main.cpp so they sit with the ports and speeds.
// Plain variables, not constexpr, so they can be changed in one place.
extern double CASCADE_LOW;      // bottom / travel
extern double CASCADE_COLLECT;  // intake height
extern double CASCADE_FLIP;     // raised above collect, where the flip piston fires

// Upper travel limit.  L1 stops raising once the rotation sensor reads this,
// so the cascade cannot be driven into its top stop.  Manual control only -
// the macro's targets are all below it.
extern double CASCADE_MAX;      // L1 stops raising here

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

// How close to CASCADE_COLLECT still counts as "in the collect state".  Wider
// than CASCADE_MOVE_TOL on purpose: the cascade drifts a little while holding,
// and the upper roller should not cut out when it does.
constexpr double CASCADE_COLLECT_TOL = 12;

/////
// PISTONS IN THE MACRO - CHANGE THESE
/////
// "Activated" means the solenoid is energised, which is the extended state -
// the same sense as PISTON_EXTENDED in main.cpp.  Swap these two if a piston
// turns out to work the other way round on your robot.
constexpr bool PISTON_ON  = true;   // activated / extended
constexpr bool PISTON_OFF = false;  // released / retracted

// Time given to a solenoid to finish moving before the cascade starts again.
constexpr int MACRO_PISTON_SETTLE = 300;  // ms

// Pause at flip height in phase 2 before the flip piston fires, so the cascade
// has stopped swinging first.
constexpr int MACRO_FLIP_SETTLE = 500;  // ms

// Intake power while the macro runs it (phase 2).
constexpr int MACRO_INTAKE_SPEED = 127;  // 0-127

// TESTING: pause this long after every single action, so each step can be
// watched on its own.  Set to 0 for normal speed once it is verified.
constexpr int MACRO_STEP_DELAY = 1000;  // ms

/////
// Running the macro
/////
// One press of the macro button.  The sequence runs in two halves:
//
//   press 1 -> flip height, then collect height, then WAIT
//              (the upper roller is armed for as long as it waits)
//   press 2 -> flip height, then back to low
//
// Pressing WHILE the cascade is moving cancels instead.  Everything runs in
// its own task, so the drivetrain never stops responding.
void macro_start();

// Asks a running move to stop at its next check, leaving the cascade where it
// is.  Nothing else is touched.
void macro_cancel();

// True while the macro is MOVING the cascade.  opcontrol() checks this and
// skips the manual L1/L2 controls, or both write every tick and the macro
// loses.  False while it waits at collect, so the driver keeps the cascade
// during the wait.  Intake, claw and drivetrain are never taken.
bool macro_running();

// One line of status for the controller screen.
const char* macro_status_text();

// True only after a macro move has COMPLETED at the collect height, and only
// until the cascade leaves that window again.  The upper intake roller runs on
// this, so driving past 276 with L1/L2 does not start it spinning.
bool cascade_at_collect();

// Physical proximity to the collect height, ignoring how it got there.  Used
// to pick which way RIGHT toggles.
bool cascade_near_collect();

// True while the sequence is paused at collect waiting for the second press.
bool macro_waiting();

// True while the macro is driving the intake itself.  opcontrol() checks this
// and skips its R1/R2 block - otherwise it writes zero to those motors every
// tick and the macro's intake never spins.
bool macro_owns_intake();
