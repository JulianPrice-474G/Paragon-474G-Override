#pragma once

#include "api.h"

/////
// CASCADE HEIGHTS - CHANGE THESE
/////
// Rotation sensor readings, the "p" value on the controller's middle row.
// Drive the cascade where you want it, read p, put the number here.
constexpr double CASCADE_LOW     = 180;  // bottom / travel
constexpr double CASCADE_COLLECT = 276;  // intake height

// Upper travel limit.  L1 stops raising once the rotation sensor reads this,
// so the cascade cannot be driven into its top stop.  Manual control only -
// the macro's targets are all below it.
constexpr double CASCADE_MAX = 1000;

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
// Running the macro
/////
// Sends the cascade to the collect height, or back to low if it is already
// there.  Runs in its own task, so the drivetrain never stops responding.
// Pressing again while it moves cancels instead of starting a second one.
void macro_start();

// Asks a running move to stop at its next check, leaving the cascade where it
// is.  Nothing else is touched.
void macro_cancel();

// True while the macro owns the CASCADE.  opcontrol() checks this and skips
// the manual L1/L2 controls, or both write every tick and the macro loses.
// The intake, claw and drivetrain stay under driver control throughout.
bool macro_running();

// One line of status for the controller screen.
const char* macro_status_text();

// True when the cascade is at the collect height, within CASCADE_COLLECT_TOL.
// The upper intake roller only runs when this is true.
bool cascade_at_collect();
