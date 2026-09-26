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
extern double CASCADE_OUT;         // phase 2 raise height, above flip
extern int    CASCADE_DOWN_SPEED;     // power for downward moves
extern int    CASCADE_FLIP_DELAY_MS;  // ms into that rise before the piston fires
extern int    CASCADE_AFTER_FLIP_MS;  // ms after the flip piston extends
                                      //   before the cascade moves again
extern int    CASCADE_FLIP_BACK_MS;   // ms into the first-click rise before
                                      //   the flip piston flips back
extern int    CASCADE_FLIP_RELEASE_MS;// ms to wait after releasing the flip
                                      //   piston before lowering to collect

// Upper travel limit.  L1 stops raising once the rotation sensor reads this,
// so the cascade cannot be driven into its top stop.  Manual control only -
// the macro's targets are all below it.
extern double CASCADE_MAX;      // L1 stops raising here

/////
// CASCADE MOVEMENT - CHANGE THESE
/////
constexpr int    CASCADE_MOVE_SPEED   = 90;   // 0-127, how hard it drives to a height
constexpr double CASCADE_MOVE_TOL     = 5;    // degrees; close enough to call it arrived
constexpr double CASCADE_MOVE_SLOW    = 50;   // degrees out, start easing off
constexpr int    CASCADE_MOVE_MIN     = 35;   // floor power - MUST be enough to move a
                                              // loaded cascade, or the approach creeps to a
                                              // halt and the stall guard fires
constexpr int    CASCADE_MOVE_TIMEOUT = 3000; // ms before a move gives up

// After reaching the target the cascade is braked and watched for this long,
// and driven back if momentum has carried it outside CASCADE_MOVE_TOL.  Without
// this a move ends the moment it touches the target and never looks again, so
// a heavy cascade coasts well past - 60 degrees past 375, measured.
constexpr int    CASCADE_SETTLE_MS    = 400;  // ms - MAXIMUM time to spend
                                              //   settling; it leaves as soon
                                              //   as it is steady
// How long the cascade must stay inside tolerance before the move is called
// done.  This is what a clean move actually costs, not CASCADE_SETTLE_MS.
constexpr int    CASCADE_SETTLE_STABLE_MS = 80;
constexpr int    CASCADE_SETTLE_POWER = 30;   // gentle correction power

// If the cascade has not moved CASCADE_STALL_DEG in CASCADE_STALL_MS, something
// is wrong - it is jammed, or the direction constant below is inverted - so the
// move aborts instead of driving into a hard stop until the timeout expires.
constexpr double CASCADE_STALL_DEG = 2;
constexpr int    CASCADE_STALL_MS  = 600;

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
// The claw and the flip piston are plumbed differently, so each gets its own
// pair.  Swap a pair if that piston works the other way round on the robot.
constexpr bool CLAW_ON   = true;    // claw gripping
constexpr bool CLAW_OFF  = false;   // claw open

constexpr bool FLIP_ON   = false;   // flip piston activated
constexpr bool FLIP_OFF  = true;    // flip piston released

// Time given to a solenoid to finish moving before the cascade starts again.
constexpr int MACRO_PISTON_SETTLE = 300;  // ms

// Intake power while the macro runs it (phase 2).
constexpr int MACRO_INTAKE_SPEED = 127;  // 0-127

// Pause after every macro action.  0 = normal speed.  Set to 1000 to step
// through the sequence one action at a time when debugging it.
constexpr int MACRO_STEP_DELAY = 0;  // ms

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

// True if the last run ended STALLED or TIMEOUT.  Stays true until the next
// press, so the failing step stays on the controller instead of vanishing the
// moment the macro stops.
bool macro_failed();

/////
// For autons
/////
// Raise or lower the cascade to a rotation-sensor value WITHOUT blocking, so
// the robot can drive at the same time.  Direction is worked out from where
// the cascade is now - a target above it raises, below it lowers.
//
//   cascade_move_async(CASCADE_OUT);             // starts moving, returns at once
//   chassis.pid_drive_set(24_in, DRIVE_SPEED);   // drives while it rises
//   chassis.pid_wait();
//   cascade_move_wait();                         // now make sure it arrived
//
// speed is 0-127; leave it out for the macro's normal speed.  Does nothing if
// a macro phase or another move is already running.  The cascade holds
// position once it arrives.
void cascade_move_async(double target, int speed = 0);

// True while a background move or a macro phase is driving the cascade.
bool cascade_move_active();

// Block until that finishes.  Returns false on timeout, or if the move stalled
// or timed out on its own.
bool cascade_move_wait(int timeout_ms = 4000);

// Run the collect macro from an auton, exactly as pressing RIGHT does in
// driver control.  Two presses: the first goes up, flips and parks at collect;
// the second grips, lifts, flips and returns to low.
//
//   macro_press();              // first press
//   macro_wait_done();          // ... parks at collect
//   intake_spin(1000, 127);     // do whatever you need here
//   macro_press();              // second press
//   macro_wait_done();          // ... back at low
void macro_press();
bool macro_wait_done(int timeout_ms = 8000);
