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

#include "api.h"

/////
// MACRO SETTINGS - set at the top of src/main.cpp
/////
// Two full sets: AUTO_ for autons, DRIVER_ for driver control.  The macro
// picks the set when each press starts.  Heights are rotation sensor readings.
extern double AUTO_CASCADE_LOW;             // bottom / travel
extern double AUTO_CASCADE_COLLECT;         // press 1 parks here, waiting for press 2
extern double AUTO_CASCADE_FLIP;            // press 1 rises here first
extern int    AUTO_DROP_DELAY_MS;           // press 1: wait after the claw/flip drop
extern bool   AUTO_CUSTOM_FLIP_ON;          // press 1 uses CUSTOM_FLIP / CUSTOM_DROP_DELAY_MS
extern double AUTO_CUSTOM_FLIP;
extern int    AUTO_CUSTOM_DROP_DELAY_MS;
extern double AUTO_CASCADE_OUT;             // press 2 rises here
extern int    AUTO_FLIP_DELAY_MS;           // press 2: ms into the rise before the flip
extern int    AUTO_AFTER_FLIP_MS;           // press 2: ms after the flip before moving on
extern int    AUTO_DOWN_SPEED;              // power for downward macro moves

extern double DRIVER_CASCADE_LOW;
extern double DRIVER_CASCADE_COLLECT;
extern double DRIVER_CASCADE_FLIP;
extern int    DRIVER_DROP_DELAY_MS;
extern bool   DRIVER_CUSTOM_FLIP_ON;
extern double DRIVER_CUSTOM_FLIP;
extern int    DRIVER_CUSTOM_DROP_DELAY_MS;
extern double DRIVER_CASCADE_OUT;
extern int    DRIVER_FLIP_DELAY_MS;
extern int    DRIVER_AFTER_FLIP_MS;
extern int    DRIVER_DOWN_SPEED;

// Upper travel limit.  L1 stops raising once the rotation sensor reads this,
// so the cascade cannot be driven into its top stop.  Manual control only, so
// one value for both modes.
extern double CASCADE_MAX;

// macro_grab() settings - set at the top of src/main.cpp.
extern int    GRAB_REV_MS;
extern int    GRAB_SPEED;

// one_pin_macro() defaults - set at the top of src/main.cpp.
extern int    ONE_PIN_FWD_MS;
extern int    ONE_PIN_CLAW_WAIT_MS;
extern int    ONE_PIN_REV_MS;
extern int    ONE_PIN_SPEED;

/////
// CASCADE MOVEMENT - CHANGE THESE
/////
constexpr int    CASCADE_MOVE_SPEED   = 127;   // 0-127, how hard it drives to a height
constexpr double CASCADE_MOVE_TOL     = 3;    // degrees; close enough to call it arrived
constexpr double CASCADE_MOVE_SLOW    = 127;   // degrees out, start easing off
constexpr int    CASCADE_MOVE_MIN     = 35;   // floor power - MUST be enough to move a
                                              // loaded cascade, or the approach creeps to a
                                              // halt and the stall guard fires
constexpr int    CASCADE_MOVE_TIMEOUT = 10000; // ms before a move gives up

// Going UP the cascade is lifting its own weight, so the eased-off power near
// the target needs a higher floor.  At CASCADE_MOVE_MIN it stalled a few
// degrees short of the flip height and never arrived.
constexpr int    CASCADE_MOVE_MIN_UP  = 127;

// If the cascade stops moving this close to the target, count it as arrived
// rather than waiting out the whole timeout for the last few degrees.
constexpr double CASCADE_CLOSE_ENOUGH   = 12;   // degrees
constexpr int    CASCADE_CLOSE_STALL_MS = 250;  // stopped this long = arrived

// After reaching the target the cascade is braked and watched for this long,
// and driven back if momentum has carried it outside CASCADE_MOVE_TOL.  Without
// this a move ends the moment it touches the target and never looks again, so
// a heavy cascade coasts well past - 60 degrees past 375, measured.
constexpr int    CASCADE_SETTLE_MS    = 200;  // ms - MAXIMUM time to spend
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
constexpr int    CASCADE_STALL_MS  = 2000;

// Set to -1 if positive motor power LOWERS the cascade instead of raising it.
// Symptom of getting it wrong: the macro immediately stalls and aborts.
constexpr int CASCADE_RAISE_SIGN = 1;

// How close to the collect height still counts as "in the collect state".  Wider
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

constexpr bool FLIP_ON   = true;    // flip piston activated
constexpr bool FLIP_OFF  = false;   // flip piston released

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
// its own task, so the drivetrain never stops responding.  L1/L2 do not
// cancel - they are ignored until the macro finishes.
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
//   cascade_move_async(AUTO_CASCADE_OUT);        // starts moving, returns at once
//   chassis.pid_drive_set(24_in, DRIVE_SPEED);   // drives while it rises
//   chassis.pid_wait();
//   cascade_move_wait();                         // now make sure it arrived
//
// speed is 0-127 and is used EXACTLY, up or down - never faster.  Very low
// speeds can stall short (going up especially), and the stall guard then
// gives up after CASCADE_STALL_MS.  Leave it out for the macro's normal
// behaviour: full speed, AUTO_DOWN_SPEED down, floor powers so it always
// arrives.  If a macro step
// or another move is still running - or a delayed macro_press() has yet to
// fire - the move WAITS and starts as soon as that is done, so it never breaks
// the macro.  A later call replaces one still waiting.  The cascade holds
// position once it arrives.
void cascade_move_async(double target, int speed = 0);

// True while a background move or a macro phase is driving the cascade, or a
// move is waiting its turn.
bool cascade_move_active();

// Block until that finishes.  Returns false on timeout, or if the move stalled
// or timed out on its own.  The timeout includes any time spent waiting for
// the macro, so give it longer when the macro may still be running.
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
// end_height is where the cascade finishes on the SECOND press, after the
// flip.  Leave it out and it stays at the out height:
//
//   macro_press();          // 1st press - up, flip, park at collect
//   macro_wait_done();
//   macro_press(500);       // 2nd press - grip, lift, flip, then go to 500
//   macro_wait_done();
//
//   macro_press(AUTO_CASCADE_LOW);   // ... or back down to the bottom
//   macro_press();              // ... or just stay at the out height
// If something is already driving the cascade:
//  - a cascade_move_async() - macro_press() cancels it and starts the macro
//    immediately.  A move still waiting its turn is dropped.
//  - a macro step that hasn't finished - the press WAITS and runs once that
//    step is done, so press 2 never lands in the middle of press 1.
// (RIGHT in driver control still just cancels.)
//
// delay_time (ms) presses the macro that long from now, WITHOUT blocking - the
// auton carries straight on, and drives, intakes and pistons are not held up:
//
//   macro_press(-1, 300);       // press in 300 ms, stay at the out height
//   macro_press(500, 300);      // press in 300 ms, finish at 500
//   chassis.pid_drive_set(24_in, DRIVE_SPEED);   // drives meanwhile
//
// The takeover above happens when the delay ends, not when you call it.
// macro_wait_done() also waits out the delay.  Another macro_press() before it
// fires replaces it; macro_cancel() drops it, and any queued press or move.
void macro_press(double end_height = -1, int delay_time = 0);
bool macro_wait_done(int timeout_ms = 8000);

// One pin macro.  Only from the parked state after press 1 (macro_waiting()).
// Runs ALL the intakes forward fwd_ms, closes the claw, waits claw_wait_ms,
// runs all the intakes backward rev_ms, then does press 2 - grip, lift, flip -
// finishing at end_height like macro_press(end_height).  In that press 2 the
// fins and dropdown OUTTAKE the whole time; the upper roller runs forward, then
// reverses at the flip, as in a normal press 2.  Returns at once; the
// cascade worker runs it, so the auton keeps driving.
//
//   one_pin_macro();                    // main.cpp defaults, stay at out height
//   one_pin_macro(500);                 // finish at 500
//   one_pin_macro(-1, 800, 0, 1200);    // 0.8 s forward, 1.2 s backward
//
// If press 1 is still running it waits for it.  If the macro is not parked
// after press 1 at all, it does nothing and returns false.  A cascade move
// still running is taken over, like macro_press().  macro_wait_done() waits
// for the whole thing.
bool one_pin_macro(double end_height = -1,
                   int fwd_ms       = ONE_PIN_FWD_MS,
                   int claw_wait_ms = ONE_PIN_CLAW_WAIT_MS,
                   int rev_ms       = ONE_PIN_REV_MS);

// X in driver control.  Only while parked after press 1 (macro_waiting()):
// runs all four intake motors backward, then after GRAB_REV_MS closes the
// claw, still backward.  The intake stays backward until press 2 (RIGHT)
// takes it over.  Returns false, doing nothing, if not parked.
bool macro_grab();

// Drops a delayed press that has not fired yet.  Called from disabled() and
// at the start of opcontrol(), so one left over from an auton never fires.
void macro_press_pending_clear();

// Tells the macro whether an auton is running, so it uses the AUTO_ settings
// rather than the DRIVER_ ones.  autonomous() sets it; opcontrol(), disabled()
// and the end of a LEFT+B test run clear it.
void macro_in_auton(bool on);
