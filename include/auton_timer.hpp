#pragma once

#include <cstdint>

/////
// AUTON TIMER - 15 s cut-off for LEFT+B test runs
/////
// Ends a LEFT+B test run at exactly AUTON_TIMER_MS after it starts, the way
// the field ends autonomous in a real match:
//
//   1. VEXos: every motor goes limp (coast) and ignores commands.
//   2. PROS:  the auton task is deleted wherever it is - mid-drive, mid-wait,
//             anywhere.  Nothing after that line of the routine runs.
//   3. PROS:  disabled() runs - in this code that vents every piston.
//
// If the routine finishes early, the robot sits as it was left (PID still
// holding, timed intake spins still going) until the 15 s mark, exactly as in
// a match, and only then is cut.
//
// After the cut the robot STAYS disabled (limp) so you can see where it ended
// up.  Press any controller button to get control back.
//
// Controller rumble at the cut:
//   "-"  the routine was still running - it was cut off
//   ".." the routine had already finished
// The exact times are printed to the terminal (pros terminal).
//
// Only LEFT+B runs are timed.  A real match or a competition switch is ended
// by PROS itself and never goes through this.
//
// TO REMOVE: set AUTON_TIMER_ON to false (LEFT+B then runs exactly as it
// did before).  To take it out completely, delete this file and
// src/auton_timer.cpp, and in ez_template_extras() in main.cpp change
// auton_run_timed() back to autonomous() and remove the #include.
constexpr bool     AUTON_TIMER_ON = true;   // set to true to enable the timer
constexpr uint32_t AUTON_TIMER_MS = 15000;   // auton period length

// Run the selected auton the way a LEFT+B test does, timed as above.  Blocks
// until the cut, then until a button is pressed.
void auton_run_timed();
