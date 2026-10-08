#include "main.h"
#include "auton_timer.hpp"
#include "pros/apix.h"   // registry_get_plugged_type()

// What a real match does when autonomous ends, and how each step is copied
// here.  Checked against the PROS 4.2.2 kernel (system_daemon.c) and VEXos:
//
//  - VEXos (1.0.11 and later): the moment the field disables the robot, every
//    smart motor is forced to COAST and motor commands are ignored, whatever
//    brake mode the code set.  A cascade or drive set to HOLD is not held.
//    3-wire outputs are NOT disabled - pistons keep their state and can still
//    be changed.
//  - PROS: its system daemon (priority MAX-2) checks the competition status
//    every 2 ms.  On the change it deletes the auton task wherever it is, then
//    starts a new task that runs disabled().  It stops nothing else - every
//    other task keeps running, VEXos just ignores their motor commands.
//
// Here there is no VEXos disable to ignore those commands, so instead the
// background jobs that drive motors (EZ PID, timed intake spins, drive pushes,
// the cascade macro) are stopped, and every motor is set to coast and kept
// there until a button is pressed.

extern "C" {
// PROS kernel (vdml.h), not in the public headers.  Takes the lock of every
// device port, waiting for any call already in progress to finish.
void port_mutex_take_all(void);
void port_mutex_give_all(void);
}

static pros::task_t       _auton_task  = nullptr;
static volatile bool      _auton_done  = true;    // the routine has returned
static volatile bool      _cut_done    = false;   // the end of auton has been done
static volatile uint64_t  _t0_us       = 0;       // when the run started
static volatile uint64_t  _finished_us = 0;       // when the routine returned, from _t0_us

/////
// Motors - VEXos's disable
/////
static constexpr int        _NUM_PORTS = 21;
static bool                 _is_motor[_NUM_PORTS] = {};
static pros::motor_brake_mode_e_t _saved_brake[_NUM_PORTS];

// Every motor plugged in, whatever it is, to coast at zero.  save = also
// record each one's brake mode first, to put back afterwards.
static void coast_all(bool save) {
  for (int p = 0; p < _NUM_PORTS; p++) {
    if (save) _is_motor[p] = pros::c::registry_get_plugged_type(p) == pros::c::E_DEVICE_MOTOR;
    if (!_is_motor[p]) continue;
    if (save) _saved_brake[p] = pros::c::motor_get_brake_mode(p + 1);
    pros::c::motor_set_brake_mode(p + 1, pros::E_MOTOR_BRAKE_COAST);
    pros::c::motor_move(p + 1, 0);
  }
}

static void restore_brakes() {
  for (int p = 0; p < _NUM_PORTS; p++)
    if (_is_motor[p]) pros::c::motor_set_brake_mode(p + 1, _saved_brake[p]);
}

/////
// The end of auton
/////
static void end_of_auton() {
  const uint64_t cut_us = pros::micros() - _t0_us;

  // 1. VEXos: every motor limp, at the buzzer.
  coast_all(true);

  // 2. PROS: delete the auton task wherever it is.  Every port lock is taken
  //    first, so the task is never deleted halfway through a device call - that
  //    would leave the device locked and frozen for the rest of the session.
  //    (PROS skips this; it is the one place this is safer than a real match.)
  //    This task outranks the routine, so it cannot finish in between.
  port_mutex_take_all();
  const bool was_running = !_auton_done;
  if (was_running) pros::c::task_delete(_auton_task);
  port_mutex_give_all();

  // 3. PROS: disabled() - vents every piston, clears the auton's macro state.
  disabled();

  // 4. Stand-in for VEXos ignoring motor commands: stop everything still
  //    driving a motor in the background.
  drive_for_time_stop();
  chassis.drive_set(0, 0);   // EZ to DISABLE - its task stops writing the drive
  intake_spin_stop();
  macro_cancel();
  fins_set(0);

  // Let them make their last writes (a cancelled macro step brakes the cascade,
  // which is coast now; the spin worker does one zeroing pass), then limp again.
  const uint32_t t = pros::millis();
  while ((macro_running() || intake_spin_active() || drive_for_time_active()) &&
         pros::millis() - t < 300)
    pros::delay(1);
  pros::delay(ez::util::DELAY_TIME);
  coast_all(false);

  const uint32_t cut_ms = (uint32_t)(cut_us / 1000);
  if (was_running) {
    printf("[auton timer] CUT at %lu.%03lu s - the routine was still running\n",
           (unsigned long)(cut_ms / 1000), (unsigned long)(cut_ms % 1000));
  } else {
    const uint32_t fin_ms = (uint32_t)(_finished_us / 1000);
    printf("[auton timer] routine finished at %lu.%03lu s, cut at %lu.%03lu s\n",
           (unsigned long)(fin_ms / 1000), (unsigned long)(fin_ms % 1000),
           (unsigned long)(cut_ms / 1000), (unsigned long)(cut_ms % 1000));
  }
  master.rumble(was_running ? "-" : "..");

  _cut_done = true;
}

// Higher priority than everything else in user code and than the PROS daemon,
// so the cut lands on time and the routine cannot run while it happens.
static void watchdog_task(void*) {
  while (true) {
    pros::Task::notify_take(true, TIMEOUT_MAX);   // armed by auton_run_timed()

    const uint64_t end_us = _t0_us + (uint64_t)AUTON_TIMER_MS * 1000;

    // Sleep in 1 ms steps to within 1 ms of the end.  Plugging into a
    // competition switch mid-run hands the robot to PROS, so end at once.
    bool comp = false;
    while (pros::micros() + 1000 < end_us) {
      if (pros::competition::is_connected()) { comp = true; break; }
      pros::delay(1);
    }
    // The last stretch on the microsecond clock - delay() only has 1 ms steps.
    while (!comp && pros::micros() < end_us) {}

    end_of_auton();
  }
}

static void auton_task(void*) {
  autonomous();
  _finished_us = pros::micros() - _t0_us;
  _auton_done  = true;   // last
}

static bool any_button() {
  static const pros::controller_digital_e_t buttons[] = {
    pros::E_CONTROLLER_DIGITAL_L1,   pros::E_CONTROLLER_DIGITAL_L2,
    pros::E_CONTROLLER_DIGITAL_R1,   pros::E_CONTROLLER_DIGITAL_R2,
    pros::E_CONTROLLER_DIGITAL_UP,   pros::E_CONTROLLER_DIGITAL_DOWN,
    pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT,
    pros::E_CONTROLLER_DIGITAL_X,    pros::E_CONTROLLER_DIGITAL_B,
    pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A};
  for (auto b : buttons)
    if (master.get_digital(b)) return true;
  return false;
}

void auton_run_timed() {
  if (!AUTON_TIMER_ON) { autonomous(); return; }

  static pros::Task watchdog(watchdog_task, nullptr, TASK_PRIORITY_MAX - 1,
                             TASK_STACK_DEPTH_DEFAULT, "Auton Timer");

  // The routine runs in its own task, as in a match, so deleting it never
  // touches opcontrol.  It ends by itself if it returns, so nothing leaks.
  _cut_done   = false;
  _auton_done = false;
  _t0_us      = pros::micros();
  pros::Task auton(auton_task, nullptr, TASK_PRIORITY_DEFAULT,
                   TASK_STACK_DEPTH_DEFAULT, "Auton (timed)");
  _auton_task = static_cast<pros::task_t>(auton);
  watchdog.notify();

  while (!_cut_done) pros::delay(ez::util::DELAY_TIME);

  // Stay disabled until a button is pressed, so the robot can be seen where
  // the buzzer left it.  Re-coasting every tick stands in for VEXos ignoring
  // motor commands.  Wait for the press to be RELEASED too, so it is not also
  // taken as a menu or driver input.
  while (any_button())  { coast_all(false); pros::delay(ez::util::DELAY_TIME); }
  while (!any_button()) { coast_all(false); pros::delay(ez::util::DELAY_TIME); }
  while (any_button())  { coast_all(false); pros::delay(ez::util::DELAY_TIME); }

  // Back to normal: each motor's own brake mode, the cascade's one holder.
  restore_brakes();
  cascade_apply_hold_motor();
}
