#include "main.h"

#include <cmath>

/////
// State
/////
static bool        _running     = false;
static bool        _cancel      = false;
static const char* _step        = "idle";
static bool        _failed      = false;  // last run ended STALLED or TIMEOUT

bool macro_running() { return _running; }
bool macro_failed()  { return _failed; }
void macro_cancel()  { if (_running) _cancel = true; }

const char* macro_status_text() {
  static char buf[20];
  snprintf(buf, sizeof(buf), "%-13s",
           (_running || macro_waiting() || _failed) ? _step : "ready");
  return buf;
}

/////
// Subsystem helpers
/////
static void cascade_stop()            { cascade_set(0); }
static void cascade_drive(int power)  { cascade_set(power); }

/////
// Move the cascade to a height
/////
// Returns false if it was cancelled, timed out, or stalled.  Always leaves the
// cascade stopped, so a failure cannot leave a motor driving.
// action, if given, is called once, action_delay_ms after the move starts - so
// something can happen DURING the movement rather than before or after it.
static bool cascade_to(double target, const char* step_name,
                       int max_speed = CASCADE_MOVE_SPEED,
                       int action_delay_ms = -1, void (*action)() = nullptr) {
  _step = step_name;

  const uint32_t start        = pros::millis();
  uint32_t       last_progress = start;
  double         last_pos      = cascade_position();
  bool           action_fired  = false;

  while (pros::millis() - start < (uint32_t)CASCADE_MOVE_TIMEOUT) {
    if (_cancel) { cascade_stop(); return false; }

    // Fire the timed action once, however far into the move it falls.
    if (action && !action_fired &&
        (int)(pros::millis() - start) >= action_delay_ms) {
      action_fired = true;
      action();
    }

    double pos = cascade_position();
    double err = target - pos;

    if (fabs(err) <= CASCADE_MOVE_TOL) {
      // Brake rather than coast, then keep watching: a heavy cascade carries a
      // long way past the target on momentum, and the old code stopped looking
      // the instant it touched the target so it never pulled the overshoot back.
      cascade_hold();
      const uint32_t settle_start = pros::millis();
      while (pros::millis() - settle_start < (uint32_t)CASCADE_SETTLE_MS) {
        if (_cancel) { cascade_stop(); return false; }
        double e = target - cascade_position();
        if (fabs(e) > CASCADE_MOVE_TOL) {
          int p = (e > 0) ? CASCADE_SETTLE_POWER : -CASCADE_SETTLE_POWER;
          cascade_drive(p * CASCADE_RAISE_SIGN);
        } else {
          cascade_hold();
        }
        pros::delay(ez::util::DELAY_TIME);
      }
      cascade_hold();
      // Move finished before the delay elapsed - fire it anyway rather than
      // losing it.
      if (action && !action_fired) { action_fired = true; action(); }
      return true;
    }

    // Stall guard.  Without this, an inverted CASCADE_RAISE_SIGN drives into a
    // hard stop at full power for the whole timeout.
    if (fabs(pos - last_pos) >= CASCADE_STALL_DEG) {
      last_pos      = pos;
      last_progress = pros::millis();
    } else if (fabs(err) > CASCADE_MOVE_SLOW &&
               pros::millis() - last_progress > (uint32_t)CASCADE_STALL_MS) {
      // Only call it a stall while still driving hard.  Inside the easing zone
      // the cascade legitimately creeps, and treating that as a jam aborts a
      // move that was about to finish.
      cascade_stop();
      _step = "STALLED";
      return false;
    }

    // Full speed until CASCADE_MOVE_SLOW degrees out, then ease off so it does
    // not overshoot and oscillate around the target.
    double scale = fabs(err) / CASCADE_MOVE_SLOW;
    if (scale > 1.0) scale = 1.0;
    int power = (int)(max_speed * scale);
    if (power < CASCADE_MOVE_MIN) power = CASCADE_MOVE_MIN;
    if (err < 0) power = -power;

    cascade_drive(power * CASCADE_RAISE_SIGN);
    pros::delay(ez::util::DELAY_TIME);
  }

  cascade_stop();
  _step = "TIMEOUT";
  return false;
}

// Interruptible delay - a plain pros::delay() would ignore a cancel request
// for its whole duration.
static bool macro_wait(int ms, const char* step_name) {
  _step = step_name;
  const uint32_t start = pros::millis();
  while (pros::millis() - start < (uint32_t)ms) {
    if (_cancel) return false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return true;
}

// Set a solenoid and keep the software mirror in step.  A DigitalOut cannot be
// read back, so those mirrors are the only record of piston state - and the
// dropdown interlock in opcontrol depends on them.
// Intake, in the R1 direction - the same way R1 spins it manually.  The
// dropdown keeps its piston interlock so the macro cannot drive it in the one
// state where it must not run.  opcontrol skips its own R1/R2 block while
// _intake_owned is set.
static bool _intake_owned = false;
bool macro_owns_intake() { return _intake_owned; }

// roller_back reverses ONLY the upper roller (port 19); the fins and dropdown
// carry on in the normal direction.
static void intake_run(bool on, bool roller_back = false) {
  int p = on ? MACRO_INTAKE_SPEED : 0;
  bool dropdown_enabled = !(high_intake_extended && middle_intake_extended);
  fin_1.move(-p);
  dropdown.move(dropdown_enabled ? -p : 0);
  upper_roller.move(roller_back ? p : -p);
  fin_2.move(p);
}



// Testing pause between actions.  Keeps the finished step's label on the
// controller so you can see which one just completed, and stays cancellable.
static bool step_pause(const char* label) {
  if (MACRO_STEP_DELAY <= 0) return !_cancel;
  static char buf[20];
  snprintf(buf, sizeof(buf), "%s ...", label);
  return macro_wait(MACRO_STEP_DELAY, buf);
}

/////
// Where the sequence is up to
/////
enum _Phase {
  PH_IDLE,     // nothing running
  PH_GOING,    // phase 1 moving: flip height, then collect
  PH_WAITING,  // parked at collect, roller armed, waiting for press 2
  PH_RETURN    // phase 2 moving: flip height, then low
};
static _Phase _phase = PH_IDLE;

bool macro_waiting() { return _phase == PH_WAITING; }

// Geometric check - is the cascade physically near the collect height.
bool cascade_near_collect() {
  return fabs(cascade_position() - CASCADE_COLLECT) <= CASCADE_COLLECT_TOL;
}

// The upper roller is allowed to spin ONLY while the sequence is parked at
// collect.  Driving through that height with L1/L2 does not arm it.
bool cascade_at_collect() {
  return _phase == PH_WAITING;
}

/////
// The two halves
/////
static void phase1_task(void*) {
  // Preflight: put the pistons into a known state before anything moves, so
  // the sequence behaves the same however they were left.
  flip_set(FLIP_ON);
  claw_set(CLAW_OFF);

  bool ok = macro_wait(MACRO_PISTON_SETTLE, "0 preflight") &&
            step_pause("0 preflight") &&
            cascade_to(CASCADE_FLIP, "1 flip") &&
            step_pause("1 flip");

  // At flip height, release the flip piston, then hold still for
  // CASCADE_FLIP_RELEASE_MS so it can finish moving before the cascade starts
  // back down.
  if (ok) {
    flip_set(FLIP_OFF);
    ok = macro_wait(CASCADE_FLIP_RELEASE_MS, "2 release") && step_pause("2 release");
  }

  if (ok) ok = cascade_to(CASCADE_COLLECT, "3 collect") && step_pause("3 collect");

  cascade_stop();
  _running = false;
  // Only park-and-arm if it actually arrived.  A cancel or stall drops back to
  // idle, so the roller never arms off a failed move.
  _phase  = ok ? PH_WAITING : PH_IDLE;
  _failed = !ok;
  if (ok) _step = "WAITING";
  _cancel = false;
}

// Fired partway through phase 2's rise - see CASCADE_FLIP_DELAY_MS.
static void fire_flip() {
  intake_run(true, true);   // upper roller reverses from here to the end
  flip_set(FLIP_ON);
}

static void phase2_task(void*) {
  // Intake runs for the whole of phase 2, from this press until it is back at
  // low.  _intake_owned stops opcontrol writing zero over the top of it.
  _intake_owned = true;
  intake_run(true);

  // Grip first, then lift.
  claw_set(CLAW_ON);
  bool ok = macro_wait(MACRO_PISTON_SETTLE, "4 claw") && step_pause("4 claw");

  // Rise to the out height, with the flip piston firing partway UP rather than
  // after arriving - CASCADE_FLIP_DELAY_MS into the move.  The upper roller
  // reverses at the same moment and stays reversed until the end.
  if (ok) ok = cascade_to(CASCADE_OUT, "5 out", CASCADE_MOVE_SPEED,
                          CASCADE_FLIP_DELAY_MS, fire_flip) &&
               step_pause("5 out");

  if (ok) ok = cascade_to(CASCADE_LOW, "6 low");

  // Release the intake on every exit path, cancel and stall included.
  intake_run(false);
  _intake_owned = false;

  // Hand the holding job to the other motor after each completed return to low,
  // so the heat of carrying the cascade is shared between them.
  if (ok) cascade_swap_hold_motor();

  cascade_stop();
  _running = false;
  _phase   = PH_IDLE;
  _cancel  = false;
  _failed  = !ok;
  if (ok) _step = "done";
}

// ONE long-lived worker rather than a task per press.  pros::Task has no
// destructor - it is a thin wrapper around a handle - so `delete` freed the
// wrapper while leaving the RTOS task behind, leaking one per press.
enum _Req { REQ_NONE, REQ_PHASE1, REQ_PHASE2 };
static volatile _Req _req = REQ_NONE;

static void macro_worker(void*) {
  while (true) {
    _Req r = _req;
    if (r != REQ_NONE) {
      _req = REQ_NONE;
      if (r == REQ_PHASE1) phase1_task(nullptr);
      else                 phase2_task(nullptr);
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

void macro_start() {
  // Pressing mid-move cancels rather than queueing anything.
  if (_running) { _cancel = true; return; }

  // Created on first use and never destroyed.  `static` inside the function so
  // it cannot run before the motors and sensors are constructed.
  static pros::Task worker(macro_worker, nullptr, "Cascade Macro");

  _running = true;
  _cancel  = false;
  _failed  = false;
  _step    = "starting";
  _req     = (_phase == PH_WAITING) ? REQ_PHASE2 : REQ_PHASE1;
}
