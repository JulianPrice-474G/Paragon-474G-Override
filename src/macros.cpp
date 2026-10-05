#include "main.h"

#include <cmath>

/////
// State
/////
static bool        _running     = false;
static bool        _cancel      = false;
static const char* _step        = "idle";
static bool        _failed      = false;  // last run ended STALLED or TIMEOUT

// Where phase 2 finishes, set by macro_press().  Negative means stay at the
// out height.
static double      _phase2_end  = -1;

// A macro_press() with a delay_time, waiting to fire.  See macro_press().
static volatile bool     _pend       = false;
static volatile double   _pend_end   = -1;
static volatile uint32_t _pend_at    = 0;

bool macro_running() { return _running; }
bool macro_failed()  { return _failed; }
void macro_press_pending_clear() { _pend = false; }
void macro_cancel()  { _pend = false; if (_running) _cancel = true; }

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
// action, if given, is called once DURING the move rather than before or after
// it.  Two ways to say when:
//   action_height >= 0  fire when the cascade reaches that height on the way
//   otherwise           fire action_delay_ms after the move starts
// Either way it fires on arrival at the latest, so it is never lost.
static bool cascade_to(double target, const char* step_name,
                       int max_speed = CASCADE_MOVE_SPEED,
                       int action_delay_ms = -1, void (*action)() = nullptr,
                       double action_height = -1) {
  _step = step_name;

  const uint32_t start        = pros::millis();
  uint32_t       last_progress = start;
  double         last_pos      = cascade_position();
  bool           action_fired  = false;

  // Downward moves get their own speed - gravity and the holding brake make a
  // descent behave differently from a lift at the same power.
  const bool going_up = (target >= last_pos);
  if (!going_up) max_speed = CASCADE_DOWN_SPEED;

  while (pros::millis() - start < (uint32_t)CASCADE_MOVE_TIMEOUT) {
    if (_cancel) { cascade_stop(); return false; }

    double pos = cascade_position();
    double err = target - pos;

    // Fire the action once, at its height or its time.
    if (action && !action_fired) {
      bool due;
      if (action_height >= 0)
        due = going_up ? (pos >= action_height) : (pos <= action_height);
      else
        due = action_delay_ms >= 0 &&
              (int)(pros::millis() - start) >= action_delay_ms;
      if (due) { action_fired = true; action(); }
    }

    // Progress tracking, used both for "close enough" and the stall guard.
    if (fabs(pos - last_pos) >= CASCADE_STALL_DEG) {
      last_pos      = pos;
      last_progress = pros::millis();
    }
    const uint32_t still_for = pros::millis() - last_progress;

    // Arrived: inside tolerance, or stopped moving close enough to it.  Going
    // up, the eased-off power near the top can leave the cascade a few degrees
    // short; without this it sat there until the timeout and the macro failed
    // before its next step.
    bool close_and_stuck = fabs(err) <= CASCADE_CLOSE_ENOUGH &&
                           still_for > (uint32_t)CASCADE_CLOSE_STALL_MS;

    if (fabs(err) <= CASCADE_MOVE_TOL || close_and_stuck) {
      // Brake rather than coast, then keep watching: a heavy cascade carries a
      // long way past the target on momentum, and the old code stopped looking
      // the instant it touched the target so it never pulled the overshoot back.
      cascade_hold();
      const uint32_t settle_start = pros::millis();
      uint32_t       steady_since = pros::millis();
      // Arrived by stalling close?  Then it has already shown it can't make
      // the last few degrees - skip the settle, which would only push weakly
      // at the same spot and delay the next step.
      while (!close_and_stuck &&
             pros::millis() - settle_start < (uint32_t)CASCADE_SETTLE_MS) {
        if (_cancel) { cascade_stop(); return false; }
        double e = target - cascade_position();
        if (fabs(e) > CASCADE_MOVE_TOL) {
          int p = (e > 0) ? CASCADE_SETTLE_POWER : -CASCADE_SETTLE_POWER;
          cascade_drive(p * CASCADE_RAISE_SIGN);
          steady_since = pros::millis();   // drifted out - start the clock again
        } else {
          cascade_hold();
          // Steady inside tolerance for long enough: stop waiting.  A move that
          // lands cleanly should not cost the full CASCADE_SETTLE_MS.
          if (pros::millis() - steady_since >= (uint32_t)CASCADE_SETTLE_STABLE_MS)
            break;
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
    if (fabs(err) > CASCADE_MOVE_SLOW && still_for > (uint32_t)CASCADE_STALL_MS) {
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
    const int floor_power = (err > 0) ? CASCADE_MOVE_MIN_UP : CASCADE_MOVE_MIN;
    if (power < floor_power) power = floor_power;
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

// Intake, in the R1 direction - the same way R1 spins it manually.  opcontrol
// skips its own R1/R2 block while _intake_owned is set.
static bool _intake_owned = false;
bool macro_owns_intake() { return _intake_owned; }

// roller_back reverses ONLY the upper roller (port 19); the fins and dropdown
// carry on in the normal direction.
static void intake_run(bool on, bool roller_back = false) {
  int p = on ? MACRO_INTAKE_SPEED : 0;
  fins_set(p);
  dropdown.move(-p);
  upper_roller.move(roller_back ? p : -p);
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
  // An end height only means anything on the SECOND press.  Clear whatever the
  // first press passed, so it cannot leak into a later phase 2.
  _phase2_end = -1;

  // Preflight: flip piston to a known state before anything moves.
  flip_set(FLIP_ON);
  bool ok = macro_wait(MACRO_PISTON_SETTLE, "0 preflight") &&
            step_pause("0 preflight");

  // Up to the high point first, and only act once it has ARRIVED.
  if (ok) ok = cascade_to(CASCADE_FLIP, "1 up") && step_pause("1 up");

  // At the top: open the claw and drop the flip piston together.
  if (ok) {
    claw_set(CLAW_OFF);
    flip_set(FLIP_OFF);
    ok = macro_wait(CASCADE_DROP_DELAY_MS, "2 drop") && step_pause("2 drop");
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
static uint32_t _flip_fired_ms = 0;   // when the flip piston last extended

static void fire_flip() {
  intake_run(true, true);   // upper roller reverses from here to the end
  flip_set(FLIP_ON);
  _flip_fired_ms = pros::millis();
}

static void phase2_task(void*) {
  // Intake runs for the whole of phase 2, from this press until it is back at
  // low.  _intake_owned stops opcontrol writing zero over the top of it.
  _intake_owned = true;

  // Take the intake over from any timed spin an auton left running
  // (intake_spin(-1, ...) in particular).  Otherwise the spin's background task
  // keeps writing the motors every tick and overrides the upper roller reversing
  // for the flip.  The short wait lets that task make its final zeroing pass
  // BEFORE we switch the intake on, not after.
  intake_spin_stop();
  pros::delay(20);
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

  // Hold before descending, so the flip piston has CASCADE_AFTER_FLIP_MS from
  // the moment it fired.  Measured from the flip rather than from arriving at
  // the top, so an early flip has already used some of it up.
  if (ok) {
    int elapsed = (int)(pros::millis() - _flip_fired_ms);
    int remain  = CASCADE_AFTER_FLIP_MS - elapsed;
    if (remain > 0) ok = macro_wait(remain, "6 after flip") && step_pause("6 after flip");
  }

  // Finish wherever macro_press() asked for.  Negative - the default - leaves
  // the cascade at the out height, and it is brought down with L1/L2 or a
  // cascade_move_async() in an auton.
  if (ok && _phase2_end >= 0)
    ok = cascade_to(_phase2_end, "7 end") && step_pause("7 end");

  // Used once.  Clearing it means a later RIGHT press in driver control - which
  // passes no height - goes back to staying at the out height.
  _phase2_end = -1;

  // Release the intake on every exit path, cancel and stall included.
  intake_run(false);
  _intake_owned = false;

  // Hand the holding job to the other motor after each completed run, so the
  // heat of carrying a raised cascade is shared between them.
  if (ok) cascade_swap_hold_motor();

  // Hold rather than coast: the cascade is left raised, so letting it free-wheel
  // would drop it.  cascade_stop() would only zero the voltage.
  cascade_hold();
  _running = false;
  _phase   = PH_IDLE;
  _cancel  = false;
  _failed  = !ok;
  if (ok) _step = "done";
}

// ONE long-lived worker rather than a task per press.  pros::Task has no
// destructor - it is a thin wrapper around a handle - so `delete` freed the
// wrapper while leaving the RTOS task behind, leaking one per press.
enum _Req { REQ_NONE, REQ_PHASE1, REQ_PHASE2, REQ_MOVE };
static void move_task();
static volatile _Req _req = REQ_NONE;

// ── Background cascade move ───────────────────────────────────────────────
// cascade_move_async() runs one cascade_to() on the macro's own worker, so the
// cascade travels while the auton gets on with driving.  Sharing the worker is
// deliberate: a move and a macro phase can never drive the cascade at once.
static double _move_target = 0;
static int    _move_speed  = 0;

static void move_task() {
  bool ok = cascade_to(_move_target, "moving", _move_speed);
  cascade_stop();
  _running = false;
  _cancel  = false;
  _failed  = !ok;
  if (ok) _step = "done";
}

static void macro_worker(void*) {
  while (true) {
    _Req r = _req;
    if (r != REQ_NONE) {
      _req = REQ_NONE;
      if      (r == REQ_PHASE1) phase1_task(nullptr);
      else if (r == REQ_PHASE2) phase2_task(nullptr);
      else                      move_task();
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

// Start the ONE worker.  macro_start() and cascade_move_async() both go
// through this - if each had its own `static pros::Task`, they would be two
// separate objects, and a run that used both would end up with two workers
// racing for the same request and running a phase twice at once.
static void ensure_worker() {
  static pros::Task worker(macro_worker, nullptr, "Cascade Macro");
}

void macro_start() {
  // Pressing mid-move cancels rather than queueing anything.
  if (_running) { _cancel = true; return; }

  ensure_worker();

  _running = true;
  _cancel  = false;
  _failed  = false;
  _step    = "starting";
  _req     = (_phase == PH_WAITING) ? REQ_PHASE2 : REQ_PHASE1;
}

/////
// Public: background cascade moves, for autons
/////
void cascade_move_async(double target, int speed) {
  if (_running) return;          // a macro phase or another move owns it

  ensure_worker();

  _move_target = target;
  _move_speed  = (speed > 0) ? speed : CASCADE_MOVE_SPEED;
  _running     = true;
  _cancel      = false;
  _failed      = false;
  _step        = "moving";
  _req         = REQ_MOVE;
}

bool cascade_move_active() { return _running; }

bool cascade_move_wait(int timeout_ms) {
  const uint32_t start = pros::millis();
  while (_running) {
    if ((int)(pros::millis() - start) > timeout_ms) return false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return !_failed;
}

// For autons.  Unlike RIGHT in driver control - where a press mid-move just
// cancels - this TAKES OVER: whatever is driving the cascade (a
// cascade_move_async() or an unfinished macro step) is cancelled and the macro
// starts straight away.  So a route can fire off a background move and press
// the macro later without first checking the move has finished.
static void macro_press_now(double end_height) {
  if (_running) {
    _cancel = true;
    // Every loop that drives the cascade checks _cancel each tick, so this is
    // one or two ticks.  The cap is only a backstop.
    const uint32_t start = pros::millis();
    while (_running && pros::millis() - start < 500) pros::delay(ez::util::DELAY_TIME);
  }

  // Only the SECOND press uses it, but storing it on every press means a
  // cancelled run cannot leave a stale target behind for the next one.
  _phase2_end = end_height;
  macro_start();
}

// Fires a delayed press when its time comes.  Its own long-lived task, so the
// takeover wait in macro_press_now() never holds up the auton.
static void macro_delay_task(void*) {
  while (true) {
    if (_pend && pros::millis() >= _pend_at) {
      _pend = false;
      macro_press_now(_pend_end);
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

void macro_press(double end_height, int delay_time) {
  _pend = false;   // a new press replaces one still waiting

  if (delay_time <= 0) {
    macro_press_now(end_height);
    return;
  }

  static pros::Task worker(macro_delay_task, nullptr, "Macro Delay");
  _pend_end = end_height;
  _pend_at  = pros::millis() + delay_time;
  _pend     = true;
}

bool macro_wait_done(int timeout_ms) {
  const uint32_t start = pros::millis();
  while (_running || _pend) {
    if ((int)(pros::millis() - start) > timeout_ms) return false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return !_failed;
}
