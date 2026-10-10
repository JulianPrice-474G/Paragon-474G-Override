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

// Set by one_pin_macro() for its press 2: the fins and dropdown run in the
// OUTTAKE direction the whole time.  The upper roller does its usual
// forward-then-reverse-at-flip.  Cleared after every phase 2.
static bool        _phase2_outtake = false;

// A macro_press() with a delay_time, waiting to fire.  See macro_press().
static volatile bool     _pend       = false;
static volatile double   _pend_end   = -1;
static volatile uint32_t _pend_at    = 0;
static volatile bool     _firing     = false;  // the delay task is pressing it now

// Requests that arrive while the cascade is busy wait their turn instead of
// breaking what is running.  The worker starts them as soon as it is free -
// see start_queued().  One slot each; a newer request replaces an older one.
static volatile bool     _move_queued      = false;   // a cascade_move_async()
static volatile double   _queued_target    = 0;
static volatile int      _queued_speed     = 0;
static volatile bool     _press_queued     = false;   // a macro_press()
static volatile double   _press_queued_end = -1;
static volatile bool     _one_pin_queued   = false;   // a one_pin_macro()

// one_pin_macro()'s times for the run about to start.
static double _one_pin_end      = -1;
static int    _one_pin_grab_ms  = 0;
static int    _one_pin_rev_ms   = 0;

// True while the worker runs a plain cascade_move_async() rather than a macro
// phase.  A macro press takes over a move, but waits for a phase.
static volatile bool     _running_move     = false;

// Guards the "is the cascade free? then start something" decisions, so the
// auton, the delay task and the worker cannot both start at once.  It pauses
// the task scheduler rather than taking a mutex: when auton ends, the auton
// task is deleted wherever it is (PROS does this in a match, and so does the
// auton timer), and a mutex it held then would stay locked for good and
// freeze the macro.  Nothing can be deleted inside a paused-scheduler section.
// Keep these sections tiny - no delays, no device calls.
extern "C" {
void    rtos_suspend_all(void);   // PROS kernel (kapi.h), not in the public headers
int32_t rtos_resume_all(void);
}
struct _StartLock {
  _StartLock()  { rtos_suspend_all(); }
  ~_StartLock() { rtos_resume_all(); }
};

// A plain cascade move is taken over by a macro press: ask it to stop, and
// wait for it.  No lock is held while waiting, for the reason above.
static void take_over_move() {
  if (!(_running && _running_move)) return;
  _cancel = true;
  // Every loop that drives the cascade checks _cancel each tick, so this is
  // one or two ticks.  The cap is only a backstop.
  const uint32_t start = pros::millis();
  while (_running && _running_move && pros::millis() - start < 500)
    pros::delay(ez::util::DELAY_TIME);
}

// True while an auton runs.  Picks which macro settings are used.
static volatile bool     _in_auton = false;
void macro_in_auton(bool on) { _in_auton = on; }

// The macro settings for the mode the robot is in: AUTO_ in an auton, DRIVER_
// otherwise (see the top of main.cpp).  Each phase reads them once as it
// starts, so a press keeps one set the whole way through.
struct MacroSettings {
  double collect, flip;
  int    drop_ms;
  bool   custom_on;
  double custom_flip;
  int    custom_drop_ms;
  double out;
  int    flip_delay_ms, after_flip_ms, down_speed;
};

static MacroSettings macro_settings() {
  if (_in_auton)
    return {AUTO_CASCADE_COLLECT, AUTO_CASCADE_FLIP, AUTO_DROP_DELAY_MS,
            AUTO_CUSTOM_FLIP_ON, AUTO_CUSTOM_FLIP, AUTO_CUSTOM_DROP_DELAY_MS,
            AUTO_CASCADE_OUT, AUTO_FLIP_DELAY_MS, AUTO_AFTER_FLIP_MS,
            AUTO_DOWN_SPEED};
  return {DRIVER_CASCADE_COLLECT, DRIVER_CASCADE_FLIP, DRIVER_DROP_DELAY_MS,
          DRIVER_CUSTOM_FLIP_ON, DRIVER_CUSTOM_FLIP, DRIVER_CUSTOM_DROP_DELAY_MS,
          DRIVER_CASCADE_OUT, DRIVER_FLIP_DELAY_MS, DRIVER_AFTER_FLIP_MS,
          DRIVER_DOWN_SPEED};
}

bool macro_running() { return _running; }
bool macro_failed()  { return _failed; }
void macro_press_pending_clear() {
  _pend = false; _move_queued = false; _press_queued = false; _one_pin_queued = false;
}
void macro_cancel() {
  _pend = false;
  _move_queued    = false;
  _press_queued   = false;
  _one_pin_queued = false;
  if (_running) _cancel = true;
}

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
// exact_speed: drive at max_speed and never faster, up or down - no
// ..._DOWN_SPEED swap and no floor above it.  For cascade_move_async()
// given a speed.  The macro's own moves leave it false.
static bool cascade_to(double target, const char* step_name,
                       int max_speed = CASCADE_MOVE_SPEED,
                       int action_delay_ms = -1, void (*action)() = nullptr,
                       double action_height = -1, bool exact_speed = false) {
  _step = step_name;

  const uint32_t start        = pros::millis();
  uint32_t       last_progress = start;
  double         last_pos      = cascade_position();
  bool           action_fired  = false;

  // Downward moves get their own speed - gravity and the holding brake make a
  // descent behave differently from a lift at the same power.
  const bool going_up = (target >= last_pos);
  if (!going_up && !exact_speed) max_speed = macro_settings().down_speed;

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
          int sp = (exact_speed && max_speed < CASCADE_SETTLE_POWER) ? max_speed
                                                                    : CASCADE_SETTLE_POWER;
          int p = (e > 0) ? sp : -sp;
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
    int floor_power = (err > 0) ? CASCADE_MOVE_MIN_UP : CASCADE_MOVE_MIN;
    if (exact_speed && floor_power > max_speed) floor_power = max_speed;
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
// carry on in the normal direction - or in the outtake direction throughout,
// during one_pin_macro()'s press 2 (_phase2_outtake).
static void intake_run(bool on, bool roller_back = false) {
  int p = on ? MACRO_INTAKE_SPEED : 0;
  int f = _phase2_outtake ? -p : p;   // fins + dropdown
  fins_set(f);
  dropdown.move(-f);
  upper_roller.move(roller_back ? p : -p);
}

// All four intake motors at one power, upper roller included.  Positive runs
// them the way intake_run(true) does; negative runs every one of them backward.
static void intake_all(int p) {
  fins_set(p);
  dropdown.move(-p);
  upper_roller.move(-p);
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
  return fabs(cascade_position() - macro_settings().collect) <= CASCADE_COLLECT_TOL;
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

  // This mode's settings.  The custom flip pair replaces the normal flip
  // height and drop wait when this mode's ..._CUSTOM_FLIP_ON switch is on.
  const MacroSettings S = macro_settings();
  const bool   custom  = S.custom_on;
  const double flip_h  = custom ? S.custom_flip    : S.flip;
  const int    drop_ms = custom ? S.custom_drop_ms : S.drop_ms;

  // Preflight: flip piston to a known state before anything moves.
  flip_set(FLIP_ON);
  bool ok = macro_wait(MACRO_PISTON_SETTLE, "0 preflight") &&
            step_pause("0 preflight");

  // Up to the high point first, and only act once it has ARRIVED.
  if (ok) ok = cascade_to(flip_h, custom ? "1 up custom" : "1 up") && step_pause("1 up");

  // At the top: open the claw and drop the flip piston together.
  if (ok) {
    claw_set(CLAW_OFF);
    flip_set(FLIP_OFF);
    ok = macro_wait(drop_ms, "2 drop") && step_pause("2 drop");
  }

  if (ok) ok = cascade_to(S.collect, "3 collect") && step_pause("3 collect");

  cascade_stop();
  // Only park-and-arm if it actually arrived.  A cancel or stall drops back to
  // idle, so the roller never arms off a failed move.
  _phase  = ok ? PH_WAITING : PH_IDLE;
  _failed = !ok;
  if (ok) _step = "WAITING";
  _cancel = false;
  // Last, so a press that sees the cascade free also sees the phase it left.
  _running = false;
}

// Fired partway through phase 2's rise - see ..._FLIP_DELAY_MS in main.cpp.
static uint32_t _flip_fired_ms = 0;   // when the flip piston last extended

static void fire_flip() {
  intake_run(true, true);   // upper roller reverses from here to the end
  flip_set(FLIP_ON);
  _flip_fired_ms = pros::millis();
}

static void phase2_task(void*) {
  const MacroSettings S = macro_settings();   // this mode's settings

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
  // after arriving - S.flip_delay_ms into the move.  The upper roller
  // reverses at the same moment and stays reversed until the end.
  if (ok) ok = cascade_to(S.out, "5 out", CASCADE_MOVE_SPEED,
                          S.flip_delay_ms, fire_flip) &&
               step_pause("5 out");

  // Hold before descending, so the flip piston has S.after_flip_ms from
  // the moment it fired.  Measured from the flip rather than from arriving at
  // the top, so an early flip has already used some of it up.
  if (ok) {
    int elapsed = (int)(pros::millis() - _flip_fired_ms);
    int remain  = S.after_flip_ms - elapsed;
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
  _phase2_outtake = false;   // only ever for the one press 2 that set it

  // Hand the holding job to the other motor after each completed run, so the
  // heat of carrying a raised cascade is shared between them.
  if (ok) cascade_swap_hold_motor();

  // Hold rather than coast: the cascade is left raised, so letting it free-wheel
  // would drop it.  cascade_stop() would only zero the voltage.
  cascade_hold();
  _phase   = PH_IDLE;
  _cancel  = false;
  _failed  = !ok;
  if (ok) _step = "done";
  _running = false;   // last - see phase1_task()
}

// macro_grab() (X in driver control): while parked after press 1, all four
// intake motors backward, then after GRAB_REV_MS close the claw.  The intake
// stays backward - _intake_owned keeps opcontrol off it - until press 2 takes
// the intake over.  Its own small task, NOT the cascade worker: the cascade is
// not touched, and RIGHT must still be able to start press 2 at any moment.
static volatile bool _grab_req = false;

static void grab_task(void*) {
  while (true) {
    if (_grab_req) {
      _grab_req = false;
      _intake_owned = true;
      intake_all(-GRAB_SPEED);
      pros::delay(GRAB_REV_MS);
      // Press 2 may have started meanwhile - it grips the claw itself.
      if (_phase == PH_WAITING && !_running) claw_set(CLAW_ON);
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

bool macro_grab() {
  if (_running || _phase != PH_WAITING) return false;   // only while parked
  static pros::Task worker(grab_task, nullptr, "Macro Grab");
  _grab_req = true;
  return true;
}

// one_pin_macro(): intakes forward, claw, intakes backward, then press 2.
// Only ever started from PH_WAITING - see one_pin_macro().
static void one_pin_task() {
  _intake_owned = true;
  intake_spin_stop();   // take the intake from any timed spin, as phase 2 does
  pros::delay(20);

  // The cup, exactly as X does it (macro_grab): all four backward, then the
  // claw closes while they keep going backward.
  intake_all(-GRAB_SPEED);
  bool ok = macro_wait(_one_pin_grab_ms, "1pin grab");

  if (ok) {
    claw_set(CLAW_ON);
    ok = macro_wait(_one_pin_rev_ms, "1pin rev");
  }

  if (ok) {
    // Press 2 as normal.  It takes the intake itself and cleans up after.
    // ... except the fins and dropdown outtake the whole time; the upper
    // roller still goes forward, then reverses at the flip.
    _phase2_end     = _one_pin_end;
    _phase2_outtake = true;
    phase2_task(nullptr);
    return;
  }

  // Cancelled before press 2.  Still parked, so a later press can carry on.
  intake_run(false);
  _intake_owned = false;
  cascade_hold();
  _cancel  = false;
  _failed  = true;
  _running = false;   // last - see phase1_task()
}

// ONE long-lived worker rather than a task per press.  pros::Task has no
// destructor - it is a thin wrapper around a handle - so `delete` freed the
// wrapper while leaving the RTOS task behind, leaking one per press.
enum _Req { REQ_NONE, REQ_PHASE1, REQ_PHASE2, REQ_MOVE, REQ_ONE_PIN };
static void move_task();
static volatile _Req _req = REQ_NONE;

// ── Background cascade move ───────────────────────────────────────────────
// cascade_move_async() runs one cascade_to() on the macro's own worker, so the
// cascade travels while the auton gets on with driving.  Sharing the worker is
// deliberate: a move and a macro phase can never drive the cascade at once.
static double _move_target = 0;
static int    _move_speed  = 0;

static void move_task() {
  // A speed given to cascade_move_async() is used exactly; 0 = the normal
  // macro behaviour at CASCADE_MOVE_SPEED.
  const bool exact = _move_speed > 0;
  bool ok = cascade_to(_move_target, "moving",
                       exact ? _move_speed : CASCADE_MOVE_SPEED,
                       -1, nullptr, -1, exact);
  cascade_stop();
  _cancel  = false;
  _failed  = !ok;
  if (ok) _step = "done";
  _running_move = false;
  _running = false;   // last - see phase1_task()
}

// Hand a move to the worker.  Only call with the cascade free.
static void move_start(double target, int speed) {
  _move_target  = target;
  _move_speed   = speed;
  _running      = true;
  _running_move = true;
  _cancel       = false;
  _failed       = false;
  _step         = "moving";
  _req          = REQ_MOVE;
}

// Start whatever waited for the step that just finished.  A press goes first.
// A move also waits for a delayed press still to fire, so it runs after that
// press's macro step - the order the auton asked for them.
static void start_queued() {
  _StartLock lock;
  if (!_running) {
    if (_one_pin_queued) {
      _one_pin_queued = false;
      if (_phase == PH_WAITING) {
        _running = true; _running_move = false; _cancel = false; _failed = false;
        _step = "1pin";
        _req  = REQ_ONE_PIN;
      } else {
        _failed = true;   // press 1 did not park - nothing to do the pin from
        _step   = "1pin: not parked";
      }
    } else if (_press_queued) {
      _press_queued = false;
      _phase2_end   = _press_queued_end;
      macro_start();
    } else if (_move_queued && !_pend && !_firing) {
      _move_queued = false;
      move_start(_queued_target, _queued_speed);
    }
  }
}

static void macro_worker(void*) {
  while (true) {
    _Req r = _req;
    if (r != REQ_NONE) {
      _req = REQ_NONE;
      if      (r == REQ_PHASE1)  phase1_task(nullptr);
      else if (r == REQ_PHASE2)  phase2_task(nullptr);
      else if (r == REQ_ONE_PIN) one_pin_task();
      else                       move_task();
    } else if (!_running && (_press_queued || _move_queued || _one_pin_queued)) {
      start_queued();
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

  _running      = true;
  _running_move = false;
  _cancel  = false;
  _failed  = false;
  _step    = "starting";
  _req     = (_phase == PH_WAITING) ? REQ_PHASE2 : REQ_PHASE1;
}

/////
// Public: background cascade moves, for autons
/////
void cascade_move_async(double target, int speed) {
  ensure_worker();
  const int s = (speed > 0) ? speed : 0;   // 0 = default; see move_task()

  _StartLock lock;
  if (_running || _pend || _firing || _press_queued || _one_pin_queued) {
    // Busy, or a press is due first: wait for it, then go.  Dropping it
    // instead left the cascade wherever the macro finished.
    _queued_target = target;
    _queued_speed  = s;
    _move_queued   = true;
  } else {
    move_start(target, s);
  }
}

bool cascade_move_active() { return _running || _move_queued; }

bool cascade_move_wait(int timeout_ms) {
  const uint32_t start = pros::millis();
  while (_running || _move_queued) {
    if ((int)(pros::millis() - start) > timeout_ms) return false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return !_failed;
}

// For autons.  Unlike RIGHT in driver control - where a press mid-move just
// cancels - this never cancels a macro step:
//  - a cascade_move_async() still running is TAKEN OVER: cancelled, and the
//    macro starts straight away.  So a route can fire off a background move
//    and press the macro later without first checking the move has finished.
//  - an unfinished macro step is WAITED FOR: the press runs once it is done.
//    Cancelling it instead dropped press 1 back to idle, so the "second" press
//    started press 1 over again.
static void macro_press_now(double end_height) {
  ensure_worker();   // here, not inside the lock - it may create the task
  take_over_move();

  _StartLock lock;
  if (_running) {
    // A macro step is still moving.  The worker presses once it finishes.
    _press_queued_end = end_height;
    _press_queued     = true;
  } else {
    // Only the SECOND press uses it, but storing it on every press means a
    // cancelled run cannot leave a stale target behind for the next one.
    _phase2_end = end_height;
    macro_start();
  }
}

// Fires a delayed press when its time comes.  Its own long-lived task, so the
// takeover wait in macro_press_now() never holds up the auton.
static void macro_delay_task(void*) {
  while (true) {
    if (_pend && pros::millis() >= _pend_at) {
      // _firing covers the gap between clearing _pend and the press starting,
      // so a queued move cannot slip in ahead of the press it waits for.
      _firing = true;
      _pend   = false;
      macro_press_now(_pend_end);
      _firing = false;
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}

void macro_press(double end_height, int delay_time) {
  _pend = false;   // a new press replaces one still waiting
  // ... and a move still waiting its turn.  A move asked for AFTER this call
  // stays queued and runs once this press's step is done.
  _move_queued = false;

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
  while (_running || _pend || _firing || _press_queued || _one_pin_queued) {
    if ((int)(pros::millis() - start) > timeout_ms) return false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return !_failed;
}

bool one_pin_macro(double end_height, int grab_ms, int rev_ms) {
  ensure_worker();
  _pend = false;           // replaces a delayed press still waiting, like macro_press()
  _move_queued = false;

  // A plain cascade move is taken over, as macro_press() does.
  take_over_move();

  _StartLock lock;
  _one_pin_end     = end_height;
  _one_pin_grab_ms = grab_ms;
  _one_pin_rev_ms  = rev_ms;

  bool started = true;
  if (_running) {
    _one_pin_queued = true;            // press 1 still going - run once it parks
  } else if (_phase == PH_WAITING) {
    _running = true; _running_move = false; _cancel = false; _failed = false;
    _step = "1pin";
    _req  = REQ_ONE_PIN;
  } else {
    _step   = "1pin: not parked";      // not after press 1 - do nothing
    started = false;
  }
  return started;
}
