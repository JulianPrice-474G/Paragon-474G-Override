#include "main.h"

#include <cmath>

/////
// State
/////
static pros::Task* _task        = nullptr;
static bool        _running     = false;
static bool        _cancel      = false;
static const char* _step        = "idle";

bool macro_running() { return _running; }
void macro_cancel()  { if (_running) _cancel = true; }

const char* macro_status_text() {
  static char buf[20];
  snprintf(buf, sizeof(buf), "%-19s", _running ? _step : "ready");
  return buf;
}

/////
// Subsystem helpers
/////
static void cascade_stop() {
  l_motor_a.move(0);
  l_motor_b.move(0);
}

// The cascade pair always runs opposite each other, exactly as L1/L2 drive it.
static void cascade_drive(int power) {
  l_motor_a.move(power);
  l_motor_b.move(-power);
}

// Intake in the R2 direction - the one that pulls game objects in.
static void intake_run(bool on) {
  int p = on ? MACRO_INTAKE_SPEED : 0;
  bool port1_enabled = !(high_intake_extended && middle_intake_extended);
  r_motor_a.move(p);
  r_motor_b.move(port1_enabled ? p : 0);
  r_motor_c.move(p);
  r_motor_d.move(-p);
}

/////
// Move the cascade to a height
/////
// Returns false if it was cancelled, timed out, or stalled.  Always leaves the
// cascade stopped, so a failure cannot leave a motor driving.
static bool cascade_to(double target, const char* step_name) {
  _step = step_name;

  const uint32_t start        = pros::millis();
  uint32_t       last_progress = start;
  double         last_pos      = cascade_position();

  while (pros::millis() - start < (uint32_t)CASCADE_MOVE_TIMEOUT) {
    if (_cancel) { cascade_stop(); return false; }

    double pos = cascade_position();
    double err = target - pos;

    if (fabs(err) <= CASCADE_MOVE_TOL) {
      cascade_stop();
      return true;
    }

    // Stall guard.  Without this, an inverted CASCADE_RAISE_SIGN drives into a
    // hard stop at full power for the whole timeout.
    if (fabs(pos - last_pos) >= CASCADE_STALL_DEG) {
      last_pos      = pos;
      last_progress = pros::millis();
    } else if (pros::millis() - last_progress > (uint32_t)CASCADE_STALL_MS) {
      cascade_stop();
      _step = "STALLED";
      return false;
    }

    // Full speed until CASCADE_MOVE_SLOW degrees out, then ease off so it does
    // not overshoot and oscillate around the target.
    double scale = fabs(err) / CASCADE_MOVE_SLOW;
    if (scale > 1.0) scale = 1.0;
    int power = (int)(CASCADE_MOVE_SPEED * scale);
    if (power < CASCADE_MOVE_MIN) power = CASCADE_MOVE_MIN;
    if (err < 0) power = -power;

    cascade_drive(power * CASCADE_RAISE_SIGN);
    pros::delay(ez::util::DELAY_TIME);
  }

  cascade_stop();
  _step = "TIMEOUT";
  return false;
}

// Interruptible delay - a plain pros::delay() would ignore a cancel request for
// its whole duration.
static bool macro_wait(int ms, const char* step_name) {
  _step = step_name;
  const uint32_t start = pros::millis();
  while (pros::millis() - start < (uint32_t)ms) {
    if (_cancel) return false;
    pros::delay(ez::util::DELAY_TIME);
  }
  return true;
}

static void set_c_flip(bool out) {
  c_flip_extended = out;
  c_flip.set_value(out);
}

static void set_claw(bool out) {
  claw_extended = out;
  claw.set_value(out);
}

/////
// The sequence
/////
static void macro_task_fn(void*) {
  // Any early return lands on the cleanup at the bottom, so there is one exit
  // path and the cascade and intake always end up released.
  do {
    if (!cascade_to(CASCADE_LOW,     "1 low"))      break;
    if (!cascade_to(CASCADE_FLIP,    "2 flip up"))  break;

    set_c_flip(!c_flip_extended);
    if (!macro_wait(MACRO_PISTON_SETTLE, "3 flip"))  break;

    if (!cascade_to(CASCADE_COLLECT, "4 collect"))  break;

    // Intake runs for a fixed time at collect height.
    _step = "5 intake";
    intake_run(true);
    bool ok = macro_wait(MACRO_INTAKE_MS, "5 intake");
    intake_run(false);
    if (!ok) break;

    set_claw(!claw_extended);
    if (!macro_wait(MACRO_CLAW_SETTLE, "6 claw")) break;

    // Intake keeps running while the cascade lifts, so anything still being
    // pulled in does not drop on the way up.
    _step = "7 lift+intake";
    intake_run(true);
    ok = cascade_to(CASCADE_FLIP, "7 lift+intake");
    intake_run(false);
    if (!ok) break;

    set_c_flip(!c_flip_extended);
    if (!macro_wait(MACRO_PISTON_SETTLE, "8 flip")) break;

    cascade_to(CASCADE_LOW, "9 lower");
  } while (false);

  // Release everything we own.  Pistons are left where they got to - stopping
  // mid-sequence should not fling the claw open with a game object in it.
  cascade_stop();
  intake_run(false);
  _running = false;
  _cancel  = false;
  if (_step[0] != 'S' && _step[0] != 'T') _step = "done";
}

void macro_start() {
  if (_running) return;
  _running = true;
  _cancel  = false;
  _step    = "starting";

  // Reuse one task object rather than leaking a new one per run.
  if (_task != nullptr) { delete _task; _task = nullptr; }
  _task = new pros::Task(macro_task_fn, nullptr, "Cascade Macro");
}
