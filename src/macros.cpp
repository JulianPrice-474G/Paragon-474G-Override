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

/////
// Is the cascade at the collect height?
/////
bool cascade_at_collect() {
  return fabs(cascade_position() - CASCADE_COLLECT) <= CASCADE_COLLECT_TOL;
}

/////
// The move
/////
// RIGHT toggles: at collect -> go to low, anywhere else -> go to collect.
// The target is chosen from the cascade's ACTUAL position, so moving it by
// hand with L1/L2 cannot leave the toggle pointing the wrong way.
static double _target = CASCADE_LOW;

static void macro_task_fn(void*) {
  bool arrived = cascade_to(_target,
                            _target == CASCADE_COLLECT ? "-> collect" : "-> low");

  // Back at low, hand the holding job to the other motor.  Only ONE motor holds
  // the cascade, so that one carries the whole load and is the one that heats
  // up; alternating spreads it.  Only on a completed trip to low - swapping
  // after a cancel or a stall would change the holder mid-air.
  if (arrived && _target == CASCADE_LOW) cascade_swap_hold_motor();

  cascade_stop();
  _running = false;
  _cancel  = false;
  if (_step[0] != 'S' && _step[0] != 'T') _step = "done";
}

void macro_start() {
  if (_running) return;
  _target  = cascade_at_collect() ? CASCADE_LOW : CASCADE_COLLECT;
  _running = true;
  _cancel  = false;
  _step    = "starting";

  // Reuse one task object rather than leaking a new one per run.
  if (_task != nullptr) { delete _task; _task = nullptr; }
  _task = new pros::Task(macro_task_fn, nullptr, "Cascade Move");
}
