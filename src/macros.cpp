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
  bool ok = cascade_to(CASCADE_FLIP, "1 flip") &&
            cascade_to(CASCADE_COLLECT, "2 collect");

  cascade_stop();
  _running = false;
  // Only park-and-arm if it actually arrived.  A cancel or stall drops back to
  // idle, so the roller never arms off a failed move.
  _phase = ok ? PH_WAITING : PH_IDLE;
  if (ok) _step = "WAITING - press";
  _cancel = false;
}

static void phase2_task(void*) {
  bool ok = cascade_to(CASCADE_FLIP, "3 flip") &&
            cascade_to(CASCADE_LOW,  "4 low");

  // Hand the holding job to the other motor after each completed return to low,
  // so the heat of carrying the cascade is shared between them.
  if (ok) cascade_swap_hold_motor();

  cascade_stop();
  _running = false;
  _phase   = PH_IDLE;
  _cancel  = false;
  if (ok) _step = "done";
}

static void start_task(void (*fn)(void*), const char* name) {
  _running = true;
  _cancel  = false;
  _step    = "starting";
  if (_task != nullptr) { delete _task; _task = nullptr; }
  _task = new pros::Task(fn, nullptr, name);
}

void macro_start() {
  // Pressing mid-move cancels rather than queueing anything.
  if (_running) { _cancel = true; return; }

  if (_phase == PH_WAITING) start_task(phase2_task, "Cascade Return");
  else                      start_task(phase1_task, "Cascade Collect");
}
