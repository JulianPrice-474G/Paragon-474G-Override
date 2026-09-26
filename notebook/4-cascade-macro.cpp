// ============================================================================
//  4. Cascade macro
//
//  One button (RIGHT) runs the collect-and-flip sequence in two presses.
//
//  First press:
//    1. set the pistons to a known starting state (flip on, claw open)
//    2. raise to FLIP height, letting the flip piston go partway up
//    3. lower to COLLECT height and park there
//    While parked, the upper roller is allowed to run and L1/L2 are locked.
//
//  Second press:
//    1. start the intake and close the claw
//    2. raise to OUT height, firing the flip piston partway up and reversing
//       the upper roller at the same moment
//    3. stay at OUT (or go to a height given by an auton)
//
//  The macro waits at the collect height between the two presses, so the
//  driver decides when to go on to the second half.
// ============================================================================


// ---- Tuning -----------------------------------------------------------------
constexpr int    CASCADE_MOVE_SPEED   = 90;    // normal power for a move
constexpr double CASCADE_MOVE_TOL     = 5;     // degrees; close enough to stop
constexpr double CASCADE_MOVE_SLOW    = 50;    // degrees out, start slowing down
constexpr int    CASCADE_MOVE_MIN     = 35;    // never ask for less than this
constexpr int    CASCADE_MOVE_TIMEOUT = 3000;  // ms before a move gives up

constexpr int    CASCADE_SETTLE_MS        = 400;  // longest we'll wait at the target
constexpr int    CASCADE_SETTLE_STABLE_MS = 80;   // how long it must sit still
constexpr int    CASCADE_SETTLE_POWER     = 30;   // gentle push back if it drifts

constexpr double CASCADE_STALL_DEG = 2;     // must move this far...
constexpr int    CASCADE_STALL_MS  = 600;   // ...in this long, or it's stuck

constexpr int MACRO_PISTON_SETTLE = 300;    // ms for a cylinder to finish moving
constexpr int MACRO_INTAKE_SPEED  = 127;


// ---- Moving the cascade to a height -----------------------------------------
// Every movement in the macro goes through this. It returns false if the move
// was cancelled, got stuck, or took too long, and the macro stops there
// instead of carrying on as if it had worked.
//
// `action` is optional: a function to call partway through the move, after
// `action_delay_ms`. That's how the flip piston fires while the cascade is
// still rising instead of waiting until it arrives.
static bool cascade_to(double target, int max_speed = CASCADE_MOVE_SPEED,
                       int action_delay_ms = -1, void (*action)() = nullptr) {
  const uint32_t start         = pros::millis();
  uint32_t       last_progress = start;
  double         last_pos      = cascade_position();
  bool           action_fired  = false;

  if (target < last_pos) max_speed = CASCADE_DOWN_SPEED;   // going down

  while (pros::millis() - start < (uint32_t)CASCADE_MOVE_TIMEOUT) {
    if (cancel_requested) { cascade_set(0); return false; }

    if (action && !action_fired &&
        (int)(pros::millis() - start) >= action_delay_ms) {
      action_fired = true;
      action();
    }

    double pos = cascade_position();
    double err = target - pos;

    // Arrived. Our first version cut the power here and moved on. The cascade
    // is heavy enough that it kept coasting: we asked for 375 and it stopped
    // at 435. Now it brakes, keeps watching, and nudges itself back if it
    // drifts. It moves on once it has sat still for 80 ms.
    if (fabs(err) <= CASCADE_MOVE_TOL) {
      cascade_hold();
      const uint32_t settle_start = pros::millis();
      uint32_t       steady_since = pros::millis();
      while (pros::millis() - settle_start < (uint32_t)CASCADE_SETTLE_MS) {
        if (cancel_requested) { cascade_set(0); return false; }
        double e = target - cascade_position();
        if (fabs(e) > CASCADE_MOVE_TOL) {
          cascade_set(e > 0 ? CASCADE_SETTLE_POWER : -CASCADE_SETTLE_POWER);
          steady_since = pros::millis();
        } else {
          cascade_hold();
          if (pros::millis() - steady_since >= (uint32_t)CASCADE_SETTLE_STABLE_MS)
            break;
        }
        pros::delay(10);
      }
      cascade_hold();
      if (action && !action_fired) { action_fired = true; action(); }
      return true;
    }

    // Stuck check. If the cascade hasn't moved 2 degrees in 600 ms while it's
    // still far from the target, something is jammed, so give up rather than
    // push at full power.
    //
    // It only counts as stuck when it's still far away. Near the target the
    // cascade slows right down on purpose, and our first version mistook that
    // for a jam and gave up on moves that were about to finish.
    if (fabs(pos - last_pos) >= CASCADE_STALL_DEG) {
      last_pos      = pos;
      last_progress = pros::millis();
    } else if (fabs(err) > CASCADE_MOVE_SLOW &&
               pros::millis() - last_progress > (uint32_t)CASCADE_STALL_MS) {
      cascade_set(0);
      return false;
    }

    // Full power until 50 degrees out, then scale down toward the target so it
    // doesn't overshoot. There's a floor, though. When we set it too low, the
    // cascade couldn't lift its own weight near the target and stopped short.
    double scale = fabs(err) / CASCADE_MOVE_SLOW;
    if (scale > 1.0) scale = 1.0;
    int power = (int)(max_speed * scale);
    if (power < CASCADE_MOVE_MIN) power = CASCADE_MOVE_MIN;
    if (err < 0) power = -power;

    cascade_set(power);
    pros::delay(10);
  }

  cascade_set(0);
  return false;   // timed out
}

// A pause that still notices a cancel. A normal delay would ignore the
// driver's cancel for however long it lasted.
static bool macro_wait(int ms) {
  const uint32_t start = pros::millis();
  while (pros::millis() - start < (uint32_t)ms) {
    if (cancel_requested) return false;
    pros::delay(10);
  }
  return true;
}

// The macro's own intake control. `roller_back` reverses just the upper
// roller for the flip; the fins and dropdown keep pulling in.
static void intake_run(bool on, bool roller_back = false) {
  int p = on ? MACRO_INTAKE_SPEED : 0;
  bool dropdown_enabled = !(high_intake_extended && middle_intake_extended);
  fin_1.move(-p);
  dropdown.move(dropdown_enabled ? -p : 0);
  upper_roller.move(roller_back ? p : -p);
  fin_2.move(p);
}


// ---- First press ------------------------------------------------------------
static void release_flip() { flip_set(FLIP_OFF); }   // fired partway up

static void phase1() {
  phase2_end = -1;   // an end height only applies to the second press

  // Put both pistons in a known state first, so the sequence runs the same
  // way no matter what the driver did before pressing.
  flip_set(FLIP_ON);
  claw_set(CLAW_OFF);

  bool ok = macro_wait(MACRO_PISTON_SETTLE);

  if (ok) ok = cascade_to(CASCADE_FLIP, CASCADE_MOVE_SPEED,
                          CASCADE_FLIP_BACK_MS, release_flip);
  if (ok) ok = macro_wait(CASCADE_FLIP_RELEASE_MS);
  if (ok) ok = cascade_to(CASCADE_COLLECT);

  cascade_set(0);
  running = false;

  // Only park if it really got there. If a move failed or was cancelled, go
  // back to idle, so the upper roller never turns on from a half-finished run.
  phase = ok ? WAITING : IDLE;
}


// ---- Second press -----------------------------------------------------------
static uint32_t flip_fired_ms = 0;

static void fire_flip() {             // fired partway up
  intake_run(true, true);             // reverse the upper roller from here on
  flip_set(FLIP_ON);
  flip_fired_ms = pros::millis();
}

static void phase2() {
  intake_owned = true;                // tells driver control to leave it alone
  intake_run(true);

  claw_set(CLAW_ON);                  // grip first, then lift
  bool ok = macro_wait(MACRO_PISTON_SETTLE);

  if (ok) ok = cascade_to(CASCADE_OUT, CASCADE_MOVE_SPEED,
                          CASCADE_FLIP_DELAY_MS, fire_flip);

  // Give the flip piston CASCADE_AFTER_FLIP_MS, counted from when it fired,
  // before the cascade moves again.
  if (ok) {
    int remain = CASCADE_AFTER_FLIP_MS - (int)(pros::millis() - flip_fired_ms);
    if (remain > 0) ok = macro_wait(remain);
  }

  // An auton can ask for a finishing height; otherwise stay at OUT.
  if (ok && phase2_end >= 0) ok = cascade_to(phase2_end);
  phase2_end = -1;

  intake_run(false);                  // off whether it worked or not
  intake_owned = false;

  if (ok) cascade_swap_hold_motor();  // share the holding load (file 2)
  cascade_hold();                     // it's raised, so hold rather than coast
  running = false;
  phase   = IDLE;
}


// ---- Starting it ------------------------------------------------------------
// The macro runs on one background task so the robot can keep driving. The
// button and the autons just leave a request for it.
static void macro_worker(void*) {
  while (true) {
    Request r = request;
    if (r != NONE) {
      request = NONE;
      if      (r == PHASE1) phase1();
      else if (r == PHASE2) phase2();
      else                  move_task();   // cascade_move_async(), file 5
    }
    pros::delay(10);
  }
}

// There's exactly one worker, and everything that starts the macro goes
// through here to get it. We had a bug where two parts of the code each made
// their own, and the two could pick up the same request and run it twice at
// once.
static void ensure_worker() {
  static pros::Task worker(macro_worker, nullptr, "Cascade Macro");
}

void macro_start() {
  if (running) { cancel_requested = true; return; }   // press again = cancel

  ensure_worker();
  running          = true;
  cancel_requested = false;
  request          = (phase == WAITING) ? PHASE2 : PHASE1;
}
