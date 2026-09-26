// ============================================================================
//  2. Helper functions
//  Driver control, the macro and the autons all move the robot through these
//  instead of talking to the motors and valves directly. That way each rule,
//  like "fin_2 runs backwards" or "the dropdown can't run when the intake is
//  up", lives in one place and can't be forgotten somewhere else.
// ============================================================================


// ---- Pistons ----------------------------------------------------------------
// Each one sets the valve and updates our record of it in the same line. If
// the code set a valve without updating the record, the toggle buttons and the
// dropdown check would be working from the wrong state, and nothing would tell
// us. Using these everywhere means that can't happen.
void claw_set(bool on)          { claw_extended          = on; claw.set_value(on); }
void flip_set(bool on)          { c_flip_extended        = on; c_flip.set_value(on); }
void high_intake_set(bool on)   { high_intake_extended   = on; high_intake.set_value(on); }
void middle_intake_set(bool on) { middle_intake_extended = on; middle_intake.set_value(on); }

// Let the air out of everything. Runs when the robot is disabled so it
// doesn't sit holding pressure between matches.
void release_all_pistons() {
  high_intake_set(false);
  middle_intake_set(false);
  claw_set(false);
  flip_set(false);
}


// ---- Intake position (Y and B buttons) --------------------------------------
// The two intake cylinders give three positions:
//   HIGH    both out, the normal position
//   MIDDLE  middle out, upper in
//   LOW     both in
//
// Our first version had Y and B each flip one cylinder from whatever state it
// was in. After a few presses the two would end up in a combination nobody
// meant, and the next press looked random. Now there's one value for the
// position, and both buttons set it.
enum IntakePos { INTAKE_HIGH, INTAKE_MIDDLE, INTAKE_LOW };
IntakePos intake_pos = INTAKE_HIGH;

void intake_pos_set(IntakePos pos) {
  intake_pos = pos;
  high_intake_set  (pos == INTAKE_HIGH);   // upper cylinder: only in HIGH
  middle_intake_set(pos != INTAKE_LOW);    // middle cylinder: HIGH and MIDDLE
}

// Pressing the same button twice goes back to HIGH, so the normal position
// is always one press away.
void press_y() { intake_pos_set(intake_pos == INTAKE_MIDDLE ? INTAKE_HIGH : INTAKE_MIDDLE); }
void press_b() { intake_pos_set(intake_pos == INTAKE_LOW    ? INTAKE_HIGH : INTAKE_LOW); }


// ---- Intake motors ----------------------------------------------------------
// Positive power runs the intake the same direction as the R1 button.
//
// The dropdown only runs when the intake is lowered (MIDDLE or LOW). In the
// HIGH position it's held at zero, and every intake function checks.
void intake_set(int power, bool roller) {
  bool dropdown_enabled = !(high_intake_extended && middle_intake_extended);

  fin_1.move(-power);
  fin_2.move(power);                                  // mounted backwards
  dropdown.move(dropdown_enabled ? -power : 0);
  upper_roller.move(roller ? -power : 0);
}

// One part of the intake on its own.
void fins_set(int power) {
  fin_1.move(-power);
  fin_2.move(power);
}
void upper_roller_set(int power) { upper_roller.move(-power); }
void dropdown_set(int power) {
  bool enabled = !(high_intake_extended && middle_intake_extended);
  dropdown.move(enabled ? -power : 0);
}


// ---- Timed intake spins -----------------------------------------------------
// In autons we wanted the intake to keep running while the robot drives, so
// these don't wait. You say how long and how fast, the function returns right
// away, and a background task turns it off when the time is up.
//
//   upper_roller_spin(800, 127);                // starts, doesn't wait
//   chassis.pid_drive_set(24_in, DRIVE_SPEED);  // roller still going
//
// A negative time means "until told to stop". The fins, the roller and the
// dropdown each have their own timer, so starting one doesn't cut off another.
struct SpinChannel {
  int  until_ms;   // when to stop, or -1 for never
  int  speed;
  bool active;
};
static volatile SpinChannel ch_fins   = {0, 0, false};
static volatile SpinChannel ch_roller = {0, 0, false};
static volatile SpinChannel ch_drop   = {0, 0, false};

bool intake_spin_active() {
  return ch_fins.active || ch_roller.active || ch_drop.active;
}

static int channel_power(volatile SpinChannel& c) {
  if (!c.active) return 0;
  if (c.until_ms >= 0 && (int)pros::millis() >= c.until_ms) {
    c.active = false;
    return 0;
  }
  return c.speed;
}

static void intake_spin_task(void*) {
  bool was_driving = false;
  while (true) {
    bool driving = intake_spin_active();
    // Keep writing while anything is running, plus one last pass to zero the
    // motors. After that it goes quiet so it doesn't fight the driver's
    // R1/R2 controls.
    if (driving || was_driving) {
      fins_set(channel_power(ch_fins));
      upper_roller_set(channel_power(ch_roller));
      dropdown_set(channel_power(ch_drop));
    }
    was_driving = driving;
    pros::delay(10);
  }
}

static void channel_start(volatile SpinChannel& c, int ms, int speed) {
  // One background task, made the first time it's needed. PROS tasks can't be
  // cleanly deleted, so making a new one every call would leak.
  static pros::Task worker(intake_spin_task, nullptr, "Intake Spin");

  if (ms == 0 || speed == 0) { c.active = false; return; }
  c.speed    = speed;
  c.until_ms = (ms < 0) ? -1 : (int)pros::millis() + ms;
  c.active   = true;
}

void fins_spin(int ms, int speed)         { channel_start(ch_fins,   ms, speed); }
void upper_roller_spin(int ms, int speed) { channel_start(ch_roller, ms, speed); }
void dropdown_spin(int ms, int speed)     { channel_start(ch_drop,   ms, speed); }

void intake_spin(int ms, int speed) {     // all three at once
  fins_spin(ms, speed);
  upper_roller_spin(ms, speed);
  dropdown_spin(ms, speed);
}
void intake_spin_stop() { intake_spin(0, 0); }


// ---- Cascade ----------------------------------------------------------------
void cascade_set(int power) {
  l_motor_a.move(power);
  l_motor_b.move(-power);
}

// Where the cascade is, in degrees. The rotation sensor reports hundredths of
// a degree, hence the /100. If the sensor gets unplugged mid-match we fall
// back to the motor's own encoder: less accurate, but the cascade keeps
// holding instead of losing its target.
double cascade_position() {
  if (cascade_rot.get_position() == PROS_ERR) return l_motor_a.get_position();
  return cascade_rot.get_position() / 100.0;
}

// Holding the cascade up.
//
// At first both motors were set to hold position. With the cascade resting on
// its bottom stop, doing no work at all, they drew 0.9 A. Each motor holds
// against its own encoder, the two never agree exactly, so they pushed against
// each other forever and got hot.
//
// Now only one motor holds and the other coasts along with the shaft. Since
// the holding motor carries all the load, the macro swaps which one it is after
// every run so they heat up evenly.
int cascade_hold_motor = 0;   // 0 = motor A holds, 1 = motor B holds

void cascade_apply_hold_motor() {
  if (cascade_hold_motor == 0) {
    l_motor_a.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    l_motor_b.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  } else {
    l_motor_a.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    l_motor_b.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  }
}

void cascade_swap_hold_motor() {
  cascade_hold_motor = 1 - cascade_hold_motor;
  cascade_apply_hold_motor();
}

void cascade_hold() {
  if (cascade_hold_motor == 0) { l_motor_a.brake(); l_motor_b.move(0); }
  else                         { l_motor_b.brake(); l_motor_a.move(0); }
}


// ---- drive_arc ---------------------------------------------------------------
// EZ-Template's swing turns felt like more setup than we needed for simple
// curves, so we wrote our own: give each side a speed and the heading to
// stop at.
//
//   drive_arc(90, 100, 40);    // curve right until facing 90 degrees
//   drive_arc(90, 80, -80);    // spin in place to 90
//
// It doesn't correct itself like a PID turn. It drives at the speeds you give
// and stops when it gets there, so fast arcs slide a little past.
constexpr double ARC_TOL_DEG = 2;

bool drive_arc(double target_deg, int left_speed, int right_speed, int timeout_ms) {
  const uint32_t start = pros::millis();

  // Always go the short way round: 350 to 10 is a 20 degree turn, not 340.
  auto error_to_target = [&]() {
    double e = target_deg - chassis.drive_imu_get();
    while (e >  180) e -= 360;
    while (e < -180) e += 360;
    return e;
  };

  double first = error_to_target();
  if (fabs(first) < 1) { chassis.drive_set(0, 0); return true; }

  while (pros::millis() - start < (uint32_t)timeout_ms) {
    double e = error_to_target();

    // Stop when we're close, or the moment we pass the target. Nothing slows
    // the robot down on the way in, so at speed it can jump straight past the
    // 2 degree window between two readings. Catching the crossing stops it
    // spinning on.
    if (fabs(e) <= ARC_TOL_DEG || (e > 0) != (first > 0)) {
      chassis.drive_set(0, 0);
      return true;
    }

    chassis.drive_set(left_speed, right_speed);
    pros::delay(10);
  }

  chassis.drive_set(0, 0);
  return false;   // ran out of time, probably blocked
}
