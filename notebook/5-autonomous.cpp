// ============================================================================
//  5. Autonomous
//  PID tuning values, the setup every auton runs first, the functions that
//  let an auton use the cascade macro, and our SAWP route.
// ============================================================================


// ---- Speeds (out of 127) ----------------------------------------------------
const int DRIVE_SPEED = 110;
const int TURN_SPEED  = 90;
const int SWING_SPEED = 110;


// ---- PID constants ----------------------------------------------------------
// P, I, D (and start-I for turns). We tuned these in the order heading, drive,
// turn, swing, since each one leans on the ones before it. A robot that drifts
// sideways makes the drive distance test read wrong, for example.
//
// We ended up keeping EZ-Template's defaults. On the tuning bench below, all
// four tests landed where they should, so there was nothing to change.
void default_constants() {
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);      // forward / back
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);     // staying straight
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);  // turning in place
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);        // swing turns

  // When a movement counts as finished: small error held for 90 ms, a bigger
  // error held for 250 ms, or no progress for 500 ms.
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  // How far past a target to keep going when chaining moves together with
  // pid_wait_quick_chain(), so the robot flows through corners.
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew: ramp up to speed over the first few inches instead of jumping to
  // full power, so the wheels don't slip at the start of a move.
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.pid_angle_behavior_set(ez::shortest);   // always turn the short way
}


// ---- Starting an auton ------------------------------------------------------
// Runs before whichever auton is selected. Zeroes the chassis so every
// distance and heading is measured from where the robot starts.
void autonomous() {
  chassis.pid_targets_reset();
  chassis.drive_imu_reset();
  chassis.drive_sensor_reset();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);

  switch (get_selected_auton()) {   // picked on the brain screen
    case 0: sawp();    break;
    case 1: skills();  break;
    case 2: one_pin(); break;
    case 3: auto_4();  break;
    case 4: auto_5();  break;
  }
}

// First line of every auton. autonomous() handles the drivetrain; this
// handles everything else, so a route starts the same way even if driver
// practice left a motor running or a piston out.
void auton_setup() {
  cascade_set(0);
  intake_set(0, true);
  cascade_apply_hold_motor();

  intake_pos_set(INTAKE_HIGH);
  claw_set(CLAW_ON);        // start with the claw closed
  flip_set(FLIP_ON);

  pros::delay(50);          // give the valves a moment before driving
}


// ---- Using the macro in an auton --------------------------------------------
// macro_press() does what a RIGHT press does in driver control. The second
// press can also say where the cascade should end up after the flip.
//
//   macro_press();          // first press
//   macro_wait_done();      // wait for it to park
//   macro_press(500);       // second press, finish at 500
//
// macro_wait_done() waits for the macro, up to a time limit, and returns false
// if something went wrong. Without that limit, a jammed cascade could stall
// the whole auton for the rest of the period.
void macro_press(double end_height) {
  phase2_end = end_height;
  macro_start();
}

bool macro_wait_done(int timeout_ms) {
  const uint32_t start = pros::millis();
  while (running) {
    if ((int)(pros::millis() - start) > timeout_ms) return false;
    pros::delay(10);
  }
  return !failed;
}

// Move the cascade to a height in the background, so the robot can drive at
// the same time. It uses the macro's worker (file 4), so a move and the macro
// can never both be driving the cascade.
static double move_target = 0;
static int    move_speed  = 0;

static void move_task() {
  bool ok = cascade_to(move_target, move_speed);
  cascade_set(0);
  running = false;
  failed  = !ok;
}

void cascade_move_async(double target, int speed) {
  if (running) return;          // the macro already has the cascade
  ensure_worker();
  move_target = target;
  move_speed  = (speed > 0) ? speed : CASCADE_MOVE_SPEED;
  running     = true;
  request     = MOVE;
}


// ---- SAWP -------------------------------------------------------------------
void sawp() {
  auton_setup();

  upper_roller_spin(700, 127);           // roller on for 0.7 s, in the background
  pros::delay(200);
  cascade_move_async(240, 127);          // cascade up to 240 while we drive

  chassis.pid_drive_set(-2_in, 100);     // back up 2 in
  chassis.pid_wait_quick_chain();
  drive_arc(100, -25, -127);             // curve backwards to face 100 degrees
  chassis.pid_wait();
  pros::delay(300);
  claw_set(false);                       // open the claw
  pros::delay(300);

  macro_press();                         // macro, first press
  press_y();                             // intake to MIDDLE
  intake_set(3000, 127);
  drive_arc(145, 127, 50);               // curve to face 145
  chassis.pid_wait();
  pros::delay(2000);

  chassis.pid_turn_set(245, 127);        // turn to face 245
  chassis.pid_wait();
  macro_press(500);                      // macro, second press, finish at 500
  chassis.pid_drive_set(-10, 127);       // back up 10 in while it runs
  chassis.pid_wait_quick_chain();
  macro_wait_done(400);
  chassis.pid_drive_set(-17, 127);       // back up 17 more
  chassis.pid_wait();

  cascade_move_async(300, 127);          // cascade to 300
  pros::delay(300);
  claw_set(false);                       // open the claw
  pros::delay(300);
  intake_set(6000, 127);
  drive_arc(250, 127, 100);              // curve to face 250
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10, 127);        // forward 10 in
  chassis.pid_wait();
}


// ---- PID tuning bench (auton slot 5) ----------------------------------------
// We used this to check the PID constants above. Set PID_TEST to the test,
// upload, pick slot 5 on the brain, and run it. Each test does a move and then
// comes back, so you can see how close it lands to where it started.
void auto_5() {
  auton_setup();

  const int PID_TEST = 4;   // 1 heading, 2 drive, 3 turn, 4 swing

  switch (PID_TEST) {
    case 1:   // heading: 48 in out and back. Watch for sideways drift.
      chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
      chassis.pid_wait();
      pros::delay(500);
      chassis.pid_drive_set(-48_in, DRIVE_SPEED, true);
      chassis.pid_wait();
      break;

    case 2:   // drive: 24 in out and back. Tape-measure it.
      chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
      chassis.pid_wait();
      pros::delay(1000);
      chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
      chassis.pid_wait();
      break;

    case 3:   // turn: 90, back to 0, then 180 to catch error the small turns hide
      chassis.pid_turn_set(90_deg, TURN_SPEED);
      chassis.pid_wait();
      pros::delay(700);
      chassis.pid_turn_set(0_deg, TURN_SPEED);
      chassis.pid_wait();
      pros::delay(700);
      chassis.pid_turn_set(180_deg, TURN_SPEED);
      chassis.pid_wait();
      break;

    case 4:   // swing: both directions
      // Our first version swung out with LEFT_SWING and came back with
      // LEFT_SWING too. The return always came up short, because coming back
      // that way drives the same side in reverse. Alternating LEFT and RIGHT
      // keeps every swing driving forward.
      chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED, 45);
      chassis.pid_wait();
      pros::delay(700);
      chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
      chassis.pid_wait();
      pros::delay(700);
      chassis.pid_swing_set(ez::RIGHT_SWING, -90_deg, SWING_SPEED, 45);
      chassis.pid_wait();
      pros::delay(700);
      chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
      chassis.pid_wait();
      break;
  }
}
