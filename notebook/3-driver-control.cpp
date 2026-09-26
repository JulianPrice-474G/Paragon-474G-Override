// ============================================================================
//  3. Driver control
//
//  Controller layout (in driver mode):
//    left stick / right stick   drive (split arcade)
//    L1 / L2                    cascade up / down (down at half speed)
//    R1 / R2                    intake, both directions
//    Y                          intake to MIDDLE, press again for HIGH
//    B                          intake to LOW, press again for HIGH
//    DOWN                       open / close the claw
//    LEFT                       flip piston
//    RIGHT                      cascade macro (see file 4)
//    UP + X, held 1 second      turn driver mode on and off
//
//  Driver mode exists because our brain screen menu uses the controller too.
//  With driver mode off, the arrow buttons scroll the menu and nothing on the
//  robot moves. With it on, every button belongs to the driver. That stops
//  someone scrolling through autons and accidentally firing a piston.
// ============================================================================

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  // Set up the one-motor hold (file 2). Done here again because running an
  // auton changes the brake modes.
  cascade_apply_hold_motor();

  while (true) {
    handle_ctrl_input();   // brain screen menu + driver mode toggle

    chassis.opcontrol_arcade_standard(ez::SPLIT);

    if (DriverModeActive()) {

      // ---- Cascade macro --------------------------------------------------
      // RIGHT starts the macro, or cancels it if it's already moving. It runs
      // in the background, so driving still works the whole time.
      if (master.get_digital_new_press(DIGITAL_RIGHT)) macro_start();

      // Grabbing L1 or L2 also cancels it. In a match the driver shouldn't
      // have to remember which button stops it.
      if (macro_running() &&
          (master.get_digital(DIGITAL_L1) || master.get_digital(DIGITAL_L2)))
        macro_cancel();

      // ---- Intake position ------------------------------------------------
      if (master.get_digital_new_press(DIGITAL_Y)) press_y();
      if (master.get_digital_new_press(DIGITAL_B)) press_b();

      // ---- Claw and flip piston -------------------------------------------
      // Toggles: one press opens, the next closes.
      if (master.get_digital_new_press(DIGITAL_DOWN)) claw_set(!claw_extended);
      if (master.get_digital_new_press(DIGITAL_LEFT)) flip_set(!c_flip_extended);

      // ---- Intake motors --------------------------------------------------
      // The upper roller only turns while the macro is parked at the collect
      // height. We first had it on whenever the cascade was near that height,
      // but then it would start spinning if the driver just raised the cascade
      // past that point. Now it only arms when the macro has actually put the
      // cascade there.
      bool roller_enabled = cascade_at_collect();

      // When the macro or a timed spin is running the intake, leave it alone.
      // Otherwise the "nothing pressed" branch below sets the motors to zero
      // every 10 ms and the macro's intake never gets to turn.
      if (macro_owns_intake() || intake_spin_active()) {
        // someone else is driving the intake
      } else if (master.get_digital(DIGITAL_R2)) {
        intake_set(-R_SPEED, roller_enabled);
      } else if (master.get_digital(DIGITAL_R1)) {
        intake_set(R_SPEED, roller_enabled);
      } else {
        intake_set(0, true);
      }

      // ---- Cascade --------------------------------------------------------
      if (macro_running()) {
        // the macro is moving the cascade, hands off
      } else if (macro_waiting()) {
        // Parked at collect between the two presses. L1/L2 are ignored here
        // so the cascade can't get bumped off the height the second press
        // expects.
        cascade_hold();
      } else if (master.get_digital(DIGITAL_L1)) {
        // Stop at the top limit instead of running into the hard stop.
        if (cascade_position() >= CASCADE_MAX) cascade_hold();
        else                                   cascade_set(L_SPEED);
      } else if (master.get_digital(DIGITAL_L2)) {
        cascade_set(-L2_SPEED);
      } else {
        cascade_hold();   // nothing pressed: hold where it is
      }

    } else {
      // Driver mode off: the controller is running the menu, so the robot
      // stays still.
      cascade_set(0);
      intake_set(0, true);
    }

    pros::delay(10);
  }
}

// When the match ends or the field disables the robot, let the air out.
void disabled() {
  release_all_pistons();
}
