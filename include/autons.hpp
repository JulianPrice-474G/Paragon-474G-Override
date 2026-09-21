#pragma once

void default_constants();

// ── Your autons ───────────────────────────────────────────────────────────────
// These are what the five buttons on the brain actually run.  Write them in
// src/autons.cpp.  The example routines below are EZ-Template's reference code
// and are no longer wired to anything - leave them, copy from them, or delete.
void sawp();
void skills();
void one_pin();
void auto_4();
void auto_5();

void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();