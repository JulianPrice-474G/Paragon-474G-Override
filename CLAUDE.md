# Paragon 474G — robot code

VEX V5 competition robot, team 474G. PROS + EZ-Template, with a custom LVGL
brain UI. The code is the source of truth for ports and values; this file
records what the code does *not* make obvious.

## Working with Julian

- **Short, direct answers.** No long preambles or recaps. Lead with the answer.
- **Commit after every change**, with a real message. **Push only when asked** —
  not after every commit.
- **If asked for steps, give steps — do not edit files.** Only edit when asked
  to make the change.
- **Ask before starting when a request has two reasonable readings** that lead
  to different code (which motor, which button, which direction). Don't guess
  on those; do guess on routine details.
- **Verify before claiming.** Build every change. Say plainly what is verified
  (compiles, links) versus untested on the robot. Several past "fixes" were
  aimed at the wrong cause because they were asserted rather than checked —
  read the source (PROS headers, EZ-Template source) instead of guessing.

## Branches

- `ai-vision` — **active branch.** PROS 4.2.2, AI Vision sensor, all macro work.
- `main` — fallback on PROS 4.1.1. Behind `ai-vision`; do not merge without
  being asked.

A branch switch changes the compiled libraries in `firmware/` too. **Always
rebuild (`rm -rf bin && make`) after switching** or you flash the wrong binary.

## Toolchain — do not "fix" this

PROS 4.2.x is built **hard-float** (`-mfloat-abi=hard`); every template published
for 4.1.x is soft-float and will not link against it (`uses VFP register
arguments`). The working combination:

- kernel **4.2.2**, liblvgl **8.3.9** (hard-float, still the LVGL 8 API)
- **EZ-Template 3.2.2 rebuilt from source** against 4.2.2 — the official 3.2.2
  template is soft-float. `firmware/EZ-Template.a` is that rebuild.
- **okapilib's `.a` is deleted**, headers kept. EZ-Template needs no real
  `okapi::` symbols; `okapi::literals` (`24_in`, `90_deg`) is header-only.

Never `pros c apply` EZ-Template or okapilib over this — it reinstates the
soft-float archives and breaks the link. Check any `.a` with
`arm-none-eabi-readelf -A lib.a | grep ABI_VFP_args`.

## Files

| file | what it holds |
|---|---|
| `src/main.cpp` | ports, speeds, cascade heights, devices, helpers, `opcontrol()`, `autonomous()` |
| `src/autons.cpp` | PID constants (`default_constants`), `auton_setup()`, the auton routines |
| `src/macros.cpp` / `include/macros.hpp` | cascade macro sequence; its tuning constants |
| `src/user_screen.cpp` | brain UI pages and controller screen (team-specific) |
| `src/ui_engine.cpp` | UI engine — shared with the template repo, see below |
| `src/vision.cpp` | AI Vision helpers and the live detection view |
| `include/subsystems.hpp` | externs + subsystem helper declarations |

## Always use the subsystem helpers

Never call `.set_value()` on a solenoid or `.move()` on intake/cascade motors
directly. A `DigitalOut` cannot be read back, so software mirrors
(`claw_extended`, `intake_piston_extended`, …) are the only record of piston
state — and the DOWN/LEFT toggles and the controller readout read them.

- Pistons: `claw_set()`, `flip_set()`. The intake is one 25 mm piston on port F
  (extended = HIGH, retracted = MIDDLE): use `intake_pos_set()` / `press_y()`,
  which keep `intake_pos` in step. The dropdown motor always runs with the intake.
- Intake: `intake_spin(ms, speed)` — non-blocking; `ms = -1` runs until
  `intake_spin_stop()`. `fins_spin()` / `fins_set()` drive only the two fins.
  `intake_spin` and `fins_spin` share one worker, so starting one replaces the
  other. `intake_set(power, roller)` is the primitive underneath.
- Fins: only the "Fin Sync" task writes `fin_1`/`fin_2`. `fins_set()` records a
  power; the task slows whichever fin is ahead of the positions taken by
  `fins_sync_zero()` (start of every auton, and power-on). `FIN_SYNC_KP/MAX`
  in `main.cpp`.
- Fin jam guard, **autons only** (`fins_jam_guard()`, on in `autonomous()`,
  off in `opcontrol()`/`disabled()`): `FIN_JAM_NM` (torque) for `FIN_JAM_MS` stops the
  fins alone until the next spin command. The spin worker writes through
  `fins_write()`, which is not a command and so does not clear a jam.
- Cascade: `cascade_set(power)`; read position with `cascade_position()`
  (rotation sensor, converted to degrees, falls back to the motor encoder).
- `drive_arc(deg, left, right)` — open-loop curve to an absolute heading. Safe to
  mix with PID motions: EZ's `drive_set()` disables PID mode, odom keeps running.
- `release_all_pistons()` — called from `disabled()`.

Positive intake power runs the intake the same way the **R1** button does.
`fin_2` is mounted opposite and is always commanded the other sign.

## Driver control

Subsystems only respond in **driver mode** (hold **UP+X** 1 s to toggle).
Outside it the UI owns LEFT/RIGHT/A/B for menu navigation.

| button | action |
|---|---|
| L1 / L2 | cascade up (stops at `CASCADE_MAX`) / down at 50% |
| R1 / R2 | intake, both directions |
| Y | intake piston HIGH ↔ MIDDLE |
| DOWN / LEFT | toggle claw / toggle C-flip |
| RIGHT | cascade macro |
| LEFT+B held 1 s | run selected auton — only outside driver mode, off a comp switch |
| A (hold) | all four intake motors the R2 way, upper roller included at any cascade height |
| B | unused (still half of the LEFT+B auton combo) |

Controller in driver mode: row 1 `p### temps heading` (IMU zeroed on entering driver mode), row 2 commanded piston
states `C F I`.

## Cascade

- **Only one motor holds.** Two motors in `BRAKE_HOLD` on one shaft each run a
  position PID on their own encoder and fight forever (measured 0.9 A at rest).
  One holds, the other coasts; the macro swaps which after each completed run.
- A shared PD hold (one reading driving both) was tried and reverted — it
  misbehaved on the robot. Its constants remain in `main.cpp`, unused.
- Heights are rotation-sensor readings (the `p` value), set at the top of
  `main.cpp`. The sensor counts from wherever it powered up; the commented
  `cascade_rot.reset_position()` in `auton_setup()` is deliberately off.

## The macro (RIGHT)

- **Press 1:** preflight (flip on) → rise to FLIP and **arrive** → open the claw
  and drop the flip piston together → wait `CASCADE_DROP_DELAY_MS` → lower to
  COLLECT → park. While parked, L1/L2 are locked and the upper roller is armed.
  (A height-triggered version that fired mid-rise was glitchy on the robot and
  was replaced by this.)
- **First press 1 of an auton** can use its own height and drop wait
  (`CASCADE_FIRST_FLIP`, `CASCADE_FIRST_DROP_DELAY_MS`), switched by
  `CASCADE_FIRST_FLIP_ON` in `main.cpp`. Armed by `autonomous()`, used up by
  the first phase 1; driver control always uses the normal values.
- **Press 2:** intake on → claw on → raise to OUT, with the flip piston extended
  and the upper roller reversed `CASCADE_FLIP_DELAY_MS` into the rise → hold
  until `CASCADE_AFTER_FLIP_MS` after the flip → **stays at OUT** by default →
  intake off, swap holding motor, hold position.
- **The macro is never broken by another cascade request.** In an auton:
  `macro_press()` cancels a running `cascade_move_async()` and starts at once,
  but **waits** for an unfinished macro step (cancelling press 1 used to make
  the "second" press restart press 1). `cascade_move_async()` during a macro
  step or a pending delayed press **waits** and runs after it (it used to be
  silently dropped). One queued slot each; a `macro_press()` call drops a
  queued move. (RIGHT in driver control still only cancels.)
- `macro_press(h)` on the second press sends the cascade on to
  height `h` after the flip. The value is ignored on the first press, and it is
  cleared after use, so a later RIGHT press in driver control stays at OUT.
- Pressing RIGHT mid-move cancels. L1/L2 are **ignored** while the macro moves
  (they used to cancel it) and work again once it ends. One long-lived worker task,
  shared with `cascade_move_async()` so the two never drive the cascade at once.
- All the timings and heights are at the top of `main.cpp`.
- Moves brake and settle at the target (a heavy cascade coasted 60° past), then
  leave as soon as they are steady. Downward moves use `CASCADE_DOWN_SPEED`.
  Easing and the stall guard trade off: too little floor power stalls, too much
  overshoots.
- Claw and flip piston have **separate** sense constants in `macros.hpp`:
  `CLAW_ON`/`CLAW_OFF` and `FLIP_ON`/`FLIP_OFF`. Both have been flipped more
  than once; confirm on the robot before assuming.

## Autons

`autonomous()` resets the chassis (IMU, encoders, PID targets, odom, brake)
before the routine; each routine then calls `auton_setup()` for subsystems.
Slots: `sawp`, `skills`, `one_pin`, `auto_4`, `auto_5`. **`auto_5` is the PID
test bench** — pick a test with `PID_TEST`. Default PID constants have been
verified for heading, drive, turn and swing.

Swing returns must **alternate** swing type (LEFT out, RIGHT back) — returning
with the same type drives one side in reverse and lands short.

## Traps that have bitten this project

- **Stale VS Code tabs.** If a file changed on disk while its tab was open,
  saving overwrites the change (this silently reverted work twice). Tell Julian
  to run **File: Revert File** before editing a file you have just changed.
- `get_digital_new_press()` is **consumed by the first caller**. The UI reads
  LEFT/RIGHT/A/B that way outside driver mode.
- **Do not `lv_obj_clean(lv_scr_act())` after `pros::lcd::initialize()`** — it
  frees LLEMU's objects while LLEMU keeps pointers, and the next LLEMU call data
  aborts. The boot spinner goes on its own explicitly sized screen instead.
- `lv_obj_remove_style_all()` strips width and height — set them back.
- `PROS_ERR_F` is NaN and every comparison with NaN is false; test for a *good*
  value, not a bad one.
- `pros::Task` has no destructor — `delete` leaks the RTOS task. Use one
  long-lived worker.
- `pros::Rotation::get_position()` is **centidegrees**; motors are degrees.

## Brain UI template

The UI engine is also published at
`github.com/JulianPrice-474G/Custom-V5-Brain-UI-Template`. `ui_engine.cpp` /
`.hpp` are shared with it; `user_screen.cpp` is team-specific and deliberately
differs from the template's demo version. Engine fixes belong in both — ask
before pushing to the template repo.

## Open items

- The claw and flip piston sense (`CLAW_ON` / `FLIP_ON` in `macros.hpp`) was
  changed several times — confirm on the robot before assuming it.
- The rotation sensor's zero is not anchored; absolute heights may drift across
  reboots.
- A startup data abort was seen once and resolved without an identified cause.
  If it returns, stop and bisect before building further.
