# Robot code for the engineering notebook

Team 474G, Paragon. These files are copies of our robot code, split up by
subsystem so each part can go on its own page. The real code is in `src/` and
`include/`; nothing in this folder gets compiled or uploaded to the robot.

We built on PROS with the EZ-Template library for the drivetrain, and wrote
everything else ourselves.

| file | what's in it |
|---|---|
| `1-robot-config.cpp` | Ports, motor and piston setup, speeds, cascade heights and timings |
| `2-helpers.cpp` | Small functions every other part uses to run the intake, pistons and cascade |
| `3-driver-control.cpp` | What each controller button does during a match |
| `4-cascade-macro.cpp` | The two-press sequence that collects a game object and flips it out |
| `5-autonomous.cpp` | PID tuning values, auton setup, and our SAWP route |
| `CHANGELOG.md` | How the code has changed over the season, with before and after code |

A few things differ from the files on the robot, all to make the pages easier
to read:

- The brain screen and controller display code is left out. It's long and it
  isn't how the robot plays.
- Debug-only hooks (a step-by-step pause we used while testing the macro, and
  the text labels it sent to the controller) are removed. They do nothing at
  match speed.
- Library boilerplate from EZ-Template's example project is removed.
- A few internal variable names are spelled out (`running` instead of
  `_running`, for example), and a couple of lines use the helper functions
  from file 2 where the robot code does the same thing longhand.

What the robot does is the same as the real code. If the two ever disagree,
the files in `src/` are the ones that run.
