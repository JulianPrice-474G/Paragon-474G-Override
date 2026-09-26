---
name: notebook-sync
description: Bring the engineering notebook copy of the robot code (the notebook/ folder) up to date with the real code in src/ and include/, and record what changed in notebook/CHANGELOG.md with before/after code. Use this whenever the user asks to update, sync, refresh or "push changes to" the notebook, wants the notebook to show recent code changes, or asks what changed since the notebook was last updated, even if they don't say "skill" or "sync".
---

# Notebook sync

`notebook/` is a readable copy of this robot's code, split by subsystem, for
the team's VEX engineering notebook. Judges read it. It drifts every time the
real code changes, and this skill brings it back in line and leaves a written
record of what changed and why.

Two things matter more than anything else here:

1. **The notebook must match what the robot really does.** A notebook that
   describes code the robot isn't running is worse than no notebook.
2. **Nothing in it can be made up.** Every "we tried X and it did Y" is read
   as the team's real testing history. Only write history that is in the
   commit messages or that the user tells you. If you don't know why
   something changed, ask. Don't guess a plausible reason.

## Files

| notebook file | mirrors |
|---|---|
| `1-robot-config.cpp` | ports, devices, speeds, heights, timings (top of `src/main.cpp`; piston sense constants from `include/macros.hpp`) |
| `2-helpers.cpp` | helper functions in `src/main.cpp`: pistons, intake selector, intake motors and timed spins, cascade hold/position, `drive_arc` |
| `3-driver-control.cpp` | `opcontrol()` and `disabled()` in `src/main.cpp` |
| `4-cascade-macro.cpp` | `src/macros.cpp` and the tuning constants in `include/macros.hpp` |
| `5-autonomous.cpp` | `src/autons.cpp` (PID constants, `auton_setup`, routines, the tuning bench), `autonomous()` in `src/main.cpp`, and the auton-facing macro functions from `src/macros.cpp` |
| `CHANGELOG.md` | dated record of every sync |
| `.last-sync` | the commit hash the notebook was last synced to |

Out of scope, so ignore changes to them: `ui_engine.cpp`, `user_screen.cpp`,
`vision.cpp`, anything in `firmware/` or `include/pros|okapi|liblvgl|EZ-Template`,
and EZ-Template boilerplate. `notebook/README.md` lists the readability
conventions the copies follow (debug hooks removed, some variable names
spelled out, helper calls where the real code does the same thing longhand).
Keep following them.

## Steps

### 1. Find what changed

```bash
base=$(cat notebook/.last-sync)
git status --short -- src include
git log --oneline "$base"..HEAD -- src include
git diff "$base"..HEAD -- src include
```

If `src/` or `include/` has uncommitted changes, stop and ask the user
whether to commit them first. The notebook should describe code that exists
in a commit, so the record has something to point back to. Don't commit
their work without asking.

If there's nothing new since the base commit, say the notebook is already
up to date and stop.

### 2. Update the notebook files

For each change in scope, open the matching notebook file and the real code
side by side and make the notebook match. Compare against the real code
rather than applying the diff blindly: the notebook copies are tidied, so a
diff line won't always appear in them word for word, and a change may already
be reflected.

Comments follow the style already in the files:

- Plain language, team voice ("we"). Explain why the code is the way it is,
  not what each line does.
- Name the real problem when there was one, with real numbers from the commit
  message ("we asked for 375 and it stopped at 435").
- No em dashes. None of: crucial, robust, seamless, leverage, delve, ensure,
  pivotal, essential, comprehensive, furthermore, additionally, testament,
  showcase. No three-item lists added for rhythm.
- A change that only moves a number (a height, a timing) usually needs no new
  comment. The changelog carries it.

### 3. Write the changelog entry

Add a new entry at the **top** of `notebook/CHANGELOG.md`, below the title.
This is the page the team puts in the notebook to show how the code
developed, so it's written for a reader, not as a commit log:

~~~markdown
## 26 Sep 2026

**What changed:** One or two sentences a judge could follow.

**Why:** The reason, from the commit messages or the user. Leave this out
rather than invent one.

**Before:**
```cpp
int CASCADE_FLIP_DELAY_MS = 300;
```

**After:**
```cpp
int CASCADE_FLIP_DELAY_MS = 200;
```

**Notebook pages updated:** 1-robot-config, 4-cascade-macro
~~~

Give each separate change its own What/Why/Before/After block inside the
day's entry. Keep the snippets short, just the lines that changed plus
enough context to recognise them. A change that only tidies code or
comments doesn't need before/after.

Use today's date in `DD Mon YYYY` form.

### 4. Show the user, then record the sync

Show the user the new changelog entry and a short summary of which notebook
files changed (`git diff --stat -- notebook/`). If they want changes, make
them before recording.

Then:

```bash
git rev-parse HEAD > notebook/.last-sync
make 2>&1 | grep -iE "error|DONE"   # notebook/ must not be part of the build
git add notebook/
git commit -m "Notebook: <short summary of what was synced>"
```

Commit only `notebook/`. Don't push unless the user asks. This project's
rule is commit every change, push only on request.
