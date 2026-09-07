#include "main.h"
#include "ui_engine.hpp"

// ══════════════════════════════════════════════════════════════════════════════
//  user_screen.cpp  —  THE ONLY FILE YOU EDIT TO CUSTOMIZE YOUR BRAIN SCREEN
//
// ── COORDINATE SYSTEM ─────────────────────────────────────────────────────────
//
//  The LVGL drawing area is 480 px wide × 240 px tall.
//  The top 32 px of the physical screen is the VEX system bar (battery icon,
//  field connection status) — LVGL cannot draw there.
//  Top-left of the LVGL area is (0, 0). X goes right, Y goes down.
//
//       (0,0) ────────────────────────── (480,0)
//         │                                  │
//         │    LVGL drawing area (480×240)   │
//         │                                  │
//       (0,240) ────────────────────── (480,240)
//
// ── IF YOU START FROM SCRATCH/MAKE YOUR OWN UI ─────────────────────────────────────────────────
//
//  These five functions are called by main.cpp and main.h — they MUST exist or
//  the project will not build, even if you delete everything else in this file:
//
//    int         get_selected_auton()  { return SelectedAuton(); }
//    void        handle_ctrl_input()   { }   // can be empty
//    void        build_screens()       { /* your PageAdd/ButtonAdd calls */ }
//    const char* battery_text()        { /* snprintf into static buf */ }
//    const char* ctrl_battery_text()   { /* snprintf into static buf */ }
//
// ── WHAT THIS FILE CONTAINS ───────────────────────────────────────────────────
//
//  build_screens()      — declares all brain screen pages and their widgets.
//                         Called once inside initialize() in main.cpp.
//                         This is where you set up the auton selector layout,
//                         status tab, detail pages, and any custom screens.
//
//  handle_ctrl_input()  — called automatically every opcontrol loop tick from
//                         main.cpp. Handles controller display text and button
//                         navigation. Do not call it yourself.
//                         Customize the text in _ctrl_home(), _ctrl_nav(), and
//                         _ctrl_auton() to match your robot and auton names.
//
//  Helper getters       — battery_text(), ctrl_battery_text(), get_selected_auton().
//                         Used by the brain screen live labels and by main.cpp.
//
// ── COMPETITION CHECKLIST ─────────────────────────────────────────────────────
//
//  Before your first competition match, make these changes:
//
//  □  Change "Team XXXX" in build_screens() → your team number
//  □  Change "Auton 1", "Auton 2", "Skills" → your actual route names
//       in the ButtonAdd calls on "auton_tab"
//       in the LabelAdd calls on each auton detail page
//       in the names[] array inside _ctrl_auton()
//  □  Change "Custom Brain UI" in _ctrl_home() → your team/robot name
//  □  Add your actual auton functions to autonomous() in main.cpp/autons.cpp
//  □  Select auton, then HOLD UP + X for 1 second before every match to lock the brain screen
//
// ── DRIVER MODE  ★ READ THIS BEFORE COMPETING ★ ──────────────────────────────
//
//  Driver mode locks the brain screen and frees LEFT/RIGHT/A/B on the controller
//  for robot subsystems (intake, arm, claw, etc.) during a match.
//
//  HOW TO TOGGLE:  HOLD UP + X together for 1 second. Same combo exits.
//                  A quick tap of either button does nothing, so UP and X stay
//                  usable for your subsystems. Change the buttons and the hold
//                  time at DRIVER_MODE_BTN_A / _B / _HOLD_MS below.
//
//  WHAT HAPPENS WHEN ACTIVE:
//    • Brain screen shows "MATCH IN PROGRESS" — touch is fully disabled so
//      nothing gets tapped accidentally during the match.
//    • Controller shows "** DRIVER MODE **" on row 0.
//    • LEFT / RIGHT / A / B no longer navigate the UI — they are free for
//      your subsystem controls in opcontrol().
//    • The auton you selected is preserved — it runs when the field fires.
//
//  COMPETITION WORKFLOW:
//    1. Before the match — tap your auton on the brain screen or select it
//       with the controller.
//    2. Hold UP + X for 1 second → brain screen locks, controller rumbles.
//    3. Field fires autonomous → your selected auton runs.
//    4. Driver control → Pre programed robot functions control your robot subsystems.
//    5. After the match → hold UP + X again to unlock for the next auton pick.
//
//  IMPORTANT: always enter driver mode before the match starts.
//             If you skip this, LEFT/RIGHT/A/B will navigate the controller
//             menu instead of controlling your robot. You can still enter
//             driver mode mid-match incase you want to acces the brain UI.
//
// ── COLOR CONSTANTS ───────────────────────────────────────────────────────────
//
//  UI_GOLD      UI_DARK_GOLD   UI_GREEN     UI_RED      UI_ORANGE
//  UI_BLUE      UI_PURPLE      UI_WHITE     UI_BLACK    UI_GRAY
//  UI_DARK_BG   (or any hex value: 0xRRGGBB)
//
// ── USABLE FUNCTIONS ──────────────────────────────────────────────────────────
//
//  All widget functions are called inside build_screens(). Declare pages first
//  with PageAdd(), then add widgets to them in any order. Call PageShow() last.
//
//  PAGES
//    BgColor(color)                      — set the background color for the next PageAdd call
//    PageAdd("name")                     — declare a new page (must be called before adding widgets)
//    PageShow("name")                    — show this page on the brain; call last in build_screens()
//    PageAnim(style)                     — UI_ANIM_NONE / UI_ANIM_FADE / UI_ANIM_SLIDE
//                                          call once before your first PageAdd; applies to all pages
//    PageClear("name")                   — remove all widgets from a page so it can be rebuilt;
//                                          also resets the page background to its original color
//    PageBgColor("name", color)          — change a page's background color at runtime
//                                          (PageClear resets it back to the original)
//
//  BUTTONS
//    ButtonAdd("page", x, y, w, h, color, "text", "goes_to", animated, auton, radius, on_tap)
//      goes_to  — page name to navigate to when tapped, "popup:name" to open a popup, or nullptr
//      animated — UI_ELEM_NONE / UI_ELEM_GROW / UI_ELEM_FADE / UI_ELEM_SLIDE / UI_ELEM_DROP
//      auton    — index returned by get_selected_auton() when this button is tapped;
//                 use -1 for non-auton buttons (tabs, back buttons, etc.)
//      radius   — corner radius: UI_SHAPE_PILL / UI_SHAPE_SHARP / any pixel value (default 8)
//      on_tap   — optional callback called when tapped: static void my_fn() { ... }
//
//  PRESS ANIMATIONS  (call before ButtonAdd; resets to default after)
//    ButtonPressStyle(UI_PRESS_FLASH)    — opacity dip on touch
//    ButtonPressStyle(UI_PRESS_PULSE)    — scale shrink then snap back
//    ButtonPressStyle(UI_PRESS_RIPPLE)   — expanding circle from touch point
//    ButtonPressStyle(UI_PRESS_NONE)     — no animation (default)
//
//  LABELS
//    LabelAdd("page", x, y, "text", font_size, color, animated)
//      font_size — 14 / 18 / 20 / 24 / 48
//      animated  — same options as buttons (UI_ELEM_NONE is default)
//
//  LIVE LABELS  (text auto-updates from a getter function)
//    LiveLabelAdd("page", x, y, getter, interval_ms, font_size, color)
//      getter      — a function that returns const char*:
//                    const char* my_fn() { static char buf[32]; snprintf(buf,32,"Val: %d",val); return buf; }
//      interval_ms — how often the getter is called and the label refreshed
//    BlinkLabelAdd("page", x, y, getter, should_blink, color, blink_color, interval_ms, font_size)
//      should_blink — a function returning bool; label blinks while it returns true
//      e.g. bool is_low() { return master.get_battery_level() < 20; }
//
//  SHAPES
//    BoxAdd("page", x, y, w, h, color, radius, border_color, border_width)
//    CircleAdd("page", x, y, diameter, color, border_color, border_width)
//    SquareAdd("page", x, y, size, color, border_color, border_width)
//    RoundedBoxAdd("page", x, y, size, color, radius, border_color, border_width)
//    TriangleAdd("page", x, y, w, h, color)
//      upward-pointing triangle that fills the w × h bounding box
//
//  POPUPS  (overlay dialogs that appear on top of any page)
//    PopupAdd("name", w, h, "Title", bg_color, border_color)
//    PopupLabelAdd("popup", x, y, getter, interval_ms, font_size, color)
//      live-refreshing label inside a popup (updates while the popup is visible)
//    goes_to = "popup:name"   — open a popup from a button
//    goes_to = "close"        — close the popup (use on buttons inside the popup)
//
//  BUTTON GRID
//    GridAdd("page", x, y, total_w, total_h, cols, items[], item_count, gap, animated, radius)
//      auto-sizes and positions item_count buttons in a grid of `cols` columns
//      items[] is an array of ui_btn_item: { color, "text", auton_idx, "goes_to", on_tap }
//      declare it like:
//        ui_btn_item items[] = {
//          { UI_GOLD,  "Auton 1", 0, "auton_1", nullptr },
//          { UI_GREEN, "Auton 2", 1, "auton_2", nullptr },
//          { UI_BLUE,  "Skills",  2, "auton_3", nullptr },
//        };
//        GridAdd("page", x, y, w, h, cols, items, 3, gap, UI_ELEM_GROW, 8);
//
//  TOGGLE BUTTON
//    ToggleAdd("page", x, y, w, h, color_off, color_on, "off text", "on text",
//                  animated, radius, on_toggle, init_state)
//      on_toggle(bool state) — called with the new state on every tap
//      init_state            — true to start in the ON position (default false)
//
//  PROGRESS BAR
//    BarAdd("page", x, y, w, h, getter, interval_ms, fill_color, bg_color,
//                radius, warn_color, warn_threshold)
//      getter returns 0–100; bar turns warn_color when value is at or below warn_threshold
//
//  LIVE COLOR DOT
//    DotAdd("page", x, y, diameter, getter, interval_ms)
//      getter returns a UI_* color — e.g. UI_GREEN when OK, UI_RED when a device faults
//
//  SLIDER
//    SliderAdd("page", x, y, w, h, min, max, default_val, color, on_change)
//      on_change(int value) — called on every drag movement with the current value
//      current value is displayed automatically to the right of the slider
//
//  COUNTDOWN TIMER
//    CountdownAdd("page", x, y, seconds, font_size, color, warn_color, warn_secs, on_expire)
//      displays MM:SS; starts paused. Call CountdownStart() to begin.
//    CountdownStart()           — start or restart from full duration
//    CountdownStop()            — pause/stop the countdown
//    CountdownRemaining()       — whole seconds remaining (0 if not started or expired)
//
//  SPINNER
//    SpinnerAdd("page", x, y, size, color, speed_ms, arc_deg)
//      animated loading arc
//      speed_ms — ms per full revolution (lower = faster, default 1200)
//      arc_deg  — arc length in degrees 1–360 (default 75)
//
//  FIELD MAP
//    FieldMapAdd("page", x, y, size, positions[], count, field_color, border_color, marker_size)
//      draws a square field with circular auton-selector markers on top
//      size        — width AND height of the square field in pixels
//      positions[] — array of ui_field_pos:
//                    { field_x, field_y, color, "label", auton_idx, "goes_to", on_tap }
//                    field_x / field_y are 0.0-1.0 (0.0 = left/top, 1.0 = right/bottom)
//      count       — number of entries in positions[]
//      draw game zones with BoxAdd() BEFORE this call so markers render on top
//
//  CONTROLLER SCREEN  (3 rows, ~15 chars visible — the VEX LCD is narrower than the 19-char API limit)
//
//    handle_ctrl_input() is already wired into opcontrol() in main.cpp and runs
//    once per loop tick — do not call it yourself.
//
//    On the first opcontrol tick it automatically reasserts the home screen,
//    overriding any EZ-Template startup text left on the controller.
//
//    Calling CtrlLabel() on a row automatically cancels any CtrlLive() running
//    on the same row, so switching pages always shows the correct static text.
//
//    ── What to customize ─────────────────────────────────────────────────────
//
//    _ctrl_home()
//      The default screen shown on startup and when pressing B from any auton page.
//      → Row 0: change "Custom Brain UI" to your team number or robot name.
//      → Row 1: live brain battery — keep or replace with any getter.
//      → Row 2: navigation hint — tells the driver what LEFT/RIGHT does.
//
//    _ctrl_nav()
//      Shown when the driver presses LEFT or RIGHT from the home screen.
//      → Edit the hint text to match your preferred button labels.
//
//    _ctrl_auton(int idx)
//      One screen per auton — cycles with LEFT/RIGHT, selected with A.
//      → Update the names[] array to match your auton button names in build_screens().
//      → To add a 4th auton: add CTRL_A3 to the enum below, a matching case in
//        the switch inside handle_ctrl_input(), and case 3 in autonomous() in main.cpp.
//
//    ── State map ─────────────────────────────────────────────────────────────
//
//      CTRL_HOME    — default: robot name + live battery + nav hint
//      CTRL_NAV     — auton select entry (LEFT or RIGHT from HOME)
//      CTRL_A0/1/2  — one screen per auton (A from NAV, then LEFT/RIGHT to cycle)
//
//    ── Button map ────────────────────────────────────────────────────────────
//
//      LEFT / RIGHT        — switch between HOME and NAV
//      A  (on NAV)         — enter auton pages (goes to CTRL_A0)
//      LEFT / RIGHT        — cycle between auton pages
//      A  (on auton page)  — select that auton + navigate brain screen to it
//      B  (on auton page)  — return to HOME
//      UP + X (hold 1s)    — toggle driver mode (locks brain screen for the match)
//
//    ── Ctrl* functions (usable anywhere, not just in handle_ctrl_input) ──────
//
//    CtrlLabel(row, "text")              — write static text to row 0 / 1 / 2
//                                          also cancels any CtrlLive on that row
//    CtrlLabelFmt(row, "Bat:%d%%", val) — printf-style static text; also cancels CtrlLive
//    CtrlLive(row, getter, interval_ms) — auto-refreshing row from a getter function
//    CtrlClear(row)                      — clear a row and cancel its live timer;
//                                          pass -1 to clear all three rows
//    CtrlFlush()                         — immediately write all live rows without waiting
//    CtrlRumble("pattern")               — vibrate the controller
//                                          '.' = short  '-' = long  ' ' = pause
//                                          example: "..-" = short, short, long
//
//  ENGINE
//    EngineDriverMode(true / false) — lock or unlock brain screen touch input
//                                     true  = locked ("MATCH IN PROGRESS"; touch fully disabled)
//                                     false = unlocked (interactive)
//                                     Already called inside handle_ctrl_input() on UP+X.
//                                     Call it directly if you want to lock from other code.
//
//    ForceSelectAuton(idx)          — set the active auton from code instead of a screen tap
//                                     idx matches the auton number you gave ButtonAdd
//                                     Use this when the driver picks autons from the controller:
//                                       ForceSelectAuton(0);   // lock in auton 0
//                                       PageShow("auton_1");   // then navigate the brain screen
//                                     get_selected_auton() returns whichever idx was last set
//                                     (by a screen tap or by ForceSelectAuton).
//
// ══════════════════════════════════════════════════════════════════════════════



// ── Demo getter / helper functions ────────────────────────────────────────────

// ══════════════════════════════════════════════════════════════════════════════
//  DRIVER MODE TOGGLE — change these two buttons to fit your robot
// ══════════════════════════════════════════════════════════════════════════════
//
//  Both buttons must be held TOGETHER for DRIVER_MODE_HOLD_MS before the mode
//  flips.  A quick tap of either one does NOTHING, so both buttons stay fully
//  usable for your subsystems.  You only give up the specific case of holding
//  both at the same time for a full second.
//
//  The controller rumbles to confirm:  "-" entering driver mode, "." leaving it.
//
//  Any two DIGITAL_* buttons work.  Pick a pair your driver would never hold
//  together for a second in the middle of a match.
//
static const pros::controller_digital_e_t DRIVER_MODE_BTN_A   = DIGITAL_UP;
static const pros::controller_digital_e_t DRIVER_MODE_BTN_B   = DIGITAL_X;
static const int                          DRIVER_MODE_HOLD_MS = 1000;

// Returns true exactly once per completed hold.  The buttons must be released
// before it can fire again, so holding them does not toggle repeatedly.
static bool _driver_mode_combo_fired() {
  static int  held_ms       = 0;
  static bool already_fired = false;

  if (master.get_digital(DRIVER_MODE_BTN_A) && master.get_digital(DRIVER_MODE_BTN_B)) {
    held_ms += ez::util::DELAY_TIME;   // handle_ctrl_input() runs once per opcontrol tick
    if (held_ms >= DRIVER_MODE_HOLD_MS && !already_fired) {
      already_fired = true;
      return true;
    }
  } else {
    held_ms       = 0;
    already_fired = false;
  }
  return false;
}

const char* battery_text() {
  static char buf[20];
  snprintf(buf, sizeof(buf), "Bat: %d%%", (int)pros::battery::get_capacity());
  return buf;
}


int get_selected_auton() { return SelectedAuton(); }

// Live readout of which auton is currently selected, so a tap on the brain (or
// an A press on the controller) has visible confirmation.  Names must match the
// ButtonAdd labels on "auton_tab".
// Live IMU heading for the popup on each auton page.
const char* imu_text() {
  static char buf[16];
  snprintf(buf, sizeof(buf), "%.1f deg", chassis.drive_imu_get());
  return buf;
}

// Zeroes the IMU.  Wired to the Reset button inside that popup.
static void do_imu_reset() { chassis.drive_imu_reset(); }

const char* selected_auton_text() {
  static char buf[32];
  static const char* names[] = { "Auto 1", "Auto 2", "Auto 3", "Auto 4", "Auto 5" };
  int idx = SelectedAuton();
  if (idx < 0 || idx >= (int)(sizeof(names) / sizeof(names[0])))
    snprintf(buf, sizeof(buf), "Selected: none");
  else
    snprintf(buf, sizeof(buf), "Selected: %s", names[idx]);
  return buf;
}

const char* ctrl_battery_text() {
  static char buf[20];
  // get_battery_capacity() is the PERCENTAGE - the controller-side analog of
  // pros::battery::get_capacity().  get_battery_level() is a different field
  // and does not report a percentage, which is why this used to read wrong.
  int pct = master.get_battery_capacity();
  if (pct < 0) pct = 0;   // PROS_ERR when the controller is not connected
  snprintf(buf, sizeof(buf), "Ctrl: %d%%", pct);
  return buf;
}


/* ═══════════════════════════════════════════════════════════════════════════════
   COMPETITION TEMPLATE — two-tab auton selector with controller state machine.

   HOW TO SWITCH FROM DEMO:
     1. Delete the entire DELETE block above (from the opening ╔ box to "END DELETE").
        Keep: battery_text(), get_selected_auton(), ctrl_battery_text() above.
     2. Uncomment the block below (remove the slash-star on the lower line and the star-slash
        at the very bottom of the file).
     3. Edit auton names, team number, and controller strings to match your robot.
     4. Add your actual auton functions to autonomous() in main.cpp.

   CONTROLLER STATE MACHINE
   ─────────────────────────────────────────────────────────────────────────────
   The menu has 5 states:
     CTRL_HOME    — startup: robot name + live battery + nav hint
     CTRL_NAV     — auton select entry (LEFT or RIGHT from HOME)
     CTRL_A0/1/2  — one screen per auton (A from NAV, then LEFT/RIGHT to cycle)

   handle_ctrl_input() is called once per opcontrol loop tick. It reads buttons,
   advances the state, and updates the controller display. Nothing blocks.

   DRIVER MODE (UP + X)
   ─────────────────────────────────────────────────────────────────────────────
   Locks the brain screen and frees LEFT/RIGHT/A/B for robot subsystems.
   Toggle with UP + X (either order). Same combo exits.
   ═══════════════════════════════════════════════════════════════════════════════
*/



enum _CtrlState { CTRL_HOME, CTRL_NAV, CTRL_A0, CTRL_A1, CTRL_A2, CTRL_A3, CTRL_A4 };

// ── What the controller shows in each state ──────────────────────────────────
//  Keep strings ≤ 15 chars — the VEX LCD is narrower than the 19-char API limit.

static void _ctrl_home() {
  CtrlLabel(0, "Custom Brain UI");  // ← change to your team/robot name
  CtrlLabel(1, battery_text());
  CtrlLabel(2, "(< >) Nxt pg");
}

static void _ctrl_nav() {
  CtrlLabel(0, "Auton Select");
  CtrlLabel(1, "(A)enter");
  CtrlLabel(2, "(< >)Nxt pg");
}

static void _ctrl_auton(int idx) {
  static const char* names[] = { "Auto 1", "Auto 2", "Auto 3", "Auto 4", "Auto 5" };  // match your button names
  CtrlLabel(0, names[idx]);
  CtrlLabel(1, "(A)sel  (B)back");
  CtrlLabel(2, "(< >) Nxt Auton");
}

// Controller confirmation after A selects an auton.  Without this the display is
// identical before and after the press, so there is no way to tell it registered.
static void _ctrl_selected(int idx) {
  static const char* names[] = { "Auto 1", "Auto 2", "Auto 3", "Auto 4", "Auto 5" };
  CtrlLabel(0, names[idx]);
  CtrlLabel(1, "** SELECTED **");
  CtrlLabel(2, "(< >) Nxt Auton");
  CtrlRumble(".");            // short buzz so you feel it too
}


// handle_ctrl_input() is called once per opcontrol loop tick from main.cpp.
// It reads buttons and advances the state machine, updating the controller
// display whenever the state changes. Nothing here blocks — it just reads
// inputs, updates state, and returns immediately every tick.
void handle_ctrl_input() {
  // static = these survive across calls; they hold the current menu state.
  static _CtrlState ctrl_state  = CTRL_HOME;
  static bool       driver_mode = false;
  static bool       first_call  = true;

  // On the very first tick of opcontrol, paint the home screen. Without this
  // the controller shows whatever EZ-Template left from autonomous/disabled.
  if (first_call) {
    first_call = false;
    _ctrl_home();
  }

  // ── Driver mode toggle ────────────────────────────────────────────────────
  // _driver_mode_combo_fired() requires DRIVER_MODE_BTN_A and DRIVER_MODE_BTN_B
  // to be held together for DRIVER_MODE_HOLD_MS.  A quick tap of either button
  // does nothing, so both stay usable for your subsystems.  Change the buttons
  // at the top of this file.
  if (_driver_mode_combo_fired()) {
    driver_mode = !driver_mode;
    EngineDriverMode(driver_mode);  // locks or unlocks brain screen touch input
    CtrlRumble(driver_mode ? "-" : ".");
    if (driver_mode) {
      CtrlLabel(0, "* DRIVER MODE *");  // confirm to driver that mode is active
      CtrlLabel(2, "hold UP+X 1s");
    } else {
      ctrl_state = CTRL_HOME;  // always return to home when exiting driver mode
      _ctrl_home();
    }
  }

  // ── Menu navigation ───────────────────────────────────────────────────────
  // Blocked while driver mode is active so LEFT/RIGHT/A/B are free for
  // your subsystem code in opcontrol() (intake, arm, etc.).
  if (!driver_mode) {
    // new_press means the action fires once per button press, not every tick.
    bool left_new  = master.get_digital_new_press(DIGITAL_LEFT);
    bool right_new = master.get_digital_new_press(DIGITAL_RIGHT);
    bool a_new     = master.get_digital_new_press(DIGITAL_A);
    bool b_new     = master.get_digital_new_press(DIGITAL_B);

    switch (ctrl_state) {

      // HOME: only LEFT/RIGHT does anything — opens the auton-select entry screen.
      case CTRL_HOME:
        if (left_new || right_new) { ctrl_state = CTRL_NAV; _ctrl_nav(); }
        break;

      // NAV: LEFT/RIGHT exits back to home; A dives into the first auton page.
      case CTRL_NAV:
        if (left_new || right_new) { ctrl_state = CTRL_HOME; _ctrl_home(); }
        if (a_new) { ctrl_state = CTRL_A0; _ctrl_auton(0); }
        break;

      // AUTON PAGES: LEFT/RIGHT cycles (wraps at the ends).
      //   A  = ForceSelectAuton(idx) locks in the choice for autonomous(),
      //        then PageShow() navigates the brain screen to that auton's detail page.
      //   B  = bail back to home without changing the auton selection.
      case CTRL_A0:
        if (left_new)  { ctrl_state = CTRL_A4; _ctrl_auton(4); }
        if (right_new) { ctrl_state = CTRL_A1; _ctrl_auton(1); }
        if (a_new)     { ForceSelectAuton(0); PageShow("auton_1"); _ctrl_selected(0); }
        if (b_new)     { ctrl_state = CTRL_HOME; _ctrl_home(); }
        break;
      case CTRL_A1:
        if (left_new)  { ctrl_state = CTRL_A0; _ctrl_auton(0); }
        if (right_new) { ctrl_state = CTRL_A2; _ctrl_auton(2); }
        if (a_new)     { ForceSelectAuton(1); PageShow("auton_2"); _ctrl_selected(1); }
        if (b_new)     { ctrl_state = CTRL_HOME; _ctrl_home(); }
        break;
      case CTRL_A2:
        if (left_new)  { ctrl_state = CTRL_A1; _ctrl_auton(1); }
        if (right_new) { ctrl_state = CTRL_A3; _ctrl_auton(3); }
        if (a_new)     { ForceSelectAuton(2); PageShow("auton_3"); _ctrl_selected(2); }
        if (b_new)     { ctrl_state = CTRL_HOME; _ctrl_home(); }
        break;
      case CTRL_A3:
        if (left_new)  { ctrl_state = CTRL_A2; _ctrl_auton(2); }
        if (right_new) { ctrl_state = CTRL_A4; _ctrl_auton(4); }
        if (a_new)     { ForceSelectAuton(3); PageShow("auton_4"); _ctrl_selected(3); }
        if (b_new)     { ctrl_state = CTRL_HOME; _ctrl_home(); }
        break;
      case CTRL_A4:
        if (left_new)  { ctrl_state = CTRL_A3; _ctrl_auton(3); }
        if (right_new) { ctrl_state = CTRL_A0; _ctrl_auton(0); }
        if (a_new)     { ForceSelectAuton(4); PageShow("auton_5"); _ctrl_selected(4); }
        if (b_new)     { ctrl_state = CTRL_HOME; _ctrl_home(); }
        break;
    }
  }
}

void build_screens() {
  BgColor(UI_DARK_BG);
  PageAnim(UI_ANIM_FADE);  // all page transitions fade in

  // Declare all pages first — widgets can be added in any order after this.
  PageAdd("auton_tab");   // main auton picker (shown first)
  PageAdd("status_tab");  // battery / team info tab
  PageAdd("auton_1");     // detail page for Auto 1
  PageAdd("auton_2");     // detail page for Auto 2
  PageAdd("auton_3");     // detail page for Auto 3
  PageAdd("auton_4");     // detail page for Auto 4
  PageAdd("auton_5");     // detail page for Auto 5

  // ── IMU popup - declared once, opened from every auton page ────────────────
  PopupAdd(     "imu", 300, 180, "IMU Position", UI_DARK_BG, UI_GREEN);
  PopupLabelAdd("imu",  85,  62, imu_text, 100, 24, UI_WHITE);   // refreshes 10x/sec
  ButtonAdd(    "imu",  20, 120, 120, 40, UI_ORANGE, "Reset",
                nullptr, UI_ELEM_NONE, -1, 8, do_imu_reset);     // stays open so you
                                                                 // see it hit 0.0
  ButtonAdd(    "imu", 160, 120, 120, 40, UI_GREEN,  "Close", "close");

  // ── Auton Selector tab ─────────────────────────────────────────────────────
  // Tab bar: two buttons side-by-side at the top (y=0, h=34).
  // Active tab = lighter background + gold underline bar.
  BoxAdd(   "auton_tab",   0,  0, 480, 34, UI_DARK_BG, 0);
  ButtonAdd("auton_tab",   0,  0, 240, 34, 0x252525, "Auton Selector", nullptr);
  ButtonAdd("auton_tab", 240,  0, 240, 34, UI_DARK_BG, "Robot Status", "status_tab");
  BoxAdd(   "auton_tab",   0, 31, 240,  3, UI_GOLD, 0);  // gold underline = active tab
  BoxAdd(   "auton_tab",   0, 33, 480,  1, UI_GRAY, 0);  // full-width separator

  // Three auton buttons — auton_idx (last number) must match case N in autonomous().
  ButtonPressStyle(UI_PRESS_RIPPLE);
  ButtonAdd("auton_tab",   5, 44, 89, 158, UI_GOLD,    "Auto 1", "auton_1", UI_ELEM_GROW, 0);
  ButtonAdd("auton_tab", 100, 44, 89, 158, UI_GOLD,    "Auto 2", "auton_2", UI_ELEM_GROW, 1);
  ButtonAdd("auton_tab", 195, 44, 89, 158, UI_GOLD,    "Auto 3", "auton_3", UI_ELEM_GROW, 2);
  ButtonAdd("auton_tab", 290, 44, 89, 158, UI_GOLD,    "Auto 4", "auton_4", UI_ELEM_GROW, 3);
  ButtonAdd("auton_tab", 385, 44, 89, 158, UI_GOLD,    "Auto 5", "auton_5", UI_ELEM_GROW, 4);
  ButtonPressStyle(UI_PRESS_NONE);

  // Selection confirmation, under the three buttons
  LiveLabelAdd("auton_tab", 10, 208, selected_auton_text, 200, 18, UI_GREEN);

  // ── Robot Status tab ───────────────────────────────────────────────────────
  BoxAdd(   "status_tab",   0,  0, 480, 34, UI_DARK_BG, 0);
  ButtonAdd("status_tab",   0,  0, 240, 34, UI_DARK_BG, "Auton Selector", "auton_tab");
  ButtonAdd("status_tab", 240,  0, 240, 34, 0x252525, "Robot Status", nullptr);
  BoxAdd(   "status_tab", 240, 31, 240,  3, UI_GOLD, 0);  // underline on active tab
  BoxAdd(   "status_tab",   0, 33, 480,  1, UI_GRAY, 0);

  LabelAdd("status_tab",  20, 50, "Team 474G", 24, UI_WHITE);  // ← change to your team number
  LabelAdd("status_tab",  20, 88, "Brain Battery:", 18, UI_GRAY);
  LiveLabelAdd("status_tab", 200, 88, battery_text, 500, 18, UI_GREEN);
  LabelAdd("status_tab",  20, 116, "Controller:", 18, UI_GRAY);
  LiveLabelAdd("status_tab", 160, 116, ctrl_battery_text, 1000, 18, UI_GREEN);

  // ── Auton detail pages — add your route description, field map, notes, etc. ─
  ButtonAdd("auton_1", 10, 10, 80, 32, UI_GOLD, "< Back", "auton_tab");
  LabelAdd( "auton_1", 130, 18, "Auto 1", 20, UI_WHITE);
  ButtonAdd("auton_1", 390, 10, 80, 32, UI_GREEN, "IMU", "popup:imu");
  BoxAdd(   "auton_1",   0, 52, 480,   2, UI_GRAY, 0);
  // Gold panel filling the page body.  Added BEFORE the label so the label
  // draws on top of it - LVGL renders in creation order.
  BoxAdd(   "auton_1",  10, 62, 460, 168, UI_GOLD, 12);
  LabelAdd( "auton_1", 140, 137, "Put auton info here", 18, UI_BLACK);

  ButtonAdd("auton_2", 10, 10, 80, 32, UI_GOLD, "< Back", "auton_tab");
  LabelAdd( "auton_2", 130, 18, "Auto 2", 20, UI_WHITE);
  ButtonAdd("auton_2", 390, 10, 80, 32, UI_GREEN, "IMU", "popup:imu");
  BoxAdd(   "auton_2",   0, 52, 480,   2, UI_GRAY, 0);
  // Gold panel filling the page body.  Added BEFORE the label so the label
  // draws on top of it - LVGL renders in creation order.
  BoxAdd(   "auton_2",  10, 62, 460, 168, UI_GOLD, 12);
  LabelAdd( "auton_2", 140, 137, "Put auton info here", 18, UI_BLACK);

  ButtonAdd("auton_3", 10, 10, 80, 32, UI_GOLD, "< Back", "auton_tab");
  LabelAdd( "auton_3", 130, 18, "Auto 3", 20, UI_WHITE);
  ButtonAdd("auton_3", 390, 10, 80, 32, UI_GREEN, "IMU", "popup:imu");
  BoxAdd(   "auton_3",   0, 52, 480,   2, UI_GRAY, 0);
  // Gold panel filling the page body.  Added BEFORE the label so the label
  // draws on top of it - LVGL renders in creation order.
  BoxAdd(   "auton_3",  10, 62, 460, 168, UI_GOLD, 12);
  LabelAdd( "auton_3", 140, 137, "Put auton info here", 18, UI_BLACK);

  ButtonAdd("auton_4", 10, 10, 80, 32, UI_GOLD, "< Back", "auton_tab");
  LabelAdd( "auton_4", 130, 18, "Auto 4", 20, UI_WHITE);
  ButtonAdd("auton_4", 390, 10, 80, 32, UI_GREEN, "IMU", "popup:imu");
  BoxAdd(   "auton_4",   0, 52, 480,   2, UI_GRAY, 0);
  // Gold panel filling the page body.  Added BEFORE the label so the label
  // draws on top of it - LVGL renders in creation order.
  BoxAdd(   "auton_4",  10, 62, 460, 168, UI_GOLD, 12);
  LabelAdd( "auton_4", 140, 137, "Put auton info here", 18, UI_BLACK);

  ButtonAdd("auton_5", 10, 10, 80, 32, UI_GOLD, "< Back", "auton_tab");
  LabelAdd( "auton_5", 130, 18, "Auto 5", 20, UI_WHITE);
  ButtonAdd("auton_5", 390, 10, 80, 32, UI_GREEN, "IMU", "popup:imu");
  BoxAdd(   "auton_5",   0, 52, 480,   2, UI_GRAY, 0);
  // Gold panel filling the page body.  Added BEFORE the label so the label
  // draws on top of it - LVGL renders in creation order.
  BoxAdd(   "auton_5",  10, 62, 460, 168, UI_GOLD, 12);
  LabelAdd( "auton_5", 140, 137, "Put auton info here", 18, UI_BLACK);

  PageShow("auton_tab");
}

