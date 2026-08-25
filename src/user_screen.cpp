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
//  □  Select auton then press UP + X on the controller before every match to lock the brain screen
//
// ── DRIVER MODE  ★ READ THIS BEFORE COMPETING ★ ──────────────────────────────
//
//  Driver mode locks the brain screen and frees LEFT/RIGHT/A/B on the controller
//  for robot subsystems (intake, arm, claw, etc.) during a match.
//
//  HOW TO TOGGLE:  Hold UP and press X (either order). Same combo exits.
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
//    2. Press UP + X → brain screen locks, controller confirms.
//    3. Field fires autonomous → your selected auton runs.
//    4. Driver control → Pre programed robot functions control your robot subsystems.
//    5. After the match → UP + X again to unlock for the next auton pick.
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
//      UP + X              — toggle driver mode (locks brain screen for the match)
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

const char* battery_text() {
  static char buf[20];
  snprintf(buf, sizeof(buf), "Bat: %d%%", (int)pros::battery::get_capacity());
  return buf;
}

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  DELETE EVERYTHING FROM HERE DOWN TO THE "END DELETE" LINE BELOW             ║
// ║  when switching from demo to a competition auton selector.                   ║
// ║  Keep: battery_text() above, get_selected_auton() and the AUTON SELECTOR     ║
// ║  EXAMPLE block below.                                                        ║
// ║  This code below is just a demo of the Brain UI features and is not          ║
// ║  inteded for competition use.                                                ║
// ╚══════════════════════════════════════════════════════════════════════════════╝

int      battery_pct()       { return (int)pros::battery::get_capacity(); }
uint32_t battery_dot_color() { return pros::battery::get_capacity() > 20.0 ? UI_GREEN : UI_RED; }
bool     battery_low()       { return pros::battery::get_capacity() <= 20.0; }

static void do_ctrl_rumble()   { CtrlRumble(".--"); }
static void do_ctrl_clear_0()  { CtrlClear(0); }
static void do_ctrl_clear_1()  { CtrlClear(1); }
static void do_ctrl_clear_2()  { CtrlClear(2); }

static int  _demo_delay = 500;
static void on_demo_delay(int val) { _demo_delay = val; }

static int  _demo_speed = 80;
static void on_demo_speed(int val) { _demo_speed = val; }

static bool _toggle_state = false;
static void on_demo_toggle(bool s) { _toggle_state = s; }

static void start_demo_countdown() { CountdownStart(); }
static void stop_demo_countdown()  { CountdownStop();  }

// ── per-feature rebuild callbacks ─────────────────────────────────────────────

static void show_buttons() {
  PageClear("detail");
  ButtonAdd("detail", 10,  10,  80, 32, UI_GRAY,   "< Back",   "menu");
  LabelAdd( "detail", 200, 14,  "ButtonAdd",        18, UI_WHITE);
  ButtonAdd("detail",  10, 55, 100, 50, UI_GOLD,   "Default");
  ButtonAdd("detail", 120, 55, 100, 50, UI_GREEN,  "Pill",     nullptr, UI_ELEM_NONE, -1, UI_SHAPE_PILL);
  ButtonAdd("detail", 230, 55, 100, 50, UI_BLUE,   "Sharp",    nullptr, UI_ELEM_NONE, -1, UI_SHAPE_SHARP);
  ButtonAdd("detail", 340, 55, 130, 50, UI_ORANGE, "Animated", nullptr, UI_ELEM_GROW);
  LabelAdd("detail",  18, 112, "radius: 8",      14, UI_GRAY);
  LabelAdd("detail", 122, 112, "UI_SHAPE_PILL",  14, UI_GRAY);
  LabelAdd("detail", 232, 112, "UI_SHAPE_SHARP", 14, UI_GRAY);
  LabelAdd("detail", 342, 112, "UI_ELEM_GROW",   14, UI_GRAY);
  LabelAdd("detail", 10, 136, "Press Animations — tap each:", 14, UI_GRAY);
  ButtonPressStyle(UI_PRESS_FLASH);
  ButtonAdd("detail",  10, 155, 130, 48, UI_GOLD,  "Flash");
  ButtonPressStyle(UI_PRESS_PULSE);
  ButtonAdd("detail", 175, 155, 130, 48, UI_GREEN, "Pulse");
  ButtonPressStyle(UI_PRESS_RIPPLE);
  ButtonAdd("detail", 340, 155, 130, 48, UI_BLUE,  "Ripple");
  ButtonPressStyle(UI_PRESS_NONE);
  LabelAdd("detail",  18, 210, "UI_PRESS_FLASH",  14, UI_GRAY);
  LabelAdd("detail", 183, 210, "UI_PRESS_PULSE",  14, UI_GRAY);
  LabelAdd("detail", 348, 210, "UI_PRESS_RIPPLE", 14, UI_GRAY);
}

static void show_labels() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 200, 14, "LabelAdd", 18, UI_WHITE);
  LabelAdd("detail",  20,  50, "Font size 14",  14, UI_WHITE);
  LabelAdd("detail",  20,  68, "Font size 18",  18, UI_GOLD);
  LabelAdd("detail",  20,  90, "Font size 20",  20, UI_GREEN);
  LabelAdd("detail",  20, 114, "Font size 24",  24, UI_ORANGE);
  LabelAdd("detail",  20, 143, "48",             48, UI_BLUE);
  LabelAdd("detail", 200, 160, "font_size: 14 / 18 / 20 / 24 / 48", 14, UI_GRAY);
  LabelAdd("detail", 200, 178, "color: any UI_* constant or 0xRRGGBB", 14, UI_GRAY);
}

static void show_live() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 170, 14, "Live & Blink Labels", 18, UI_WHITE);
  LabelAdd(    "detail", 20,  50, "LiveLabelAdd — refreshes automatically:", 14, UI_GRAY);
  LiveLabelAdd("detail", 20,  68, battery_text, 500, 20, UI_GREEN);
  LabelAdd(    "detail", 20,  93, "getter function called every 500 ms",     14, UI_GRAY);
  BoxAdd("detail", 0, 113, 480, 1, UI_GRAY, 0);
  LabelAdd(      "detail", 20, 120, "BlinkLabelAdd — blinks when condition is true:", 14, UI_GRAY);
  BlinkLabelAdd( "detail", 20, 138, battery_text, battery_low, UI_WHITE, UI_RED, 400, 20);
  LabelAdd(      "detail", 20, 166, "blinks red when battery <= 20%", 14, UI_GRAY);
}

static void show_shapes() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 205, 14, "Shapes", 18, UI_WHITE);
  BoxAdd(       "detail",  36, 50, 52, 52, UI_GOLD,   8);
  CircleAdd(    "detail", 124, 50, 52, UI_GREEN);
  SquareAdd(    "detail", 212, 50, 52, UI_BLUE);
  RoundedBoxAdd("detail", 300, 50, 52, UI_ORANGE, 14);
  TriangleAdd(  "detail", 388, 50, 52, 52, UI_PURPLE);
  LabelAdd("detail",  44, 108, "BoxAdd",      14, UI_GRAY);
  LabelAdd("detail", 123, 108, "CircleAdd",   14, UI_GRAY);
  LabelAdd("detail", 211, 108, "SquareAdd",   14, UI_GRAY);
  LabelAdd("detail", 296, 108, "RoundedBox",  14, UI_GRAY);
  LabelAdd("detail", 381, 108, "TriangleAdd", 14, UI_GRAY);
  LabelAdd("detail", 10, 132, "BoxAdd(page, x, y, w, h, color)",                14, UI_GRAY);
  LabelAdd("detail", 10, 150, "CircleAdd / SquareAdd(page, x, y, size, color)", 14, UI_GRAY);
  LabelAdd("detail", 10, 168, "TriangleAdd(page, x, y, w, h, color)",           14, UI_GRAY);
  LabelAdd("detail", 10, 186, "All shapes accept border_color + border_width too.", 14, UI_GRAY);
}

static void show_popup() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 200, 14, "Popups", 18, UI_WHITE);
  LabelAdd( "detail", 20,  55, "PopupAdd creates an overlay dialog.",     18, UI_WHITE);
  LabelAdd( "detail", 20,  78, "PopupLabelAdd adds a live value inside.", 18, UI_WHITE);
  LabelAdd( "detail", 20, 100, "Use goes_to = \"popup:name\" to open it.", 14, UI_GRAY);
  ButtonAdd("detail", 165, 135, 150, 45, UI_GREEN, "Open Popup", "popup:demo_pop");
}

static void show_bar() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 170, 14, "Progress Bar & Color Dot", 18, UI_WHITE);
  LabelAdd("detail",  20,  50, "BarAdd — fills 0-100, warn color below threshold:", 14, UI_GRAY);
  BarAdd(  "detail",  20,  68, 380, 20, battery_pct, 500, UI_GREEN, UI_GRAY, 6, UI_RED, 20);
  LabelAdd("detail",  20,  95, "Turns red when battery <= 20%", 14, UI_GRAY);
  LabelAdd("detail",  20, 135, "DotAdd — small circle whose color comes from a getter:", 14, UI_GRAY);
  DotAdd(  "detail",  20, 158,  24, battery_dot_color, 500);
  LabelAdd("detail",  52, 160, "Green > 20%   Red <= 20%", 18, UI_WHITE);
}

static void show_toggle() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 190, 14, "Toggle Button", 18, UI_WHITE);
  LabelAdd("detail", 20,  52, "Stays highlighted when active.",          18, UI_WHITE);
  LabelAdd("detail", 20,  75, "on_toggle(bool state) fires each tap.",   18, UI_WHITE);
  LabelAdd("detail", 20, 103, "ToggleAdd(page, x, y, w, h,",            14, UI_GRAY);
  LabelAdd("detail", 20, 121, "  color_off, color_on,",                  14, UI_GRAY);
  LabelAdd("detail", 20, 139, "  \"off text\", \"on text\", anim, radius, callback)", 14, UI_GRAY);
  ToggleAdd("detail", 160, 178, 160, 50, UI_GRAY, UI_GREEN, "OFF", "ON", UI_ELEM_GROW, 8, on_demo_toggle, _toggle_state);
}

static void show_grid() {
  static const ui_btn_item GRID_ITEMS[] = {
    { UI_GOLD,   "Item 1", -1, nullptr },
    { UI_GREEN,  "Item 2", -1, nullptr },
    { UI_BLUE,   "Item 3", -1, nullptr },
    { UI_ORANGE, "Item 4", -1, nullptr },
    { UI_PURPLE, "Item 5", -1, nullptr },
    { UI_RED,    "Item 6", -1, nullptr },
  };
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 185, 14, "Button Grid", 18, UI_WHITE);
  LabelAdd("detail", 20, 50, "GridAdd auto-sizes N buttons into columns.", 18, UI_WHITE);
  LabelAdd("detail", 20, 73, "GridAdd(page, x, y, total_w, total_h,",     14, UI_GRAY);
  LabelAdd("detail", 20, 91, "  cols, items[], item_count, gap, anim)",    14, UI_GRAY);
  GridAdd( "detail", 10, 110, 460, 126, 3, GRID_ITEMS, 6, 8, UI_ELEM_GROW);
}

static void show_bg() {
  PageClear("detail");
  PageBgColor("detail", UI_PURPLE);
  ButtonAdd("detail", 10, 10, 80, 32, 0x6A0DAD, "< Back", "menu");
  LabelAdd( "detail", 155, 14, "BG Color & Page Anim", 18, UI_WHITE);
  LabelAdd("detail", 20,  52, "This page uses PageBgColor(\"detail\", UI_PURPLE).", 18, UI_WHITE);
  LabelAdd("detail", 20,  75, "Use BgColor(color) before PageAdd() to bake a",   18, UI_WHITE);
  LabelAdd("detail", 20,  98, "background into a page permanently.",               18, UI_WHITE);
  BoxAdd("detail", 0, 122, 480, 1, 0x9966CC, 0);
  LabelAdd("detail", 20, 130, "PageAnim() sets how pages transition:",   18, UI_WHITE);
  LabelAdd("detail", 20, 155, "  UI_ANIM_NONE   — instant switch",       14, 0xCCAAFF);
  LabelAdd("detail", 20, 173, "  UI_ANIM_FADE   — new page fades in",    14, 0xCCAAFF);
  LabelAdd("detail", 20, 191, "  UI_ANIM_SLIDE  — pages slide sideways", 14, 0xCCAAFF);
}

static void show_controller() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 165, 14, "Controller Screen", 18, UI_WHITE);
  LabelAdd("detail", 20, 48, "The controller has a 3-row text display (19 chars max).", 14, UI_GRAY);
  BoxAdd("detail", 10, 64, 460, 82, 0x0D0D0D, 6, 0x555555, 2);
  LabelAdd("detail", 20,  74, "Row 0:", 14, 0x666666);
  LabelAdd("detail", 82,  74, "Custom Brain UI",          14, 0x88FF88);
  LabelAdd("detail", 20,  94, "Row 1:", 14, 0x666666);
  LabelAdd("detail", 82,  94, "Bat: 87%  (updates live)", 14, 0x88FF88);
  LabelAdd("detail", 20, 114, "Row 2:", 14, 0x666666);
  LabelAdd("detail", 20, 154, "Clear a row — tap to send CtrlClear(row):", 14, UI_GRAY);
  ButtonAdd("detail",  20, 172, 130, 38, UI_RED,    "Clear Row 0", nullptr, UI_ELEM_NONE, -1, 8, do_ctrl_clear_0);
  ButtonAdd("detail", 165, 172, 130, 38, UI_RED,    "Clear Row 1", nullptr, UI_ELEM_NONE, -1, 8, do_ctrl_clear_1);
  ButtonAdd("detail", 310, 172, 130, 38, UI_RED,    "Clear Row 2", nullptr, UI_ELEM_NONE, -1, 8, do_ctrl_clear_2);
  ButtonAdd("detail", 320, 120, 140, 34, UI_ORANGE, "Rumble .--",  nullptr, UI_ELEM_NONE, -1, 8, do_ctrl_rumble);
}

static void show_slider() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 205, 14, "Slider", 18, UI_WHITE);
  LabelAdd("detail", 20, 48, "Drag to pick a value — useful for auton delay, speed limits, etc.", 14, UI_GRAY);
  LabelAdd( "detail",  20,  76, "Auton Delay (ms):", 14, UI_GRAY);
  SliderAdd("detail",  20,  94, 300, 24, 0, 3000, _demo_delay, UI_GOLD, on_demo_delay);
  LabelAdd( "detail",  20, 132, "Speed Limit (%):", 14, UI_GRAY);
  SliderAdd("detail",  20, 150, 300, 24, 0, 100, _demo_speed, UI_GREEN, on_demo_speed);
  LabelAdd("detail", 20, 190, "SliderAdd(page, x, y, w, h, min, max, default, color, on_change)", 14, UI_GRAY);
  LabelAdd("detail", 20, 208, "Store the value in a static int — use it in autonomous().", 14, UI_GRAY);
}

static void show_countdown() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 180, 14, "Countdown", 18, UI_WHITE);
  LabelAdd("detail", 20, 48, "Counts down from a set duration. Turns red at warn_seconds.", 14, UI_GRAY);
  CountdownAdd("detail", 185, 72, 60, 48, UI_WHITE, UI_RED, 10);
  ButtonAdd("detail", 130, 140, 105, 40, UI_GREEN, "Start / Reset", nullptr, UI_ELEM_NONE, -1, 8, start_demo_countdown);
  ButtonAdd("detail", 245, 140, 105, 40, UI_RED,   "Stop",          nullptr, UI_ELEM_NONE, -1, 8, stop_demo_countdown);
  LabelAdd("detail", 20, 194, "CountdownAdd(page, x, y, seconds, font_size, color, warn_color, warn_secs)", 14, UI_GRAY);
  LabelAdd("detail", 20, 212, "CountdownStart()  /  CountdownStop()  /  CountdownRemaining()", 14, UI_GRAY);
}

static void show_spinner() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 195, 14, "Spinner", 18, UI_WHITE);
  LabelAdd("detail", 20, 48, "SpinnerAdd(page, x, y, size, color, speed_ms, arc_deg)", 14, UI_GRAY);
  SpinnerAdd("detail",  30, 78,  60, UI_GOLD,  1200, 75);
  SpinnerAdd("detail", 185, 70,  80, UI_GREEN,  800, 90);
  SpinnerAdd("detail", 355, 62, 100, UI_BLUE,   600, 60);
  LabelAdd("detail",  28, 146, "60px",  14, UI_GRAY);
  LabelAdd("detail", 190, 156, "80px",  14, UI_GRAY);
  LabelAdd("detail", 365, 168, "100px", 14, UI_GRAY);
  LabelAdd("detail",  20, 178, "UI_GOLD  1200ms  75deg", 14, 0xDB9E23);
  LabelAdd("detail",  20, 196, "UI_GREEN  800ms  90deg", 14, UI_GREEN);
  LabelAdd("detail",  20, 214, "UI_BLUE   600ms  60deg", 14, UI_BLUE);
}

// ── menu items ────────────────────────────────────────────────────────────────

static void show_driver_info() {
  PageClear("detail");
  ButtonAdd("detail", 10, 10, 80, 32, UI_GRAY, "< Back", "menu");
  LabelAdd( "detail", 170, 14, "Driver Mode", 18, UI_RED);
  BoxAdd(   "detail", 0, 44, 480, 2, UI_RED, 0);

  LabelAdd("detail", 20, 52, "Toggle: Hold UP + press X  (same combo to exit)", 14, UI_GRAY);

  LabelAdd("detail", 20,  70, "While active:", 14, UI_GRAY);
  LabelAdd("detail", 20,  86, "  Brain screen LOCKED — nothing is touchable",   18, UI_WHITE);
  LabelAdd("detail", 20, 108, "  Controller shows DRIVER MODE",                  18, UI_WHITE);
  LabelAdd("detail", 20, 130, "  Driver controls function rather than UI navigation",      18, UI_WHITE);

  BoxAdd(   "detail", 0, 152, 480, 1, UI_GRAY, 0);

  LabelAdd("detail", 20, 158, "Competition workflow:", 14, UI_GRAY);
  LabelAdd("detail", 20, 174, "1. Tap your auton button on the brain screen",    14, UI_WHITE);
  LabelAdd("detail", 20, 190, "2. Press UP+X  →  driver mode ON, screen locks",  14, UI_WHITE);
  LabelAdd("detail", 20, 206, "3. Field fires  →  selected auton runs",           14, UI_WHITE);
  LabelAdd("detail", 20, 222, "4. Drive!  UP+X again to unlock after the match",  14, UI_GOLD);
}

static const ui_btn_item MENU_ITEMS[] = {
  { UI_GOLD,      "Buttons",     -1, "detail", show_buttons     },
  { UI_BLUE,      "Labels",      -1, "detail", show_labels      },
  { UI_GREEN,     "Live Text",   -1, "detail", show_live        },
  { UI_ORANGE,    "Shapes",      -1, "detail", show_shapes      },
  { UI_PURPLE,    "Popups",      -1, "detail", show_popup       },
  { UI_RED,       "Bar & Dot",   -1, "detail", show_bar         },
  { UI_DARK_GOLD, "Toggle",      -1, "detail", show_toggle      },
  { 0x0055AA,     "Button Grid", -1, "detail", show_grid        },
  { UI_GRAY,      "BG & Anim",   -1, "detail", show_bg          },
  { 0x008888,     "Controller",  -1, "detail", show_controller  },
  { UI_GOLD,      "Slider",      -1, "detail", show_slider      },
  { 0x005588,     "Countdown",   -1, "detail", show_countdown   },
  { UI_DARK_GOLD, "Spinner",     -1, "detail", show_spinner     },
  { 0xCC1111,     "Driver Mode", -1, "detail", show_driver_info },
};

void build_screens() {
  BgColor(UI_WHITE);   PageAdd("home");
  BgColor(UI_DARK_BG); PageAdd("menu");
                       PageAdd("detail");

  // popup declared once — persists across detail reloads
  PopupAdd(     "demo_pop", 300, 170, "Live Battery", UI_DARK_BG, UI_GREEN);
  PopupLabelAdd("demo_pop",  90,  60, battery_text, 200, 24, UI_WHITE);
  ButtonAdd(    "demo_pop", 100, 110, 100, 38, UI_GREEN, "Close", "close");

  // ── home ──────────────────────────────────────────────────────────────────
  LabelAdd("home",  60,  55, "Custom Brain UI Template", 24, UI_BLACK);
  LabelAdd("home",  90,  98, "Tap Continue to explore every",   18, UI_GRAY);
  LabelAdd("home",  90, 120, "UI function available.",          18, UI_GRAY);
  ButtonAdd("home", 190, 178, 100, 42, UI_GOLD, "Continue", "menu", UI_ELEM_FADE);

  // ── menu ──────────────────────────────────────────────────────────────────
  LabelAdd("menu", 148, 8, "Choose a Feature", 20, UI_WHITE);
  BoxAdd("menu", 0, 32, 480, 2, UI_GRAY, 0);
  GridAdd("menu", 5, 38, 470, 198, 3, MENU_ITEMS, 14, 6, UI_ELEM_GROW);

  // detail starts empty; show_* callbacks fill it on navigation

  // ── controller display setup ───────────────────────────────────────────────
  CtrlLabel(0, "Custom Brain UI");
  CtrlLive(1, battery_text, 500);
  CtrlLabel(2, "");

  PageAnim(UI_ANIM_FADE);
  PageShow("home");
}

// Demo stub — handles only the UP+X driver mode toggle so it can be tested.
// The competition template below replaces this with the full navigation state machine.
void handle_ctrl_input() {
  static bool driver_mode = false;
  bool up_held = master.get_digital(DIGITAL_UP);
  bool x_new   = master.get_digital_new_press(DIGITAL_X);
  bool x_held  = master.get_digital(DIGITAL_X);
  bool up_new  = master.get_digital_new_press(DIGITAL_UP);
  if ((up_held && x_new) || (x_held && up_new)) {
    driver_mode = !driver_mode;
    EngineDriverMode(driver_mode);
    if (driver_mode) { CtrlLabel(0, "* DRIVER MODE *"); CtrlLabel(2, "UP+X to exit"); }
    else             { CtrlLabel(0, "Custom Brain UI"); CtrlLive(1, battery_text, 500); CtrlLabel(2, ""); }
  }
}

// ╔══════════════════════════════════════════════════════════════════════════════╗
// ║  END DELETE — keep everything below this line                               ║
// ╚══════════════════════════════════════════════════════════════════════════════╝

int get_selected_auton() { return SelectedAuton(); }

const char* ctrl_battery_text() {
  static char buf[20];
  snprintf(buf, sizeof(buf), "Ctrl: %d%%", master.get_battery_level());
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


/*
// ── Controller state machine ───────────────────────────────────────────────────

enum _CtrlState { CTRL_HOME, CTRL_NAV, CTRL_A0, CTRL_A1, CTRL_A2 };

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
  static const char* names[] = { "Auton 1", "Auton 2", "Skills" };  // match your button names
  CtrlLabel(0, names[idx]);
  CtrlLabel(1, "(A)sel  (B)back");
  CtrlLabel(2, "(< >) Nxt Auton");
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

  // ── Driver mode toggle (UP + X) ───────────────────────────────────────────
  // We require a two-button combo to prevent accidental activation mid-match.
  //   get_digital()           = true every tick the button is held
  //   get_digital_new_press() = true only on the first tick it goes down
  // The combo fires if one button is already held when the other is newly pressed.
  bool up_held = master.get_digital(DIGITAL_UP);
  bool x_new   = master.get_digital_new_press(DIGITAL_X);
  bool x_held  = master.get_digital(DIGITAL_X);
  bool up_new  = master.get_digital_new_press(DIGITAL_UP);

  if ((up_held && x_new) || (x_held && up_new)) {
    driver_mode = !driver_mode;
    EngineDriverMode(driver_mode);  // locks or unlocks brain screen touch input
    if (driver_mode) {
      CtrlLabel(0, "* DRIVER MODE *");  // confirm to driver that mode is active
      CtrlLabel(2, "UP+X to exit");
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
        if (left_new)  { ctrl_state = CTRL_A2; _ctrl_auton(2); }  // wrap to last
        if (right_new) { ctrl_state = CTRL_A1; _ctrl_auton(1); }
        if (a_new)     { ForceSelectAuton(0); PageShow("auton_1"); }
        if (b_new)     { ctrl_state = CTRL_HOME; _ctrl_home(); }
        break;
      case CTRL_A1:
        if (left_new)  { ctrl_state = CTRL_A0; _ctrl_auton(0); }
        if (right_new) { ctrl_state = CTRL_A2; _ctrl_auton(2); }
        if (a_new)     { ForceSelectAuton(1); PageShow("auton_2"); }
        if (b_new)     { ctrl_state = CTRL_HOME; _ctrl_home(); }
        break;
      case CTRL_A2:
        if (left_new)  { ctrl_state = CTRL_A1; _ctrl_auton(1); }
        if (right_new) { ctrl_state = CTRL_A0; _ctrl_auton(0); }  // wrap to first
        if (a_new)     { ForceSelectAuton(2); PageShow("auton_3"); }
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
  PageAdd("auton_1");     // detail page for Auton 1
  PageAdd("auton_2");     // detail page for Auton 2
  PageAdd("auton_3");     // detail page for Skills

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
  ButtonAdd("auton_tab",  50,  66, 380, 41, UI_GOLD,  "Auton 1", "auton_1", UI_ELEM_GROW, 0);
  ButtonAdd("auton_tab",  50, 117, 380, 41, UI_GREEN, "Auton 2", "auton_2", UI_ELEM_GROW, 1);
  ButtonAdd("auton_tab",  50, 168, 380, 41, UI_BLUE,  "Skills",  "auton_3", UI_ELEM_GROW, 2);
  ButtonPressStyle(UI_PRESS_NONE);

  // ── Robot Status tab ───────────────────────────────────────────────────────
  BoxAdd(   "status_tab",   0,  0, 480, 34, UI_DARK_BG, 0);
  ButtonAdd("status_tab",   0,  0, 240, 34, UI_DARK_BG, "Auton Selector", "auton_tab");
  ButtonAdd("status_tab", 240,  0, 240, 34, 0x252525, "Robot Status", nullptr);
  BoxAdd(   "status_tab", 240, 31, 240,  3, UI_GOLD, 0);  // underline on active tab
  BoxAdd(   "status_tab",   0, 33, 480,  1, UI_GRAY, 0);

  LabelAdd("status_tab",  20, 50, "Team XXXX", 24, UI_WHITE);  // ← change to your team number
  LabelAdd("status_tab",  20, 88, "Brain Battery:", 18, UI_GRAY);
  LiveLabelAdd("status_tab", 200, 88, battery_text, 500, 18, UI_GREEN);
  LabelAdd("status_tab",  20, 116, "Controller:", 18, UI_GRAY);
  LiveLabelAdd("status_tab", 160, 116, ctrl_battery_text, 1000, 18, UI_GREEN);

  // ── Auton detail pages — add your route description, field map, notes, etc. ─
  ButtonAdd("auton_1", 10, 10, 80, 32, UI_GOLD,  "< Back", "auton_tab");
  LabelAdd( "auton_1", 130, 18, "Auton 1", 20, UI_WHITE);
  BoxAdd(   "auton_1",   0, 52, 480,  2, UI_GRAY, 0);
  LabelAdd( "auton_1", 140, 137, "Put auton info here", 18, UI_GRAY);

  ButtonAdd("auton_2", 10, 10, 80, 32, UI_GREEN, "< Back", "auton_tab");
  LabelAdd( "auton_2", 130, 18, "Auton 2", 20, UI_WHITE);
  BoxAdd(   "auton_2",   0, 52, 480,  2, UI_GRAY, 0);
  LabelAdd( "auton_2", 140, 137, "Put auton info here", 18, UI_GRAY);

  ButtonAdd("auton_3", 10, 10, 80, 32, UI_BLUE,  "< Back", "auton_tab");
  LabelAdd( "auton_3", 130, 18, "Skills",  20, UI_WHITE);
  BoxAdd(   "auton_3",   0, 52, 480,  2, UI_GRAY, 0);
  LabelAdd( "auton_3", 140, 137, "Put auton info here", 18, UI_GRAY);

  PageShow("auton_tab");
}

═══════════════════════════════════════════════════════════════════════════════ */
