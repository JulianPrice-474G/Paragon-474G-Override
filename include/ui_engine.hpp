#pragma once
#include "main.h"

// ═══════════════════════════════════════════════════════════════════════════════
//  UI Engine — Brain Screen API for VEX V5
//
//  Edit src/user_screen.cpp to build your screen.
//  All raw LVGL code is hidden in src/ui_engine.cpp — never touch that file.
//
//  Screen: 480 wide × 272 tall pixels   top-left = (0, 0)
// ═══════════════════════════════════════════════════════════════════════════════

// ── Button shape presets ───────────────────────────────────────────────────────
// Pass to the corner_radius parameter of ButtonAdd / CreateButton.
#define UI_SHAPE_SHARP  0    // no rounding — square corners
#define UI_SHAPE_PILL  -1    // fully rounded ends (radius = height / 2)
// Any positive integer sets the corner radius directly (e.g. 4, 8, 16).

// ── Page transition presets ────────────────────────────────────────────────────
// Pass to PageAnim() in build_screens().
#define UI_ANIM_NONE   0    // instant switch (default)
#define UI_ANIM_FADE   1    // new page fades in
#define UI_ANIM_SLIDE  2    // pages slide left/right

// ── Element animation styles ───────────────────────────────────────────────────
// Pass to the animated parameter of ButtonAdd() or LabelAdd().
// false / 0  = UI_ELEM_NONE  (no animation)
// true  / 1  = UI_ELEM_GROW  (same as before — backwards compatible)
#define UI_ELEM_NONE   0    // no animation (same as false)
#define UI_ELEM_GROW   1    // button grows in from zero width (same as true)
#define UI_ELEM_FADE   2    // fades in from transparent
#define UI_ELEM_SLIDE  3    // slides in from the left edge
#define UI_ELEM_DROP   4    // drops in from above with a bounce

// ── Button press animations ────────────────────────────────────────────────────
// Call ButtonPressStyle(style) before ButtonAdd() to set the press effect.
// Reset with ButtonPressStyle(UI_PRESS_NONE) when done.
#define UI_PRESS_NONE   0   // no press animation (default)
#define UI_PRESS_FLASH  1   // brief opacity dip on touch
#define UI_PRESS_PULSE  2   // scale down slightly then snap back
#define UI_PRESS_RIPPLE 3   // expanding circle from center

// ── Preset colors ─────────────────────────────────────────────────────────────
// Pass these to any color parameter, or use your own hex value: 0xRRGGBB
#define UI_GOLD       0xDB9E23
#define UI_DARK_GOLD  0xB8860B
#define UI_GREEN      0x00CC00
#define UI_RED        0xFF0000
#define UI_WHITE      0xFFFFFF
#define UI_BLACK      0x000000
#define UI_DARK_BG    0x333333
#define UI_GRAY       0x555555
#define UI_ORANGE     0xFF6600
#define UI_BLUE       0x0066FF
#define UI_PURPLE     0x8B00FF

// ── Declarative builder ────────────────────────────────────────────────────────
// Build custom screens by declaring pages, buttons, and labels by name.
// No callbacks, no LVGL — navigation between pages is automatic.
//
// Usage in build_screens():
//
//   BgColor(0x1A1A2E);                      // optional custom background color
//
//   PageAdd("home");                       // declare a page (do this first)
//   PageAdd("detail");
//
//   // ButtonAdd(page, x, y, w, h, color, text, goes_to, animated, auton_idx)
//   ButtonAdd("home", 20,  60, 200, 60, UI_GOLD, "Left Auto",   "detail", true,  0);
//   ButtonAdd("home", 260, 60, 200, 60, UI_GREEN,"Right Auto",  "detail", true,  1);
//   ButtonAdd("detail", 10, 10,  80, 35, UI_GOLD,"< Back",      "home",   false);
//
//   // LabelAdd(page, x, y, text, font_size, color)
//   LabelAdd("home", 10, 248, "Team 1234X", 14, UI_GRAY);
//
//   PageShow("home");                       // load the starting page

// Set the background color for all pages declared after this call.
// Use a UI_* preset or any hex value like 0x1A1A2E.  Default: UI_DARK_BG.
void BgColor(uint32_t color);

// Set the press animation for all ButtonAdd() calls that follow.
// Reset with ButtonPressStyle(UI_PRESS_NONE) when done.
void ButtonPressStyle(int press_anim);

// Set the animation style used when navigating between pages.
// Call this once in build_screens() before PageShow().
//   UI_ANIM_NONE  — instant (default)
//   UI_ANIM_FADE  — new page fades in over 200 ms
//   UI_ANIM_SLIDE — pages slide horizontally
void PageAnim(int anim_type);

// Declare a named page.  Must be called before adding buttons/labels to it.
void PageAdd(const char* name);

// Add a button to a named page.
//   goes_to  — name of the page to load when tapped (nullptr = no navigation)
//   animated — true to play a grow-in animation (buttons on the same page
//              are auto-staggered so they animate one after another)
//   auton_idx — if >= 0, tapping this button records it as the selected auton;
//               get_selected_auton() will return this value
// corner_radius — UI_SHAPE_SHARP, UI_SHAPE_PILL, or any pixel value (default 8).
// on_tap — optional plain function called when the button is tapped, before any navigation.
//          Use this to trigger robot actions (e.g. chassis.drive_imu_reset()) from a button.
void ButtonAdd(const char* page,
            int x, int y, int w, int h,
            uint32_t color        = UI_WHITE,
            const char* text      = "",
            const char* goes_to   = nullptr,
            int  animated         = UI_ELEM_NONE,
            int  auton_idx        = -1,
            int  corner_radius    = 8,
            void (*on_tap)()      = nullptr);

// Add a text label to a named page.
//   animated — UI_ELEM_* style to animate in when the page first appears (staggered)
//              UI_ELEM_GROW on a label behaves the same as UI_ELEM_FADE (text can't grow)
void LabelAdd(const char* page,
            int x, int y,
            const char* text,
            int font_size  = 18,
            uint32_t color = UI_WHITE,
            int animated   = UI_ELEM_NONE);

// Add a colored box to a named page.
// border_color — outline color; pass 0 (default) for no border.
// border_width — outline thickness in pixels (default 2).
void BoxAdd(const char* page,
            int x, int y, int w, int h,
            uint32_t color,
            int radius            = 8,
            uint32_t border_color = 0,
            int border_width      = 2);

// Add a filled circle to a named page.  x, y = top-left of the bounding square.
void CircleAdd(const char* page,
               int x, int y, int diameter,
               uint32_t color,
               uint32_t border_color = 0,
               int border_width      = 2);

// Add a filled square (sharp corners) to a named page.
void SquareAdd(const char* page,
               int x, int y, int size,
               uint32_t color,
               uint32_t border_color = 0,
               int border_width      = 2);

// Add a square with rounded corners.  radius controls how round (0 = sharp, size/2 = circle).
void RoundedBoxAdd(const char* page,
                       int x, int y, int size,
                       uint32_t color,
                       int radius            = 8,
                       uint32_t border_color = 0,
                       int border_width      = 2);

// Upward-pointing filled triangle inside a w × h bounding box.
void TriangleAdd(const char* page,
                 int x, int y, int w, int h,
                 uint32_t color);

// Load a page by name — call this last to show the starting screen.
void PageShow(const char* page);

// Remove all widgets from a page, reset its live-widget slots, and restore
// its original background color.  Use this to reuse one "detail" page.
void PageClear(const char* page);

// Change a page's background color at runtime (e.g. inside a show_* callback).
// PageClear restores the original color set at PageAdd time.
void PageBgColor(const char* page, uint32_t color);

// Declare a popup overlay.  Add content with ButtonAdd/LabelAdd using the same name.
// To open it from a button: goes_to = "popup:name"
// To close it from a button inside: goes_to = "close"
// title — text at the top of the popup (pass "" for none)
void PopupAdd(const char* name, int w, int h, const char* title,
              uint32_t bg_color     = UI_DARK_BG,
              uint32_t border_color = UI_GREEN);

// ── Quick start ────────────────────────────────────────────────────────────────
// Builds the full two-tab brain screen automatically.
// Call once from build_screens() in user_screen.cpp.
//
//   auton_names[]    — array of your auton route names
//   num_autons       — number of entries in that array
//   port_dots[20]    — dot color per port (UI_GOLD/UI_PURPLE/UI_BLUE/0)
//
// Optional extras — pass a function to add labels/buttons/boxes to an existing
// page after the standard content is placed.  Pass nullptr to skip.
//
//   on_auton_page(lv_obj_t* page) — called after auton buttons are added
//   on_port_page (lv_obj_t* page) — called after port grid is added
//
// Example (add team name to bottom of auton page):
//   static void my_extras(lv_obj_t* page) {
//     CreateLabel(page, 10, 248, "Team 1234X", 14, UI_GRAY);
//   }
//   void build_screens() {
//     ScreenStart(AUTON_NAMES, NUM_AUTONS, PORT_DOTS, my_extras);
//   }
void ScreenStart(const char* auton_names[], int num_autons, uint32_t port_dots[20],
              void (*on_auton_page)(lv_obj_t* page) = nullptr,
              void (*on_port_page) (lv_obj_t* page) = nullptr);

// Returns which auton the driver tapped (0-based index), or -1 if none.
int SelectedAuton();

// Programmatically set the selected auton (e.g. from the controller).
void ForceSelectAuton(int idx);

// ── Screen management ──────────────────────────────────────────────────────────
// Create a blank screen (call this before adding anything to it).
lv_obj_t* ScreenCreate();

// Make a screen visible on the brain display.
void ScreenLoad(lv_obj_t* screen);

// ── Basic elements ─────────────────────────────────────────────────────────────
//
//  parent    — the screen (or box) to place this inside
//  x, y      — position in pixels from the top-left corner of the parent
//  w, h      — width and height in pixels
//  color     — background or text color (use a UI_* constant or 0xRRGGBB)
//  text      — the text to display
//  font_size — one of: 14, 18, 20, 24, 48
//  radius    — corner rounding in pixels (0 = sharp corners)
//  on_click  — function to call when clicked (or nullptr for no click)
//  data      — optional value passed to your callback via lv_event_get_user_data()

// Clickable button with centered text.
// corner_radius — use UI_SHAPE_SHARP, UI_SHAPE_PILL, or any pixel value (default 8).
lv_obj_t* CreateButton(lv_obj_t* parent,
                         int x, int y, int w, int h,
                         uint32_t color, const char* text,
                         void (*on_click)(lv_event_t*) = nullptr,
                         void* data                    = nullptr,
                         int corner_radius             = 8);

// Button that animates in (grows from zero width).
// anim_delay_ms: how long to wait before the grow animation starts.
// Stagger multiple buttons by increasing delay each time (e.g. 0, 80, 160 ...).
lv_obj_t* CreateButtonAnimated(lv_obj_t* parent,
                                  int x, int y, int w, int h,
                                  uint32_t color, const char* text,
                                  int anim_delay_ms   = 0,
                                  void (*on_click)(lv_event_t*) = nullptr,
                                  void* data          = nullptr,
                                  int corner_radius   = 12);

// Text label.
lv_obj_t* CreateLabel(lv_obj_t* parent,
                        int x, int y,
                        const char* text,
                        int font_size = 18,
                        uint32_t color = UI_WHITE);

// Solid colored box / panel (useful as a background section or divider).
// border_color — outline color; pass 0 (default) for no border.
// border_width — thickness of the border in pixels (default 2).
lv_obj_t* CreateBox(lv_obj_t* parent,
                      int x, int y, int w, int h,
                      uint32_t color,
                      int radius            = 8,
                      uint32_t border_color = 0,
                      int border_width      = 2);

// Filled circle.  x, y = top-left corner of the bounding square.
// border_color — outline color; pass 0 for no border.
lv_obj_t* CreateCircle(lv_obj_t* parent,
                          int x, int y, int diameter,
                          uint32_t color,
                          uint32_t border_color = 0,
                          int border_width      = 2);

// Upward-pointing filled triangle inside a w × h bounding box.
// Apex at top-center, base along the bottom edge.
lv_obj_t* CreateTriangle(lv_obj_t* parent,
                            int x, int y, int w, int h,
                            uint32_t color);

// ── Tab bar ────────────────────────────────────────────────────────────────────
// Adds a two-tab navigation bar to the top of both screens.
// Call AFTER creating both screens, BEFORE loading either one.
// The tab bar occupies y = 0–40, so place your content starting at y = 42.
void TabBarAdd(lv_obj_t* screen1, lv_obj_t* screen2,
                    const char* tab1_name, const char* tab2_name,
                    uint32_t active_color   = UI_DARK_GOLD,
                    uint32_t inactive_color = UI_GOLD);

// ── Popup overlay ──────────────────────────────────────────────────────────────
// Creates a popup dialog centered on the currently visible screen.
// Add labels and buttons to the returned pointer just like any other parent.
// title      — text shown at the top of the popup (pass "" for no title)
// bg_color   — background fill color of the popup box
// border_color — color of the 3px border around the popup
lv_obj_t* PopupCreate(int w, int h, const char* title,
                           uint32_t bg_color     = UI_DARK_BG,
                           uint32_t border_color = UI_GREEN);

// Deletes the popup — safe to call from inside a button callback.
void PopupClose(lv_obj_t* popup);

// ── Port grid ──────────────────────────────────────────────────────────────────
// Builds a 4-row × 5-column grid of 20 port buttons (ports 1-20).
// start_x, start_y — top-left corner of the grid
// dot_colors[20]   — small indicator dot shown on each button;
//                    use UI_GOLD, UI_PURPLE, UI_BLUE, etc. — pass 0 for no dot
// on_click         — called when a port button is clicked;
//                    use (int)(intptr_t)lv_event_get_user_data(e) to get the port number
// out_buttons[20]  — filled with the 20 button pointers so you can update them live
void PortGridAdd(lv_obj_t* parent,
                      int start_x, int start_y,
                      uint32_t dot_colors[20],
                      void (*on_click)(lv_event_t*),
                      lv_obj_t* out_buttons[20]);

// ── Live updates ───────────────────────────────────────────────────────────────
// All of these are safe to call from background pros::Task functions.

// Change the text of a label.
void LabelSetText(lv_obj_t* label, const char* text);

// Change the text of a label using printf-style formatting.
// Example: LabelSetTextFmt(temp_label, "Temp: %d F", temp);
void LabelSetTextFmt(lv_obj_t* label, const char* fmt, ...);

// Change the background color of any object (buttons, boxes).
void ObjSetColor(lv_obj_t* obj, uint32_t color);

// Change the text color of a label.
void LabelSetColor(lv_obj_t* label, uint32_t color);

// Updates all 20 port button colors based on what is physically plugged in.
// Green = device connected, Red = nothing connected.
// Call this from a background task on a loop (e.g. every 100 ms).
void PortStatusUpdate(lv_obj_t* port_buttons[20]);

// ── Convenience helpers ────────────────────────────────────────────────────────
// Returns true if a device is plugged into the given port (1–20).
bool PortConnected(int port);

// Returns motor temperature in Fahrenheit for the given port (1–20).
int MotorTempF(int port);

// ── Blink labels ───────────────────────────────────────────────────────────────
// A live label that also blinks between two colors when should_blink() returns true.
// Useful for "Motors Connected: 7/8" that turns red and flashes when something is missing.
//
// getter       — returns the text to display (updated every interval_ms)
// should_blink — when true, label alternates between color_normal and color_blink
// color_normal — color when everything is fine (or blink_state is off)
// color_blink  — color when blinking is active and state is on
// interval_ms  — how often to refresh text AND toggle blink color (default 300)
//
// Example:
//   const char* motor_count() {
//     static char buf[32];
//     snprintf(buf, sizeof(buf), "Motors: %d/8", count_connected_motors());
//     return buf;
//   }
//   bool motors_missing() { return count_connected_motors() < 8; }
//   BlinkLabelAdd("ready", 20, 80, motor_count, motors_missing,
//                     UI_WHITE, UI_RED, 300, 18);
void BlinkLabelAdd(const char* page,
                       int x, int y,
                       const char* (*getter)(),
                       bool (*should_blink)(),
                       uint32_t color_normal = UI_WHITE,
                       uint32_t color_blink  = UI_RED,
                       int      interval_ms  = 300,
                       int      font_size    = 18);

// ── Port names ─────────────────────────────────────────────────────────────────
// Set custom names shown on port detail screens when using ScreenStart().
// Pass an array of 20 strings (nullptr for ports with no custom name).
// Call this in build_screens() before PageShow().
//
// Example:
//   static const char* MY_PORT_NAMES[20] = {
//     nullptr, nullptr, nullptr, nullptr, nullptr,   // ports 1-5
//     nullptr, nullptr, nullptr, "Intake Motor",     // ports 6-9
//     "Left Drive", nullptr, nullptr, "IMU",         // ports 10-13
//     // remaining ports default to "Port N"
//   };
//   PortNames(MY_PORT_NAMES);
void PortNames(const char* names[20]);

// ── Popup live labels ──────────────────────────────────────────────────────────
// A label that auto-refreshes while a specific popup is open.
// Register it in build_screens() alongside your PopupAdd() call.
// The engine creates the label when the popup opens and stops updating it when closed.
//
// This is the key ingredient for things like an IMU rotation display inside a popup.
//
// Example — IMU popup with live rotation:
//   const char* imu_text() {
//     static char buf[16];
//     snprintf(buf, sizeof(buf), "%.1f deg", chassis.drive_imu_get());
//     return buf;
//   }
//   static void do_imu_reset() { chassis.drive_imu_reset(); }
//
//   // In build_screens():
//   PopupAdd("imu", 300, 180, "IMU Check", UI_DARK_BG, UI_GREEN);
//   PopupLabelAdd("imu", 80, 65, imu_text, 50, 48, UI_WHITE);  // 50ms refresh
//   ButtonAdd("imu", 20,  135, 110, 35, UI_ORANGE, "Reset IMU", "close",
//          false, -1, 8, do_imu_reset);
//   ButtonAdd("imu", 170, 135, 110, 35, UI_GREEN,  "Close",     "close");
void PopupLabelAdd(const char* popup_name,
                       int x, int y,
                       const char* (*getter)(),
                       int      interval_ms = 100,
                       int      font_size   = 18,
                       uint32_t color       = UI_WHITE);

// ── Progress bar ───────────────────────────────────────────────────────────────
// A horizontal bar whose fill width reflects a 0–100 getter (battery, temp, etc.).
// The fill turns to warn_color when the value drops at or below warn_threshold.
//
// Example — battery bar that goes red below 20%:
//   int battery_pct() { return (int)pros::battery::get_capacity(); }
//   BarAdd("select", 10, 258, 200, 10, battery_pct, 1000,
//               UI_GREEN, UI_GRAY, 4, UI_RED, 20);
void BarAdd(const char* page,
                 int x, int y, int w, int h,
                 int (*getter)(),
                 int interval_ms       = 500,
                 uint32_t fill_color   = UI_GREEN,
                 uint32_t bg_color     = UI_GRAY,
                 int radius            = 4,
                 uint32_t warn_color   = UI_RED,
                 int warn_threshold    = 20);

// ── Toggle button ──────────────────────────────────────────────────────────────
// Stays highlighted in color_on and shows text_on while active.
// Tapping again reverts to color_off / text_off.
// on_toggle is called with the new state (true = on) after every tap.
//
// Example — slow-mode toggle:
//   static bool slow_mode = false;
//   static void on_slow(bool s) { slow_mode = s; }
//   ToggleAdd("select", 20, 200, 140, 45,
//                 UI_GRAY, UI_ORANGE, "Fast", "Slow",
//                 UI_ELEM_GROW, 8, on_slow);
void ToggleAdd(const char* page,
                   int x, int y, int w, int h,
                   uint32_t color_off, uint32_t color_on,
                   const char* text_off, const char* text_on,
                   int animated               = UI_ELEM_NONE,
                   int corner_radius          = 8,
                   void (*on_toggle)(bool)    = nullptr,
                   bool init_state            = false);

// ── Live color dot ─────────────────────────────────────────────────────────────
// A filled circle whose color is set by getter() every interval_ms.
// Return UI_GREEN when ready, UI_RED when faulted, UI_ORANGE for a warning, etc.
//
// Example — IMU status dot:
//   uint32_t imu_color() {
//     return chassis.drive_imu_calibrated() ? UI_GREEN : UI_RED;
//   }
//   DotAdd("select", 460, 255, 12, imu_color, 200);
void DotAdd(const char* page,
                 int x, int y, int diameter,
                 uint32_t (*getter)(),
                 int interval_ms = 250);

// ── Button grid ────────────────────────────────────────────────────────────────
// Places item_count buttons in a grid, auto-computing each button's size and
// position to fill the given total_w × total_h area evenly.
//
// ui_btn_item fields:
//   color      — button background color
//   text       — button label
//   auton_idx  — auton index recorded when tapped (-1 = not an auton button)
//   goes_to    — page name or "popup:name" to navigate on tap (nullptr = none)
//   on_tap     — plain function called on tap before navigation
//
// Example — 5 auton buttons in a 2-column grid:
//   static const ui_btn_item AUTONS[] = {
//     { UI_GOLD,   "Left Auto",   0, "ready" },
//     { UI_GREEN,  "Right Auto",  1, "ready" },
//     { UI_BLUE,   "Elim Left",   2, "ready" },
//     { UI_PURPLE, "Elim Right",  3, "ready" },
//     { UI_ORANGE, "Skills",      4, "ready" },
//   };
//   GridAdd("select", 20, 60, 440, 160, 2, AUTONS, 5);
struct ui_btn_item {
  uint32_t    color;
  const char* text;
  int         auton_idx  = -1;
  const char* goes_to    = nullptr;
  void (*on_tap)()       = nullptr;
};

void GridAdd(const char* page,
                 int x, int y, int total_w, int total_h,
                 int cols,
                 const ui_btn_item items[], int item_count,
                 int gap           = 10,
                 int animated      = UI_ELEM_GROW,
                 int corner_radius = 8);

// ── Slider ─────────────────────────────────────────────────────────────────────
// A horizontal drag slider for picking an integer value.
// The current value is displayed automatically to the right of the slider.
// on_change fires on every drag movement.
//
// Example — auton delay slider:
//   static int auton_delay_ms = 0;
//   static void on_delay(int v) { auton_delay_ms = v; }
//   SliderAdd("select", 20, 180, 300, 24, 0, 3000, 0, UI_GOLD, on_delay);
//   // Use auton_delay_ms in autonomous() e.g. pros::delay(auton_delay_ms);
void SliderAdd(const char* page,
               int x, int y, int w, int h,
               int min_val, int max_val, int default_val,
               uint32_t color            = UI_GOLD,
               void (*on_change)(int v)  = nullptr);

// ── Countdown timer ─────────────────────────────────────────────────────────────
// Displays a live MM:SS countdown. Starts paused — call CountdownStart() to begin
// (e.g. from a button on_tap or from the top of autonomous()).
// Turns warn_color when remaining time drops to or below warn_seconds.
// on_expire is called once when the timer reaches 0:00.
//
// Example — 60-second skills timer:
//   static void on_expire() { CtrlRumble("---"); }
//   CountdownAdd("skills", 185, 90, 60, 48, UI_WHITE, UI_RED, 10, on_expire);
//
//   static void start_timer() { CountdownStart(); }
//   ButtonAdd("skills", 165, 170, 150, 45, UI_GREEN, "Start", nullptr,
//                  UI_ELEM_NONE, -1, 8, start_timer);
void CountdownAdd(const char* page,
                  int x, int y,
                  int seconds,
                  int      font_size    = 48,
                  uint32_t color        = UI_WHITE,
                  uint32_t warn_color   = UI_RED,
                  int      warn_seconds = 10,
                  void (*on_expire)()   = nullptr);

// Start or restart all visible countdown timers from their full duration.
void CountdownStart();

// Stop all visible countdown timers (can be restarted with CountdownStart).
void CountdownStop();

// Returns whole seconds remaining on the first countdown (0 if expired or not started).
int CountdownRemaining();

// ── Field map selector ─────────────────────────────────────────────────────────
// Draws a square field diagram with circular position markers that act as
// auton selector buttons.  Each marker is placed at a normalized coordinate
// (0.0 = left/top edge, 1.0 = right/bottom edge).  Markers are clamped so they
// always stay fully inside the field boundary.
//
// Game-specific zones and lines should be drawn with BoxAdd() BEFORE this call
// so they render underneath the markers.  See the Push Back example in
// user_screen.cpp for a complete layout (zones + barrier + markers).
//
// Typical usage — replace the manual ButtonAdd calls in Step 2 with this:
//   static const ui_field_pos POSITIONS[] = {
//     { 0.15f, 0.75f, UI_GOLD,   "L", 0, "ready" },   // bottom-left
//     { 0.85f, 0.75f, UI_GREEN,  "R", 1, "ready" },   // bottom-right
//     { 0.50f, 0.85f, UI_BLUE,   "S", 2, "ready" },   // skills (center bottom)
//   };
//   FieldMapAdd("select", 10, 50, 220, POSITIONS, 3);
//
// field_color  — fill color of the field (default: dark VEX green)
// border_color — outline + robot-side indicator color
// marker_size  — diameter of each position button in pixels (default: 36)
//
// Leave room for other UI elements — a 220 px field leaves 250 px to the right
// for labels, a confirmation box, battery bar, etc.
struct ui_field_pos {
  float       field_x;           // 0.0 = left edge, 1.0 = right edge
  float       field_y;           // 0.0 = top edge,  1.0 = bottom edge
  uint32_t    color;             // marker color
  const char* label;             // short text shown inside the marker (1–3 chars)
  int         auton_idx  = -1;   // auton index recorded on tap
  const char* goes_to    = nullptr;  // page or "popup:name" (nullptr = none)
  void (*on_tap)()       = nullptr;  // optional callback before navigation
};

void FieldMapAdd(const char* page,
                  int x, int y, int size,
                  const ui_field_pos positions[], int count,
                  uint32_t field_color  = 0x1A6B1A,
                  uint32_t border_color = UI_WHITE,
                  int marker_size       = 36);

// ── Live labels ────────────────────────────────────────────────────────────────
// Add a label whose text refreshes automatically by calling getter() every interval_ms.
//
// getter must be a plain function (no captures) that returns const char*.
// Use a static char buf[] inside it to hold the formatted string.
//
// Example:
//   const char* battery_text() {
//     static char buf[32];
//     snprintf(buf, sizeof(buf), "Battery: %d%%", (int)pros::battery::get_capacity());
//     return buf;
//   }
//   // In build_screens():
//   LiveLabelAdd("select", 300, 250, battery_text, 500, 14, UI_GREEN);
//
// The engine starts a shared background task automatically when the first
// live label is registered — you don't need to manage any task yourself.
void LiveLabelAdd(const char* page,
                 int x, int y,
                 const char* (*getter)(),
                 int      interval_ms = 500,
                 int      font_size   = 18,
                 uint32_t color       = UI_WHITE);

// ── Controller screen ──────────────────────────────────────────────────────────
// The VEX V5 controller has a 3-line text display.  Rows are 0, 1, 2.
// Each row holds up to 19 characters.
//
// Example (static text):
//   CtrlLabel(0, "Team 1234X");
//   CtrlLabelFmt(1, "Bat: %d%%", (int)pros::battery::get_capacity());
//
// Example (auto-refreshing):
//   const char* ctrl_battery() {
//     static char buf[20];
//     snprintf(buf, sizeof(buf), "Bat: %d%%", (int)pros::battery::get_capacity());
//     return buf;
//   }
//   CtrlLive(1, ctrl_battery, 500);   // refresh every 500 ms

// Write static text to a controller row (0–2).
void CtrlLabel(int row, const char* text);

// Write formatted text to a controller row.  Same rules as printf.
void CtrlLabelFmt(int row, const char* fmt, ...);

// Register a getter that auto-refreshes a controller row every interval_ms.
// The engine manages the background task automatically.
void CtrlLive(int row,
                  const char* (*getter)(),
                  int interval_ms = 200);

// Clear one row (0–2), or all rows if row == -1.
void CtrlClear(int row = -1);

// Immediately write all queued controller rows with proper rate-limit delays.
// Call once at the end of initialize() so the display is populated before
// opcontrol starts, without waiting for the background task.
void CtrlFlush();

// Rumble the controller.  '.' = short buzz, '-' = long buzz, ' ' = pause.
void CtrlRumble(const char* pattern);

// ── Spinner ────────────────────────────────────────────────────────────────────
// An animated spinning arc — useful as a loading indicator or status widget.
//
// x, y     — top-left corner of the spinner's bounding box
// size     — width and height of the spinner in pixels (e.g. 80)
// color    — arc indicator color (default UI_GOLD)
// speed_ms — time in ms for one full revolution (default 1200)
// arc_deg  — length of the arc in degrees, 1–360 (default 75)
//
// Example — gold spinner while calibrating:
//   SpinnerAdd("loading", 190, 86, 100, UI_GOLD, 1200, 75);
void SpinnerAdd(const char* page,
                int x, int y, int size,
                uint32_t color    = UI_GOLD,
                int speed_ms      = 1200,
                int arc_deg       = 75);

// ── Driver mode ────────────────────────────────────────────────────────────────
// Locks the brain screen and disables touch during a match.
// Call EngineDriverMode(true)  when the driver combo is detected (UP + X).
// Call EngineDriverMode(false) to return to the normal UI.
//
// While active:
//   - Brain screen shows "MATCH IN PROGRESS" with live battery — untouchable
//   - Touch input is fully disabled so nothing can be tapped accidentally
//   - Controller UI actions (page switching, counter) should be gated in opcontrol
//
// Example in opcontrol():
//   bool up = master.get_digital(DIGITAL_UP);
//   bool x  = master.get_digital_new_press(DIGITAL_X);
//   if (up && x) { driver_mode = !driver_mode; EngineDriverMode(driver_mode); }
void EngineDriverMode(bool active);

// ── Misc ───────────────────────────────────────────────────────────────────────
// Called once in initialize() in main.cpp. Do not call yourself.
void EngineInit();
