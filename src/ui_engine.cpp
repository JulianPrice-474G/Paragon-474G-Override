#include "main.h"
#include "ui_engine.hpp"
#include <cstdarg>

// ── Internal state shared across all subsystems ───────────────────────────────
static pros::Mutex* _mutex          = nullptr;
static int          _selected_auton = -1;   // set by ButtonAdd(auton_idx) or _on_auton_btn
static bool         _building       = true; // true during build_screens() — background tasks skip LVGL calls
static uint32_t     _build_last_yield = 0;

// Drive LVGL directly every 15ms during build — lcd::shutdown() deleted the
// display task, so we call lv_task_handler() ourselves to keep the spinner
// animating. EngineInit() clears the stuck flush state first.
// No-ops after PageShow() clears _building.
static void _build_breathe() {
  if (!_building) return;
  uint32_t now = pros::millis();
  if (now - _build_last_yield >= 15) {
    lv_task_handler();
    _build_last_yield = pros::millis();
  }
}

void EngineInit() {
  _mutex            = new pros::Mutex();
  _building         = true;
  _build_last_yield = pros::millis();

  // pros::lcd::shutdown() deletes the display task without calling
  // lv_disp_flush_ready(), leaving the driver stuck in "flushing" state.
  // Clear it now so lv_task_handler() can run without blocking.
  lv_disp_t* disp = lv_disp_get_default();
  if (disp && disp->driver) lv_disp_flush_ready(disp->driver);
}

// ── Helper: integer font size → LVGL font pointer ────────────────────────────
static const lv_font_t* get_font(int size) {
  switch (size) {
    case 14: return &lv_font_montserrat_14;
    case 20: return &lv_font_montserrat_20;
    case 24: return &lv_font_montserrat_24;
    case 48: return &lv_font_montserrat_48;
    default: return &lv_font_montserrat_18;
  }
}

// ── Screen management ─────────────────────────────────────────────────────────
lv_obj_t* ScreenCreate() {
  lv_obj_t* screen = lv_obj_create(NULL);
  lv_obj_remove_style_all(screen);
  lv_obj_set_style_bg_color(screen, lv_color_hex(UI_DARK_BG), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  return screen;
}

void ScreenLoad(lv_obj_t* screen) {
  lv_scr_load(screen);
}

// ── Button ────────────────────────────────────────────────────────────────────
lv_obj_t* CreateButton(lv_obj_t* parent,
                         int x, int y, int w, int h,
                         uint32_t color, const char* text,
                         void (*on_click)(lv_event_t*),
                         void* data,
                         int corner_radius) {
  int r = (corner_radius == UI_SHAPE_PILL) ? h / 2 : corner_radius;
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_size(btn, w, h);
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_style_bg_color(btn, lv_color_hex(color), 0);
  lv_obj_set_style_radius(btn, r, 0);
  lv_obj_set_style_border_width(btn, 0, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(UI_WHITE), 0);
  lv_obj_center(lbl);

  if (on_click != nullptr)
    lv_obj_add_event_cb(btn, on_click, LV_EVENT_CLICKED, data);

  return btn;
}

// ── Animation callbacks ───────────────────────────────────────────────────────
static void _anim_width_cb(void* var, int32_t v) {
  lv_obj_set_width((lv_obj_t*)var, v);
}

// Fades an object's opacity from 0 (transparent) to 255 (fully visible).
static void _anim_opa_cb(void* var, int32_t v) {
  lv_obj_set_style_opa((lv_obj_t*)var, (lv_opa_t)v, 0);
}

// Slides an object horizontally to an x position.
static void _anim_x_cb(void* var, int32_t v) {
  lv_obj_set_x((lv_obj_t*)var, v);
}

// Drops an object vertically to a y position.
static void _anim_y_cb(void* var, int32_t v) {
  lv_obj_set_y((lv_obj_t*)var, v);
}

static void _anim_zoom_cb(void* var, int32_t v) {
  lv_obj_set_style_transform_zoom((lv_obj_t*)var, (lv_coord_t)v, 0);
}

static void _anim_ripple_size_cb(void* var, int32_t v) {
  lv_obj_t* circle = (lv_obj_t*)var;
  if (!circle) return;
  lv_obj_t* btn = lv_obj_get_parent(circle);
  if (!btn) return;
  lv_obj_set_size(circle, v, v);
  lv_obj_set_pos(circle, (lv_obj_get_width(btn) - v) / 2,
                          (lv_obj_get_height(btn) - v) / 2);
}

static void _ripple_ready_cb(lv_anim_t* a) {
  lv_obj_t* circle = (lv_obj_t*)a->var;
  if (circle) lv_obj_del(circle);
}

static void _press_anim_cb(lv_event_t* e) {
  lv_obj_t* btn  = lv_event_get_target(e);
  int style      = (int)(intptr_t)lv_event_get_user_data(e);
  lv_anim_t a;

  if (style == UI_PRESS_FLASH) {
    lv_anim_init(&a);
    lv_anim_set_var(&a, btn);
    lv_anim_set_values(&a, LV_OPA_COVER, LV_OPA_40);
    lv_anim_set_exec_cb(&a, _anim_opa_cb);
    lv_anim_set_time(&a, 100);
    lv_anim_set_playback_time(&a, 100);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);

  } else if (style == UI_PRESS_PULSE) {
    lv_obj_set_style_transform_pivot_x(btn, lv_obj_get_width(btn)  / 2, 0);
    lv_obj_set_style_transform_pivot_y(btn, lv_obj_get_height(btn) / 2, 0);
    lv_anim_init(&a);
    lv_anim_set_var(&a, btn);
    lv_anim_set_values(&a, 256, 215);
    lv_anim_set_exec_cb(&a, _anim_zoom_cb);
    lv_anim_set_time(&a, 100);
    lv_anim_set_playback_time(&a, 130);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in);
    lv_anim_start(&a);

  } else if (style == UI_PRESS_RIPPLE) {
    int bw = lv_obj_get_width(btn);
    int bh = lv_obj_get_height(btn);
    int max_dim = (bw > bh ? bw : bh) + 20;
    lv_obj_t* circle = lv_obj_create(btn);
    if (!circle) return;
    lv_obj_remove_style_all(circle);
    lv_obj_set_style_bg_color(circle, lv_color_hex(UI_WHITE), 0);
    lv_obj_set_style_bg_opa(circle, LV_OPA_30, 0);
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(circle, 0, 0);
    lv_obj_clear_flag(circle, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(circle, 1, 1);
    lv_obj_align(circle, LV_ALIGN_CENTER, 0, 0);
    lv_anim_init(&a);
    lv_anim_set_var(&a, circle);
    lv_anim_set_values(&a, 1, max_dim);
    lv_anim_set_exec_cb(&a, _anim_ripple_size_cb);
    lv_anim_set_time(&a, 350);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, _ripple_ready_cb);
    lv_anim_start(&a);
  }
}

// Apply one of the UI_ELEM_* animations to any LVGL object.
// target_x / target_y are the final resting position of the object.
static void _apply_anim(lv_obj_t* obj, int anim_type, int target_x, int target_y, int delay_ms) {
  lv_anim_t anim;
  lv_anim_init(&anim);
  lv_anim_set_var(&anim, obj);
  lv_anim_set_time(&anim, 350);
  lv_anim_set_delay(&anim, delay_ms);

  switch (anim_type) {
    case UI_ELEM_FADE:
      lv_obj_set_style_opa(obj, LV_OPA_TRANSP, 0);
      lv_anim_set_values(&anim, LV_OPA_TRANSP, LV_OPA_COVER);
      lv_anim_set_exec_cb(&anim, _anim_opa_cb);
      lv_anim_set_path_cb(&anim, lv_anim_path_ease_in_out);
      break;
    case UI_ELEM_SLIDE:
      // Start off the left edge, ease into final x position.
      lv_obj_set_x(obj, -250);
      lv_anim_set_values(&anim, -250, target_x);
      lv_anim_set_exec_cb(&anim, _anim_x_cb);
      lv_anim_set_path_cb(&anim, lv_anim_path_ease_out);
      break;
    case UI_ELEM_DROP:
      // Start above the screen, overshoot (bounce) into final y position.
      lv_obj_set_y(obj, -70);
      lv_anim_set_values(&anim, -70, target_y);
      lv_anim_set_exec_cb(&anim, _anim_y_cb);
      lv_anim_set_path_cb(&anim, lv_anim_path_overshoot);
      break;
    default:
      return;   // UI_ELEM_NONE or unrecognized — do nothing
  }
  lv_anim_start(&anim);
}

lv_obj_t* CreateButtonAnimated(lv_obj_t* parent,
                                  int x, int y, int w, int h,
                                  uint32_t color, const char* text,
                                  int anim_delay_ms,
                                  void (*on_click)(lv_event_t*),
                                  void* data,
                                  int corner_radius) {
  int r = (corner_radius == UI_SHAPE_PILL) ? h / 2 : corner_radius;
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_height(btn, h);     // height is set immediately so row layout is stable
  lv_obj_set_width(btn, 0);      // start at 0 — animation grows it to w
  lv_obj_set_pos(btn, x, y);
  lv_obj_set_style_bg_color(btn, lv_color_hex(color), 0);
  lv_obj_set_style_radius(btn, r, 0);
  lv_obj_set_style_border_width(btn, 0, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(UI_WHITE), 0);
  lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_center(lbl);

  if (on_click != nullptr)
    lv_obj_add_event_cb(btn, on_click, LV_EVENT_CLICKED, data);

  lv_anim_t anim;
  lv_anim_init(&anim);
  lv_anim_set_var(&anim, btn);
  lv_anim_set_values(&anim, 0, w);
  lv_anim_set_exec_cb(&anim, _anim_width_cb);
  lv_anim_set_path_cb(&anim, lv_anim_path_overshoot);
  lv_anim_set_time(&anim, 300);
  lv_anim_set_delay(&anim, anim_delay_ms);
  lv_anim_start(&anim);

  return btn;
}

// ── Label ─────────────────────────────────────────────────────────────────────
lv_obj_t* CreateLabel(lv_obj_t* parent,
                        int x, int y,
                        const char* text,
                        int font_size,
                        uint32_t color) {
  lv_obj_t* lbl = lv_label_create(parent);
  lv_label_set_text(lbl, text);
  lv_obj_set_style_text_font(lbl, get_font(font_size), 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(color), 0);
  lv_obj_set_pos(lbl, x, y);
  return lbl;
}

// ── Box ───────────────────────────────────────────────────────────────────────
lv_obj_t* CreateBox(lv_obj_t* parent,
                      int x, int y, int w, int h,
                      uint32_t color,
                      int radius,
                      uint32_t border_color,
                      int border_width) {
  lv_obj_t* box = lv_obj_create(parent);
  lv_obj_set_size(box, w, h);
  lv_obj_set_pos(box, x, y);
  lv_obj_set_style_bg_color(box, lv_color_hex(color), 0);
  lv_obj_set_style_radius(box, radius, 0);
  lv_obj_set_style_pad_all(box, 0, 0);
  lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
  if (border_color != 0) {
    lv_obj_set_style_border_width(box, border_width, 0);
    lv_obj_set_style_border_color(box, lv_color_hex(border_color), 0);
  } else {
    lv_obj_set_style_border_width(box, 0, 0);
  }
  return box;
}

// ── Triangle ──────────────────────────────────────────────────────────────────
// Renders via lv_canvas so triangle points are canvas-relative (0-based),
// avoiding any coordinate-space mismatch between lv_obj_get_coords and the
// draw context used by lv_draw_polygon in LV_EVENT_DRAW_MAIN callbacks.
static void _triangle_buf_free_cb(lv_event_t* e) {
  free(lv_event_get_user_data(e));
}

lv_obj_t* CreateTriangle(lv_obj_t* parent,
                           int x, int y, int w, int h,
                           uint32_t color) {
  void* buf = malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(w, h));
  if (!buf) return nullptr;

  lv_obj_t* canvas = lv_canvas_create(parent);
  lv_canvas_set_buffer(canvas, buf, w, h, LV_IMG_CF_TRUE_COLOR_ALPHA);
  lv_obj_set_pos(canvas, x, y);
  lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
  lv_canvas_fill_bg(canvas, lv_color_hex(0), LV_OPA_TRANSP);

  lv_draw_rect_dsc_t dsc;
  lv_draw_rect_dsc_init(&dsc);
  dsc.bg_color     = lv_color_hex(color);
  dsc.bg_opa       = LV_OPA_COVER;
  dsc.border_width = 0;
  dsc.radius       = 0;

  lv_point_t pts[3] = {
    { (lv_coord_t)(w / 2), 0               },  // apex — top center
    { 0,                   (lv_coord_t)(h - 1) },  // bottom left
    { (lv_coord_t)(w - 1), (lv_coord_t)(h - 1) },  // bottom right
  };
  lv_canvas_draw_polygon(canvas, pts, 3, &dsc);

  lv_obj_add_event_cb(canvas, _triangle_buf_free_cb, LV_EVENT_DELETE, buf);
  return canvas;
}

// ── Circle ────────────────────────────────────────────────────────────────────
lv_obj_t* CreateCircle(lv_obj_t* parent,
                         int x, int y, int diameter,
                         uint32_t color,
                         uint32_t border_color,
                         int border_width) {
  lv_obj_t* c = lv_obj_create(parent);
  lv_obj_set_size(c, diameter, diameter);
  lv_obj_set_pos(c, x, y);
  lv_obj_set_style_bg_color(c, lv_color_hex(color), 0);
  lv_obj_set_style_radius(c, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_pad_all(c, 0, 0);
  lv_obj_clear_flag(c, LV_OBJ_FLAG_SCROLLABLE);
  if (border_color != 0) {
    lv_obj_set_style_border_width(c, border_width, 0);
    lv_obj_set_style_border_color(c, lv_color_hex(border_color), 0);
  } else {
    lv_obj_set_style_border_width(c, 0, 0);
  }
  return c;
}

// ── Forward declarations ──────────────────────────────────────────────────────
// These static helpers are defined later in the declarative-builder section.
// Forward declarations are required in C++ for static functions called before
// their definition point in the same translation unit.
static void      _scr_load(lv_obj_t* s);
static lv_obj_t* _find_screen(const char* name);

// ── Tab bar ───────────────────────────────────────────────────────────────────
// Each tab button stores its target screen as user_data and calls _scr_load.
static void _tab_click_cb(lv_event_t* e) {
  lv_obj_t* target = (lv_obj_t*)lv_event_get_user_data(e);
  _scr_load(target);
}

static void _make_tab_button(lv_obj_t* parent, int slot,
                              const char* name, bool active,
                              uint32_t active_color, uint32_t inactive_color,
                              lv_obj_t* target_screen) {
  lv_obj_t* btn = lv_btn_create(parent);
  lv_obj_set_size(btn, 240, 40);
  lv_obj_set_pos(btn, slot == 0 ? 0 : 240, 0);
  lv_obj_set_style_bg_color(btn, lv_color_hex(active ? active_color : inactive_color), 0);
  lv_obj_set_style_radius(btn, 0, 0);
  lv_obj_set_style_border_width(btn, 1, 0);
  lv_obj_set_style_border_color(btn, lv_color_hex(0x8B6508), 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(btn, _tab_click_cb, LV_EVENT_CLICKED, target_screen);

  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, name);
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(UI_WHITE), 0);
  lv_obj_center(lbl);
}

void TabBarAdd(lv_obj_t* screen1, lv_obj_t* screen2,
                    const char* tab1_name, const char* tab2_name,
                    uint32_t active_color, uint32_t inactive_color) {
  // On screen1: tab1 is active, tab2 goes to screen2
  _make_tab_button(screen1, 0, tab1_name, true,  active_color, inactive_color, screen1);
  _make_tab_button(screen1, 1, tab2_name, false, active_color, inactive_color, screen2);

  // On screen2: tab1 goes to screen1, tab2 is active
  _make_tab_button(screen2, 0, tab1_name, false, active_color, inactive_color, screen1);
  _make_tab_button(screen2, 1, tab2_name, true,  active_color, inactive_color, screen2);
}

// ── Popup ─────────────────────────────────────────────────────────────────────
lv_obj_t* PopupCreate(int w, int h, const char* title,
                           uint32_t bg_color, uint32_t border_color) {
  lv_obj_t* popup = lv_obj_create(lv_scr_act());
  lv_obj_set_size(popup, w, h);
  lv_obj_align(popup, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(popup, lv_color_hex(bg_color), 0);
  lv_obj_set_style_border_color(popup, lv_color_hex(border_color), 0);
  lv_obj_set_style_border_width(popup, 3, 0);
  lv_obj_set_style_radius(popup, 15, 0);
  lv_obj_clear_flag(popup, LV_OBJ_FLAG_SCROLLABLE);

  if (title != nullptr && title[0] != '\0') {
    lv_obj_t* lbl = lv_label_create(popup);
    lv_label_set_text(lbl, title);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lbl, lv_color_hex(border_color), 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 10);
  }

  return popup;
}

void PopupClose(lv_obj_t* popup) {
  if (popup != nullptr)
    lv_obj_del(popup);
}

// ── Port grid ─────────────────────────────────────────────────────────────────
void PortGridAdd(lv_obj_t* parent,
                      int start_x, int start_y,
                      uint32_t dot_colors[20],
                      void (*on_click)(lv_event_t*),
                      lv_obj_t* out_buttons[20]) {
  const int btn_w   = 88, btn_h   = 42;
  const int margin_x = 8, margin_y = 6;

  for (int port = 1; port <= 20; port++) {
    int idx = port - 1;
    int row = idx / 5;
    int col = idx % 5;
    int x   = start_x + col * (btn_w + margin_x);
    int y   = start_y + row * (btn_h + margin_y);

    lv_obj_t* btn = lv_btn_create(parent);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_size(btn, 0, 0);   // start at 0 for grow animation
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_style_radius(btn, 10, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(UI_RED), 0);
    lv_obj_set_style_border_width(btn, 2, 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(0xCC0000), 0);
    lv_obj_set_style_shadow_width(btn, 0, 0);
    lv_obj_set_style_pad_all(btn, 0, 0);

    if (on_click != nullptr)
      lv_obj_add_event_cb(btn, on_click, LV_EVENT_CLICKED, (void*)(intptr_t)port);

    lv_obj_t* lbl = lv_label_create(btn);
    lv_label_set_text_fmt(lbl, "P: %d", port);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    lv_obj_center(lbl);

    // Indicator dot
    if (dot_colors != nullptr && dot_colors[idx] != 0) {
      lv_obj_t* dot = lv_obj_create(btn);
      lv_obj_set_size(dot, 8, 8);
      lv_obj_align(dot, LV_ALIGN_BOTTOM_RIGHT, -4, -4);
      lv_obj_set_style_radius(dot, 4, 0);
      lv_obj_set_style_bg_color(dot, lv_color_hex(dot_colors[idx]), 0);
      lv_obj_set_style_border_width(dot, 0, 0);
      lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
    }

    // Grow-in animation with staggered delay
    lv_obj_set_height(btn, btn_h);
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, btn);
    lv_anim_set_values(&anim, 0, btn_w);
    lv_anim_set_exec_cb(&anim, _anim_width_cb);
    lv_anim_set_path_cb(&anim, lv_anim_path_overshoot);
    lv_anim_set_time(&anim, 300);
    lv_anim_set_delay(&anim, idx * 30);
    lv_anim_start(&anim);

    if (out_buttons != nullptr)
      out_buttons[idx] = btn;
  }
}

// ── Live updates (mutex-safe) ─────────────────────────────────────────────────
void LabelSetText(lv_obj_t* label, const char* text) {
  if (label == nullptr || _mutex == nullptr || !_mutex->take(5)) return;
  lv_label_set_text(label, text);
  _mutex->give();
}

void LabelSetTextFmt(lv_obj_t* label, const char* fmt, ...) {
  if (label == nullptr || _mutex == nullptr || !_mutex->take(5)) return;
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  lv_label_set_text(label, buf);
  _mutex->give();
}

void ObjSetColor(lv_obj_t* obj, uint32_t color) {
  if (obj == nullptr || _mutex == nullptr || !_mutex->take(5)) return;
  lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
  _mutex->give();
}

void LabelSetColor(lv_obj_t* label, uint32_t color) {
  if (label == nullptr || _mutex == nullptr || !_mutex->take(5)) return;
  lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
  _mutex->give();
}

void PortStatusUpdate(lv_obj_t* port_buttons[20]) {
  if (_mutex == nullptr || !_mutex->take(5)) return;
  for (int port = 1; port <= 20; port++) {
    lv_obj_t* btn = port_buttons[port - 1];
    if (btn == nullptr) continue;
    bool connected = pros::Device(port).is_installed();
    lv_obj_set_style_bg_color(btn,
      lv_color_hex(connected ? UI_GREEN : UI_RED), 0);
    lv_obj_set_style_border_color(btn,
      lv_color_hex(connected ? 0x009900 : 0xCC0000), 0);
  }
  _mutex->give();
}

// ── Convenience helpers ───────────────────────────────────────────────────────
bool PortConnected(int port) {
  // pros::Device::is_installed() returns TRUE when a device IS plugged in,
  // so this must not be negated.
  return pros::Device(port).is_installed();
}

int MotorTempF(int port) {
  pros::Motor m(port);
  double c = m.get_temperature();
  return (int)((c * 9.0 / 5.0) + 32.0);
}

// ── Live labels ───────────────────────────────────────────────────────────────
// Each live label stores a pointer to its LVGL object, the getter function that
// produces the text, and how often (in ms) to call that getter.

#define _MAX_LIVE_LBLS 16

typedef const char* (*_LiveGetter)();

struct _LiveLbl {
  lv_obj_t*   obj;
  _LiveGetter getter;
  int         interval_ms;
  uint32_t    last_ms;
};

static _LiveLbl _live_lbls[_MAX_LIVE_LBLS] = {};
static int      _live_lbl_count            = 0;
static bool     _live_task_started         = false;

// Background task — checks each live label and calls its getter when due.
static void _live_label_task(void*) {
  while (true) {
    if (_building) { pros::delay(20); continue; }
    uint32_t now = pros::millis();
    for (int i = 0; i < _live_lbl_count; i++) {
      _LiveLbl& ll = _live_lbls[i];
      if (now - ll.last_ms >= (uint32_t)ll.interval_ms) {
        if (ll.getter) {
          const char* text = ll.getter();
          if (text) LabelSetText(ll.obj, text);
        }
        ll.last_ms = now;
      }
    }
    pros::delay(50);   // poll at 50ms; updates only fire at each label's own interval
  }
}

void LiveLabelAdd(const char* page,
                 int x, int y,
                 const char* (*getter)(),
                 int interval_ms,
                 int font_size,
                 uint32_t color) {
  if (_live_lbl_count >= _MAX_LIVE_LBLS || getter == nullptr) return;
  lv_obj_t* s = _find_screen(page);
  if (!s) return;

  lv_obj_t* lbl = CreateLabel(s, x, y, getter(), font_size, color);

  _LiveLbl& ll   = _live_lbls[_live_lbl_count++];
  ll.obj         = lbl;
  ll.getter      = getter;
  ll.interval_ms = interval_ms;
  ll.last_ms     = pros::millis();

  // Start the shared background task on the first live label registered.
  if (!_live_task_started) {
    static pros::Task task(_live_label_task, nullptr, "Live Labels");
    _live_task_started = true;
  }
  _build_breathe();
}

// ── Controller screen ─────────────────────────────────────────────────────────
// VEX V5 controller: 3-line display, 19 chars per line.
// The controller has a GLOBAL 50 ms rate limit — only one write (to any row)
// goes through per window.  Rapid back-to-back set_text calls are silently
// dropped.  The correct approach (same as the robot's Custom-Brain-UI branch):
//   master.clear() → delay(55) → master.print(row0) → delay(55) → ...
// Each write gets its own 50 ms window, so all rows arrive reliably.

static pros::Controller _ctrl(pros::E_CONTROLLER_MASTER);

static char _ctrl_text[3][20]  = {};   // text for each row (%-19s padded)
static bool _ctrl_active[3]    = {};   // whether row has content to display
static bool _ctrl_dirty        = false;
static bool _ctrl_task_started = false;
static uint32_t _ctrl_last_send = 0;
static const uint32_t _CTRL_REFRESH_MS = 3000; // periodic refresh in case of drift

// Send all active rows: clear → delay → print each row → delay.
// Called from the background task, never directly.
static void _ctrl_send_all() {
  _ctrl.clear();
  pros::delay(55);
  for (int row = 0; row < 3; row++) {
    if (_ctrl_active[row]) {
      _ctrl.print(row, 0, "%s", _ctrl_text[row]);  // "%s" so stored text is never a format string
      pros::delay(55);
    }
  }
  _ctrl_last_send = pros::millis();
}

static lv_obj_t* _easter_scr       = nullptr;
static lv_obj_t* _pre_easter_scr   = nullptr;
static bool      _easter_triggered = false;

// Set by EngineDriverMode().  Read by the easter egg check below and exposed
// to user code through DriverModeActive().
static bool      _driver_mode_active = false;

static void _ctrl_task(void*) {
  while (true) {
    if (_building) { pros::delay(50); continue; }
    uint32_t now = pros::millis();
    if (_ctrl_dirty || now - _ctrl_last_send >= _CTRL_REFRESH_MS) {
      _ctrl_dirty = false;
      _ctrl_send_all();
    }

    // Easter egg: hold all 4 top triggers for ~600ms.
    // Skipped entirely in driver mode.  The easter screen is dismissed by a
    // TAP, and driver mode disables touch - so firing it during a match would
    // leave the brain stuck on it with no way out.
    {
      static int hold = 0;
      bool combo = !_driver_mode_active &&
                   _ctrl.get_digital(DIGITAL_L1) && _ctrl.get_digital(DIGITAL_L2) &&
                   _ctrl.get_digital(DIGITAL_R1) && _ctrl.get_digital(DIGITAL_R2);
      if (combo) {
        if (++hold == 30 && !_easter_triggered) {
          _pre_easter_scr   = lv_scr_act();
          _easter_triggered = true;
          lv_scr_load_anim(_easter_scr, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, false);
        }
      } else {
        hold = 0;
      }
    }

    pros::delay(20);
  }
}

// ── Easter egg ────────────────────────────────────────────────────────────────
static void _easter_dismiss_cb(lv_event_t*) {
  if (_pre_easter_scr) {
    lv_scr_load_anim(_pre_easter_scr, LV_SCR_LOAD_ANIM_FADE_ON, 400, 0, false);
    _pre_easter_scr   = nullptr;
    _easter_triggered = false;
  }
}

static void _setup_easter_egg() {
  _easter_scr = lv_obj_create(nullptr);
  lv_obj_remove_style_all(_easter_scr);
  lv_obj_set_style_bg_color(_easter_scr, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(_easter_scr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(_easter_scr, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(_easter_scr, _easter_dismiss_cb, LV_EVENT_CLICKED, nullptr);

  lv_obj_t* t1 = CreateLabel(_easter_scr, 0, 55, "PARAGON", 48, UI_GOLD);
  lv_obj_set_width(t1, 480);
  lv_obj_set_style_text_align(t1, LV_TEXT_ALIGN_CENTER, 0);

  lv_obj_t* t2 = CreateLabel(_easter_scr, 0, 135, "was here........", 24, UI_WHITE);
  lv_obj_set_width(t2, 480);
  lv_obj_set_style_text_align(t2, LV_TEXT_ALIGN_CENTER, 0);

  lv_obj_t* t3 = CreateLabel(_easter_scr, 0, 215, "tap to close", 14, 0x333333);
  lv_obj_set_width(t3, 480);
  lv_obj_set_style_text_align(t3, LV_TEXT_ALIGN_CENTER, 0);
}

static void _start_ctrl_task() {
  if (!_ctrl_task_started) {
    _setup_easter_egg();
    static pros::Task task(_ctrl_task, nullptr, TASK_PRIORITY_DEFAULT,
                           TASK_STACK_DEPTH_DEFAULT, "Ctrl Screen");
    _ctrl_task_started = true;
  }
}

// Set one row of text.  Stores the text and triggers a full display send
// (clear + all rows) from the background task so every row arrives reliably.
void CtrlLabel(int row, const char* text) {
  if (row < 0 || row > 2) return;
  snprintf(_ctrl_text[row], sizeof(_ctrl_text[row]), "%-19s", text ? text : "");
  _ctrl_active[row] = true;
  _ctrl_dirty = true;
  _start_ctrl_task();
}

// Set a line of text using printf-style formatting.
void CtrlLabelFmt(int row, const char* fmt, ...) {
  if (row < 0 || row > 2) return;
  char buf[20];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  snprintf(_ctrl_text[row], sizeof(_ctrl_text[row]), "%-19s", buf);
  _ctrl_active[row] = true;
  _ctrl_dirty = true;
  _start_ctrl_task();
}

// CtrlLive: kept for API compatibility — schedules a one-time write.
void CtrlLive(int row, const char* (*getter)(), int interval_ms) {
  if (row < 0 || row > 2 || !getter) return;
  const char* text = getter();
  snprintf(_ctrl_text[row], sizeof(_ctrl_text[row]), "%-19s", text ? text : "");
  _ctrl_active[row] = true;
  _ctrl_dirty = true;
  _start_ctrl_task();
}

// Clear one row (or all rows if row == -1).
void CtrlClear(int row) {
  if (row == -1) {
    for (int i = 0; i < 3; i++) { _ctrl_active[i] = false; }
  } else if (row >= 0 && row <= 2) {
    _ctrl_active[row] = false;
  }
  _ctrl_dirty = true;
}

// Rumble the controller.  Pattern: '.' = short, '-' = long, ' ' = pause.
// Example: "..--" = two short, two long.
void CtrlRumble(const char* pattern) {
  _ctrl.rumble(pattern);
}

// ── Blink labels ──────────────────────────────────────────────────────────────
// Like a live label, but also blinks between two colors when should_blink() is true.

#define _MAX_BLINK_LBLS 8

struct _BlinkLbl {
  lv_obj_t*   obj;
  _LiveGetter getter;          // text to display (called every interval)
  bool (*should_blink)();      // when true, toggle between color_normal and color_blink
  uint32_t    color_normal;
  uint32_t    color_blink;
  int         interval_ms;
  uint32_t    last_ms;
  bool        blink_state;
};

static _BlinkLbl _blink_lbls[_MAX_BLINK_LBLS] = {};
static int       _blink_lbl_count             = 0;
static bool      _blink_task_started          = false;

static void _blink_task(void*) {
  while (true) {
    if (_building) { pros::delay(20); continue; }
    uint32_t now = pros::millis();
    for (int i = 0; i < _blink_lbl_count; i++) {
      _BlinkLbl& bl = _blink_lbls[i];
      if (now - bl.last_ms < (uint32_t)bl.interval_ms) continue;
      bl.last_ms    = now;
      bl.blink_state = !bl.blink_state;

      if (bl.getter) {
        const char* text = bl.getter();
        if (text) LabelSetText(bl.obj, text);
      }

      bool blinking = bl.should_blink ? bl.should_blink() : false;
      uint32_t color = (blinking && bl.blink_state) ? bl.color_blink : bl.color_normal;
      LabelSetColor(bl.obj, color);
    }
    pros::delay(50);
  }
}

void BlinkLabelAdd(const char* page,
                       int x, int y,
                       const char* (*getter)(),
                       bool (*should_blink)(),
                       uint32_t color_normal,
                       uint32_t color_blink,
                       int interval_ms,
                       int font_size) {
  if (_blink_lbl_count >= _MAX_BLINK_LBLS) return;
  lv_obj_t* s = _find_screen(page);
  if (!s) return;

  const char* init = getter ? getter() : "";
  lv_obj_t* lbl = CreateLabel(s, x, y, init ? init : "", font_size, color_normal);

  _BlinkLbl& bl   = _blink_lbls[_blink_lbl_count++];
  bl.obj          = lbl;
  bl.getter       = getter;
  bl.should_blink = should_blink;
  bl.color_normal = color_normal;
  bl.color_blink  = color_blink;
  bl.interval_ms  = interval_ms;
  bl.last_ms      = pros::millis();
  bl.blink_state  = false;

  if (!_blink_task_started) {
    static pros::Task task(_blink_task, nullptr, "Blink Labels");
    _blink_task_started = true;
  }
  _build_breathe();
}

// ── Port names ────────────────────────────────────────────────────────────────
// Custom names shown on port detail screens when using ScreenStart().
// Call PortNames() in build_screens() before PageShow().

static const char* _global_port_names[20] = {};

void PortNames(const char* names[20]) {
  for (int i = 0; i < 20; i++)
    _global_port_names[i] = (names && names[i]) ? names[i] : nullptr;
}

// ── Popup live labels ─────────────────────────────────────────────────────────
// Labels that exist only while a specific popup is open.
// Registered at build_screens() time; activated when the popup opens,
// deactivated (and pointer cleared) when it closes — no use-after-free.

#define _MAX_POPUP_LIVE 8

struct _PopupLiveLblSpec {
  char        popup_name[32];
  int         x, y, font_size;
  uint32_t    color;
  _LiveGetter getter;
  int         interval_ms;
  // Set/cleared at popup open/close — not valid outside that window
  lv_obj_t*   obj;
  uint32_t    last_ms;
  bool        active;
};

static _PopupLiveLblSpec _popup_live_specs[_MAX_POPUP_LIVE] = {};
static int               _popup_live_count        = 0;
static bool              _popup_live_task_started = false;

static void _popup_live_task(void*) {
  while (true) {
    if (_building) { pros::delay(20); continue; }
    uint32_t now = pros::millis();
    for (int i = 0; i < _popup_live_count; i++) {
      _PopupLiveLblSpec& s = _popup_live_specs[i];
      if (!s.active) continue;
      if (now - s.last_ms < (uint32_t)s.interval_ms) continue;
      s.last_ms = now;
      // Read and use s.obj under mutex so _close_active_popup() cannot null it
      // and delete the widget between our null-check and our lv_label_set_text call.
      if (!_mutex || !_mutex->take(10)) continue;
      lv_obj_t* obj = s.obj;
      if (obj && s.active && s.getter) {
        const char* text = s.getter();
        if (text) lv_label_set_text(obj, text);  // called directly — mutex already held
      }
      _mutex->give();
    }
    pros::delay(25);
  }
}

void PopupLabelAdd(const char* popup_name,
                       int x, int y,
                       const char* (*getter)(),
                       int interval_ms,
                       int font_size,
                       uint32_t color) {
  if (_popup_live_count >= _MAX_POPUP_LIVE || getter == nullptr) return;
  _PopupLiveLblSpec& s = _popup_live_specs[_popup_live_count++];
  strncpy(s.popup_name, popup_name, 31); s.popup_name[31] = '\0';
  s.x           = x;
  s.y           = y;
  s.font_size   = font_size;
  s.color       = color;
  s.getter      = getter;
  s.interval_ms = interval_ms;
  s.obj         = nullptr;
  s.active      = false;

  if (!_popup_live_task_started) {
    static pros::Task task(_popup_live_task, nullptr, "Popup Live");
    _popup_live_task_started = true;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Declarative builder  (PageAdd / ButtonAdd / LabelAdd / PageShow)
// ═══════════════════════════════════════════════════════════════════════════════

// ── Page transition animation ─────────────────────────────────────────────────
// Set via PageAnim(). Applied every time a page is loaded.
static int _page_anim_type = UI_ANIM_NONE;

void PageAnim(int anim_type) {
  _page_anim_type = anim_type;
}

// Internal helper — loads a screen using whatever transition is configured.
static void _scr_load(lv_obj_t* s) {
  switch (_page_anim_type) {
    case UI_ANIM_FADE:
      lv_scr_load_anim(s, LV_SCR_LOAD_ANIM_FADE_ON, 200, 0, false);
      break;
    case UI_ANIM_SLIDE:
      lv_scr_load_anim(s, LV_SCR_LOAD_ANIM_MOVE_LEFT, 200, 0, false);
      break;
    default:
      lv_scr_load(s);
      break;
  }
}

#define _MAX_PAGES       16
#define _MAX_BTN_DATA    96   // shared pool for page buttons + popup buttons
#define _MAX_POPUPS       8
#define _MAX_POPUP_ITEMS 16

struct _PageRec { char name[32]; lv_obj_t* screen; int anim_slot; int lbl_anim_slot; uint32_t bg_color; };
struct _BtnData { char goes_to[32]; int auton_idx; void (*on_tap)(); int press_anim; };

// One pending item (button or label) stored before a popup is shown
struct _PopupItem {
  bool     is_btn;
  int      x, y, w, h;
  uint32_t color;
  char     text[32];
  char     goes_to[32];
  int      auton_idx;
  int      font_size;        // labels only
  int      corner_radius;    // buttons only
  void     (*on_tap)();      // optional callback fired when tapped
  int      press_anim;     // UI_PRESS_* style
};

struct _PopupDef {
  char       name[32];
  int        w, h;
  uint32_t   bg_color, border_color;
  char       title[32];
  _PopupItem items[_MAX_POPUP_ITEMS];
  int        item_count;
  lv_obj_t*  widget;    // nullptr when hidden
};

static _PageRec  _page_reg[_MAX_PAGES]    = {};
static int       _page_count              = 0;
static _BtnData  _btn_pool[_MAX_BTN_DATA] = {};
static int       _btn_pool_n              = 0;
static uint32_t  _dcl_bg                  = UI_DARK_BG;
static int       _dcl_press_anim          = UI_PRESS_NONE;
static _PopupDef _popup_defs[_MAX_POPUPS] = {};
static int       _popup_count             = 0;
static _PopupDef* _active_popup           = nullptr;

static _PageRec* _find_page_rec(const char* name) {
  for (int i = 0; i < _page_count; i++)
    if (strncmp(_page_reg[i].name, name, 31) == 0)
      return &_page_reg[i];
  return nullptr;
}

static lv_obj_t* _find_screen(const char* name) {
  _PageRec* r = _find_page_rec(name);
  return r ? r->screen : nullptr;
}

static _PopupDef* _find_popup(const char* name) {
  for (int i = 0; i < _popup_count; i++)
    if (strncmp(_popup_defs[i].name, name, 31) == 0)
      return &_popup_defs[i];
  return nullptr;
}

static void _close_active_popup() {
  if (!_active_popup || !_active_popup->widget) {
    _active_popup = nullptr;
    return;
  }
  // Null out live label pointers under the same mutex the background task uses,
  // so the task cannot pass the null check and then have the object deleted under it.
  if (_mutex && _mutex->take(50)) {
    for (int i = 0; i < _popup_live_count; i++) {
      if (strncmp(_popup_live_specs[i].popup_name, _active_popup->name, 31) == 0) {
        _popup_live_specs[i].active = false;
        _popup_live_specs[i].obj   = nullptr;
      }
    }
    _mutex->give();
  }
  lv_obj_del(_active_popup->widget);
  _active_popup->widget = nullptr;
  _active_popup = nullptr;
}

static void _open_popup(const char* name);  // forward declaration

// Single generic nav/auton callback — handles pages, popups, close, and on_tap
static void _dcl_btn_cb(lv_event_t* e) {
  _BtnData* d = (_BtnData*)lv_event_get_user_data(e);
  if (!d) return;
  if (d->auton_idx >= 0) _selected_auton = d->auton_idx;
  if (d->on_tap) d->on_tap();   // call custom callback before navigation

  const char* target = d->goes_to;
  if (!target[0]) return;

  if (strcmp(target, "close") == 0) {
    // Close the open popup and stay on the current page
    _close_active_popup();
  } else if (strncmp(target, "popup:", 6) == 0) {
    // Open a popup by name — e.g. goes_to = "popup:confirm"
    _open_popup(target + 6);
  } else {
    // Navigate to a page — also closes any open popup
    _close_active_popup();
    lv_obj_t* s = _find_screen(target);
    if (s) _scr_load(s);
  }
}

// Build and display a popup over the current screen
static void _open_popup(const char* name) {
  _close_active_popup();
  _PopupDef* pd = _find_popup(name);
  if (!pd) return;

  pd->widget = lv_obj_create(lv_scr_act());
  lv_obj_set_size(pd->widget, pd->w, pd->h);
  lv_obj_align(pd->widget, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(pd->widget, lv_color_hex(pd->bg_color), 0);
  lv_obj_set_style_border_color(pd->widget, lv_color_hex(pd->border_color), 0);
  lv_obj_set_style_border_width(pd->widget, 3, 0);
  lv_obj_set_style_radius(pd->widget, 12, 0);
  lv_obj_clear_flag(pd->widget, LV_OBJ_FLAG_SCROLLABLE);

  if (pd->title[0]) {
    lv_obj_t* ttl = lv_label_create(pd->widget);
    lv_label_set_text(ttl, pd->title);
    lv_obj_set_style_text_font(ttl, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(ttl, lv_color_hex(pd->border_color), 0);
    lv_obj_align(ttl, LV_ALIGN_TOP_MID, 0, 10);
  }

  for (int i = 0; i < pd->item_count; i++) {
    _PopupItem& it = pd->items[i];
    if (it.is_btn && _btn_pool_n < _MAX_BTN_DATA) {
      _BtnData* bd   = &_btn_pool[_btn_pool_n++];
      bd->auton_idx  = it.auton_idx;
      bd->on_tap     = it.on_tap;
      bd->press_anim = it.press_anim;
      bd->goes_to[0] = '\0';
      if (it.goes_to[0]) { strncpy(bd->goes_to, it.goes_to, 31); bd->goes_to[31] = '\0'; }
      lv_obj_t* pbtn = CreateButton(pd->widget, it.x, it.y, it.w, it.h, it.color, it.text, _dcl_btn_cb, bd, it.corner_radius);
      if (pbtn && it.press_anim != UI_PRESS_NONE)
        lv_obj_add_event_cb(pbtn, _press_anim_cb, LV_EVENT_PRESSED, (void*)(intptr_t)it.press_anim);
    } else if (!it.is_btn) {
      CreateLabel(pd->widget, it.x, it.y, it.text, it.font_size, it.color);
    }
  }

  // Activate live labels registered for this popup
  for (int i = 0; i < _popup_live_count; i++) {
    _PopupLiveLblSpec& s = _popup_live_specs[i];
    if (strncmp(s.popup_name, name, 31) != 0) continue;
    const char* init = s.getter ? s.getter() : nullptr;
    s.obj     = CreateLabel(pd->widget, s.x, s.y, init ? init : "", s.font_size, s.color);
    s.last_ms = pros::millis();
    s.active  = true;
  }

  _active_popup = pd;
}

void BgColor(uint32_t color) {
  _dcl_bg = color;
}

void ButtonPressStyle(int press_anim) {
  _dcl_press_anim = press_anim;
}

void PageAdd(const char* name) {
  if (_page_count >= _MAX_PAGES) return;
  _PageRec& r = _page_reg[_page_count++];
  strncpy(r.name, name, 31);
  r.name[31] = '\0';
  r.anim_slot     = 0;
  r.lbl_anim_slot = 0;
  r.bg_color      = _dcl_bg;
  r.screen = lv_obj_create(NULL);
  lv_obj_remove_style_all(r.screen);
  lv_obj_set_style_bg_color(r.screen, lv_color_hex(_dcl_bg), 0);
  lv_obj_set_style_bg_opa(r.screen, LV_OPA_COVER, 0);
  lv_obj_clear_flag(r.screen, LV_OBJ_FLAG_SCROLLABLE);
}

void PopupAdd(const char* name, int w, int h, const char* title,
              uint32_t bg_color, uint32_t border_color) {
  if (_popup_count >= _MAX_POPUPS) return;
  _PopupDef& pd      = _popup_defs[_popup_count++];
  strncpy(pd.name, name, 31);  pd.name[31] = '\0';
  strncpy(pd.title, title ? title : "", 31); pd.title[31] = '\0';
  pd.w            = w;
  pd.h            = h;
  pd.bg_color     = bg_color;
  pd.border_color = border_color;
  pd.item_count   = 0;
  pd.widget       = nullptr;
}

void ButtonAdd(const char* page,
            int x, int y, int w, int h,
            uint32_t color, const char* text,
            const char* goes_to,
            int  animated,
            int  auton_idx,
            int  corner_radius,
            void (*on_tap)()) {
  // Route to popup if the name matches a declared popup
  _PopupDef* pd = _find_popup(page);
  if (pd) {
    if (pd->item_count >= _MAX_POPUP_ITEMS) return;
    _PopupItem& it   = pd->items[pd->item_count++];
    it.is_btn        = true;
    it.x = x; it.y = y; it.w = w; it.h = h;
    it.color         = color;
    it.auton_idx     = auton_idx;
    it.corner_radius = corner_radius;
    it.on_tap        = on_tap;
    it.press_anim    = _dcl_press_anim;
    strncpy(it.text, text, 31); it.text[31] = '\0';
    it.goes_to[0] = '\0';
    if (goes_to) { strncpy(it.goes_to, goes_to, 31); it.goes_to[31] = '\0'; }
    return;
  }

  // Otherwise add to page
  _PageRec* pr = _find_page_rec(page);
  if (!pr || _btn_pool_n >= _MAX_BTN_DATA) return;

  _BtnData* d   = &_btn_pool[_btn_pool_n++];
  d->goes_to[0] = '\0';
  d->auton_idx  = auton_idx;
  d->on_tap     = on_tap;
  d->press_anim = _dcl_press_anim;
  if (goes_to) { strncpy(d->goes_to, goes_to, 31); d->goes_to[31] = '\0'; }

  lv_obj_t* btn = nullptr;
  if (animated == UI_ELEM_GROW) {
    btn = CreateButtonAnimated(pr->screen, x, y, w, h, color, text,
                               pr->anim_slot * 80, _dcl_btn_cb, d, corner_radius);
    pr->anim_slot++;
  } else {
    btn = CreateButton(pr->screen, x, y, w, h, color, text,
                       _dcl_btn_cb, d, corner_radius);
    if (animated != UI_ELEM_NONE && btn) {
      _apply_anim(btn, animated, x, y, pr->anim_slot * 80);
      pr->anim_slot++;
    }
  }
  if (btn && _dcl_press_anim != UI_PRESS_NONE)
    lv_obj_add_event_cb(btn, _press_anim_cb, LV_EVENT_PRESSED, (void*)(intptr_t)_dcl_press_anim);
  _build_breathe();
}

void LabelAdd(const char* page,
            int x, int y,
            const char* text,
            int font_size,
            uint32_t color,
            int animated) {
  // Route to popup if the name matches a declared popup
  _PopupDef* pd = _find_popup(page);
  if (pd) {
    if (pd->item_count >= _MAX_POPUP_ITEMS) return;
    _PopupItem& it = pd->items[pd->item_count++];
    it.is_btn    = false;
    it.x = x; it.y = y;
    it.font_size = font_size;
    it.color     = color;
    strncpy(it.text, text, 31); it.text[31] = '\0';
    return;
  }

  // Otherwise add to page
  _PageRec* pr = _find_page_rec(page);
  if (!pr) return;
  lv_obj_t* lbl = CreateLabel(pr->screen, x, y, text, font_size, color);

  if (animated != UI_ELEM_NONE && lbl) {
    // For labels, GROW (1/true) maps to FADE — growing text width looks wrong.
    int type = (animated == UI_ELEM_GROW) ? UI_ELEM_FADE : animated;
    _apply_anim(lbl, type, x, y, pr->lbl_anim_slot * 60);
    pr->lbl_anim_slot++;
  }
  _build_breathe();
}

void BoxAdd(const char* page,
            int x, int y, int w, int h,
            uint32_t color,
            int radius,
            uint32_t border_color,
            int border_width) {
  lv_obj_t* s = _find_screen(page);
  if (s) { CreateBox(s, x, y, w, h, color, radius, border_color, border_width); _build_breathe(); }
}

void CircleAdd(const char* page,
               int x, int y, int diameter,
               uint32_t color,
               uint32_t border_color,
               int border_width) {
  lv_obj_t* s = _find_screen(page);
  if (s) { CreateCircle(s, x, y, diameter, color, border_color, border_width); _build_breathe(); }
}

void SquareAdd(const char* page,
               int x, int y, int size,
               uint32_t color,
               uint32_t border_color,
               int border_width) {
  lv_obj_t* s = _find_screen(page);
  if (s) { CreateBox(s, x, y, size, size, color, 0, border_color, border_width); _build_breathe(); }
}

void RoundedBoxAdd(const char* page,
                       int x, int y, int size,
                       uint32_t color,
                       int radius,
                       uint32_t border_color,
                       int border_width) {
  lv_obj_t* s = _find_screen(page);
  if (s) { CreateBox(s, x, y, size, size, color, radius, border_color, border_width); _build_breathe(); }
}

void TriangleAdd(const char* page,
                 int x, int y, int w, int h,
                 uint32_t color) {
  lv_obj_t* s = _find_screen(page);
  if (s) { CreateTriangle(s, x, y, w, h, color); _build_breathe(); }
}

void PageShow(const char* page) {
  _building = false;  // build complete — background tasks may now call LVGL
  lv_obj_t* s = _find_screen(page);
  if (s) _scr_load(s);
}

// ── Progress bar ──────────────────────────────────────────────────────────────
// A horizontal track + fill bar.  The fill width reflects a 0–100 getter value.
// fill_color turns to warn_color when the value drops at or below warn_threshold.

#define _MAX_LIVE_BARS 8
typedef int (*_IntGetter)();

struct _LiveBar {
  lv_obj_t*  fill_obj;
  int        max_w;
  _IntGetter getter;
  int        interval_ms;
  uint32_t   last_ms;
  uint32_t   fill_color;
  uint32_t   warn_color;
  int        warn_threshold;
};

static _LiveBar _live_bars[_MAX_LIVE_BARS] = {};
static int      _live_bar_count            = 0;
static bool     _live_bar_task_started     = false;

static void _live_bar_task(void*) {
  while (true) {
    if (_building) { pros::delay(20); continue; }
    uint32_t now = pros::millis();
    for (int i = 0; i < _live_bar_count; i++) {
      _LiveBar& b = _live_bars[i];
      if (now - b.last_ms < (uint32_t)b.interval_ms) continue;
      b.last_ms   = now;
      int val     = b.getter ? b.getter() : 0;
      if (val < 0)   val = 0;
      if (val > 100) val = 100;
      int new_w      = (b.max_w * val) / 100;
      uint32_t color = (b.warn_color && val <= b.warn_threshold) ? b.warn_color : b.fill_color;
      if (_mutex && _mutex->take(5)) {
        if (b.fill_obj) {
          lv_obj_set_width(b.fill_obj, new_w);
          lv_obj_set_style_bg_color(b.fill_obj, lv_color_hex(color), 0);
        }
        _mutex->give();
      }
    }
    pros::delay(50);
  }
}

void BarAdd(const char* page,
                 int x, int y, int w, int h,
                 int (*getter)(),
                 int interval_ms,
                 uint32_t fill_color,
                 uint32_t bg_color,
                 int radius,
                 uint32_t warn_color,
                 int warn_threshold) {
  if (_live_bar_count >= _MAX_LIVE_BARS || !getter) return;
  lv_obj_t* s = _find_screen(page);
  if (!s) return;

  // Background track
  lv_obj_t* track = lv_obj_create(s);
  lv_obj_set_size(track, w, h);
  lv_obj_set_pos(track, x, y);
  lv_obj_set_style_bg_color(track, lv_color_hex(bg_color), 0);
  lv_obj_set_style_radius(track, radius, 0);
  lv_obj_set_style_border_width(track, 0, 0);
  lv_obj_set_style_pad_all(track, 0, 0);
  lv_obj_clear_flag(track, LV_OBJ_FLAG_SCROLLABLE);

  // Fill bar — child of track, grows from left edge
  int init_val       = getter();
  if (init_val < 0)   init_val = 0;
  if (init_val > 100) init_val = 100;
  int      init_w     = (w * init_val) / 100;
  uint32_t init_color = (warn_color && init_val <= warn_threshold) ? warn_color : fill_color;

  lv_obj_t* fill = lv_obj_create(track);
  lv_obj_set_size(fill, init_w, h);
  lv_obj_set_pos(fill, 0, 0);
  lv_obj_set_style_bg_color(fill, lv_color_hex(init_color), 0);
  lv_obj_set_style_radius(fill, radius, 0);
  lv_obj_set_style_border_width(fill, 0, 0);
  lv_obj_set_style_pad_all(fill, 0, 0);
  lv_obj_clear_flag(fill, LV_OBJ_FLAG_SCROLLABLE);

  _LiveBar& b      = _live_bars[_live_bar_count++];
  b.fill_obj       = fill;
  b.max_w          = w;
  b.getter         = getter;
  b.interval_ms    = interval_ms;
  b.last_ms        = pros::millis();
  b.fill_color     = fill_color;
  b.warn_color     = warn_color;
  b.warn_threshold = warn_threshold;

  if (!_live_bar_task_started) {
    static pros::Task task(_live_bar_task, nullptr, "Live Bars");
    _live_bar_task_started = true;
  }
  _build_breathe();
}

// ── Toggle button ─────────────────────────────────────────────────────────────
// Stays highlighted in color_on / shows text_on while active.
// Tapping again reverts to color_off / text_off.
// on_toggle(bool state) is called with the new state after each tap.

#define _MAX_TOGGLE_BTNS 8

struct _ToggleBtn {
  lv_obj_t* btn;
  lv_obj_t* lbl;
  uint32_t  color_off, color_on;
  char      text_off[32], text_on[32];
  bool      state;
  void (*on_toggle)(bool);
};

static _ToggleBtn _toggle_btns[_MAX_TOGGLE_BTNS] = {};
static int        _toggle_btn_count              = 0;

static void _toggle_btn_cb(lv_event_t* e) {
  _ToggleBtn* t = (_ToggleBtn*)lv_event_get_user_data(e);
  t->state = !t->state;
  lv_obj_set_style_bg_color(t->btn, lv_color_hex(t->state ? t->color_on : t->color_off), 0);
  lv_label_set_text(t->lbl, t->state ? t->text_on : t->text_off);
  if (t->on_toggle) t->on_toggle(t->state);
}

void ToggleAdd(const char* page,
                   int x, int y, int w, int h,
                   uint32_t color_off, uint32_t color_on,
                   const char* text_off, const char* text_on,
                   int animated,
                   int corner_radius,
                   void (*on_toggle)(bool),
                   bool init_state) {
  if (_toggle_btn_count >= _MAX_TOGGLE_BTNS) return;
  lv_obj_t* s = _find_screen(page);
  if (!s) return;

  _ToggleBtn& t = _toggle_btns[_toggle_btn_count++];
  t.state       = init_state;
  t.color_off   = color_off;
  t.color_on    = color_on;
  t.on_toggle   = on_toggle;
  strncpy(t.text_off, text_off ? text_off : "", 31); t.text_off[31] = '\0';
  strncpy(t.text_on,  text_on  ? text_on  : "", 31); t.text_on[31]  = '\0';

  int r = (corner_radius == UI_SHAPE_PILL) ? h / 2 : corner_radius;

  lv_obj_t* btn = lv_btn_create(s);
  lv_obj_set_height(btn, h);
  lv_obj_set_style_bg_color(btn, lv_color_hex(init_state ? color_on : color_off), 0);
  lv_obj_set_style_radius(btn, r, 0);
  lv_obj_set_style_border_width(btn, 0, 0);
  lv_obj_set_style_shadow_width(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(btn, _toggle_btn_cb, LV_EVENT_CLICKED, &t);
  t.btn = btn;

  lv_obj_t* lbl = lv_label_create(btn);
  lv_label_set_text(lbl, init_state ? (text_on ? text_on : "") : (text_off ? text_off : ""));
  lv_obj_set_style_text_font(lbl, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(lbl, lv_color_hex(UI_WHITE), 0);
  lv_obj_center(lbl);
  t.lbl = lbl;

  _PageRec* pr = _find_page_rec(page);

  if (animated == UI_ELEM_GROW) {
    lv_obj_set_width(btn, 0);
    lv_obj_set_pos(btn, x, y);
    int delay = pr ? pr->anim_slot * 80 : 0;
    if (pr) pr->anim_slot++;
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, btn);
    lv_anim_set_values(&anim, 0, w);
    lv_anim_set_exec_cb(&anim, _anim_width_cb);
    lv_anim_set_path_cb(&anim, lv_anim_path_overshoot);
    lv_anim_set_time(&anim, 300);
    lv_anim_set_delay(&anim, delay);
    lv_anim_start(&anim);
  } else {
    lv_obj_set_width(btn, w);
    lv_obj_set_pos(btn, x, y);
    if (animated != UI_ELEM_NONE) {
      int delay = pr ? pr->anim_slot * 80 : 0;
      if (pr) pr->anim_slot++;
      _apply_anim(btn, animated, x, y, delay);
    }
  }
  _build_breathe();
}

// ── Live color dot ────────────────────────────────────────────────────────────
// A circle whose fill color is returned by getter() every interval_ms.
// Useful for at-a-glance status: return UI_GREEN when ready, UI_RED when not.

#define _MAX_LIVE_DOTS 8
typedef uint32_t (*_ColorGetter)();

struct _LiveDot {
  lv_obj_t*    obj;
  _ColorGetter getter;
  int          interval_ms;
  uint32_t     last_ms;
};

static _LiveDot _live_dots[_MAX_LIVE_DOTS] = {};
static int      _live_dot_count            = 0;
static bool     _live_dot_task_started     = false;

static void _live_dot_task(void*) {
  while (true) {
    if (_building) { pros::delay(20); continue; }
    uint32_t now = pros::millis();
    for (int i = 0; i < _live_dot_count; i++) {
      _LiveDot& d = _live_dots[i];
      if (now - d.last_ms < (uint32_t)d.interval_ms) continue;
      d.last_ms      = now;
      uint32_t color = d.getter ? d.getter() : UI_GRAY;
      if (_mutex && _mutex->take(5)) {
        if (d.obj) lv_obj_set_style_bg_color(d.obj, lv_color_hex(color), 0);
        _mutex->give();
      }
    }
    pros::delay(50);
  }
}

void DotAdd(const char* page,
                 int x, int y, int diameter,
                 uint32_t (*getter)(),
                 int interval_ms) {
  if (_live_dot_count >= _MAX_LIVE_DOTS || !getter) return;
  lv_obj_t* s = _find_screen(page);
  if (!s) return;

  lv_obj_t* dot = lv_obj_create(s);
  lv_obj_set_size(dot, diameter, diameter);
  lv_obj_set_pos(dot, x, y);
  lv_obj_set_style_bg_color(dot, lv_color_hex(getter()), 0);
  lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(dot, 0, 0);
  lv_obj_set_style_pad_all(dot, 0, 0);
  lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);

  _LiveDot& d   = _live_dots[_live_dot_count++];
  d.obj         = dot;
  d.getter      = getter;
  d.interval_ms = interval_ms;
  d.last_ms     = pros::millis();

  if (!_live_dot_task_started) {
    static pros::Task task(_live_dot_task, nullptr, "Live Dots");
    _live_dot_task_started = true;
  }
  _build_breathe();
}

// ── Countdown (struct forward-declared here so PageClear can null its objects) ─
#define _MAX_COUNTDOWNS 4
struct _CountdownRec {
  lv_obj_t* obj;
  int       total_ms;
  int       remaining_ms;
  uint32_t  color_normal;
  uint32_t  color_warn;
  int       warn_ms;
  void    (*on_expire)();
  bool      running;
  uint32_t  last_tick;
};
static _CountdownRec _countdowns[_MAX_COUNTDOWNS] = {};
static int           _countdown_count             = 0;
static bool          _countdown_task_started      = false;

// ── PageClear ─────────────────────────────────────────────────────────────────
// Removes all children from a page's screen and resets its animation counters.
// Call from an on_tap callback before rebuilding the page with new content.
// Safely nulls any live-update registrations that pointed into this screen.

void PageClear(const char* page) {
  _PageRec* pr = _find_page_rec(page);
  if (!pr) return;
  for (int i = 0; i < _live_lbl_count;  i++)
    if (_live_lbls[i].obj  && lv_obj_get_screen(_live_lbls[i].obj)  == pr->screen)
      _live_lbls[i].obj  = nullptr;
  for (int i = 0; i < _blink_lbl_count; i++)
    if (_blink_lbls[i].obj && lv_obj_get_screen(_blink_lbls[i].obj) == pr->screen)
      _blink_lbls[i].obj = nullptr;
  for (int i = 0; i < _live_bar_count;  i++)
    if (_live_bars[i].fill_obj && lv_obj_get_screen(_live_bars[i].fill_obj) == pr->screen)
      _live_bars[i].fill_obj = nullptr;
  for (int i = 0; i < _live_dot_count;  i++)
    if (_live_dots[i].obj  && lv_obj_get_screen(_live_dots[i].obj)  == pr->screen)
      _live_dots[i].obj  = nullptr;
  for (int i = 0; i < _countdown_count; i++)
    if (_countdowns[i].obj && lv_obj_get_screen(_countdowns[i].obj) == pr->screen) {
      _countdowns[i].obj     = nullptr;
      _countdowns[i].running = false;  // stop orphaned timers so they can't interfere
    }
  pr->anim_slot     = 0;
  pr->lbl_anim_slot = 0;
  lv_obj_set_style_bg_color(pr->screen, lv_color_hex(pr->bg_color), 0);
  lv_obj_clean(pr->screen);
}

// ── Button grid ───────────────────────────────────────────────────────────────
// Fills a total_w × total_h area with item_count buttons arranged in `cols`
// columns.  Button sizes and positions are computed automatically.

void GridAdd(const char* page,
                 int x, int y, int total_w, int total_h,
                 int cols,
                 const ui_btn_item items[], int item_count,
                 int gap,
                 int animated,
                 int corner_radius) {
  if (item_count <= 0 || cols <= 0) return;
  int rows  = (item_count + cols - 1) / cols;
  int btn_w = (total_w - gap * (cols - 1)) / cols;
  int btn_h = (total_h - gap * (rows - 1)) / rows;
  for (int i = 0; i < item_count; i++) {
    int col = i % cols;
    int row = i / cols;
    ButtonAdd(page,
           x + col * (btn_w + gap),
           y + row * (btn_h + gap),
           btn_w, btn_h,
           items[i].color,
           items[i].text    ? items[i].text    : "",
           items[i].goes_to,
           animated,
           items[i].auton_idx,
           corner_radius,
           items[i].on_tap);
  }
}

// ── Slider ────────────────────────────────────────────────────────────────────
// A horizontal drag slider for picking an integer value between min and max.
// The current value is shown to the right of the slider automatically.
// on_change fires every time the value changes while dragging.

#define _MAX_SLIDERS 8

struct _SliderData {
  void      (*on_change)(int);
  lv_obj_t*   val_label;
};

static _SliderData _sliders[_MAX_SLIDERS] = {};
static int         _slider_count          = 0;

static void _slider_cb(lv_event_t* e) {
  _SliderData* d = (_SliderData*)lv_event_get_user_data(e);
  int val = (int)lv_slider_get_value(lv_event_get_target(e));
  if (d->val_label) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%d", val);
    lv_label_set_text(d->val_label, buf);
  }
  if (d->on_change) d->on_change(val);
}

void SliderAdd(const char* page,
               int x, int y, int w, int h,
               int min_val, int max_val, int default_val,
               uint32_t color,
               void (*on_change)(int)) {
  if (_slider_count >= _MAX_SLIDERS) return;
  lv_obj_t* s = _find_screen(page);
  if (!s) return;

  lv_obj_t* sl = lv_slider_create(s);
  lv_obj_set_size(sl, w, h);
  lv_obj_set_pos(sl, x, y);
  lv_slider_set_range(sl, min_val, max_val);
  lv_slider_set_value(sl, default_val, LV_ANIM_OFF);

  lv_obj_set_style_bg_color(sl,    lv_color_hex(0x444444),  LV_PART_MAIN);
  lv_obj_set_style_bg_opa(sl,      LV_OPA_COVER,            LV_PART_MAIN);
  lv_obj_set_style_radius(sl,      h / 2,                   LV_PART_MAIN);
  lv_obj_set_style_border_width(sl, 0,                      LV_PART_MAIN);

  lv_obj_set_style_bg_color(sl,    lv_color_hex(color),     LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(sl,      LV_OPA_COVER,            LV_PART_INDICATOR);
  lv_obj_set_style_radius(sl,      h / 2,                   LV_PART_INDICATOR);

  lv_obj_set_style_bg_color(sl,    lv_color_hex(UI_WHITE),  LV_PART_KNOB);
  lv_obj_set_style_bg_opa(sl,      LV_OPA_COVER,            LV_PART_KNOB);
  lv_obj_set_style_radius(sl,      LV_RADIUS_CIRCLE,        LV_PART_KNOB);
  lv_obj_set_style_pad_all(sl,     6,                       LV_PART_KNOB);
  lv_obj_set_style_border_width(sl, 0,                      LV_PART_KNOB);

  char init_buf[16];
  snprintf(init_buf, sizeof(init_buf), "%d", default_val);
  lv_obj_t* val_lbl = CreateLabel(s, x + w + 10, y - 1, init_buf, 18, UI_WHITE);

  _SliderData& d = _sliders[_slider_count++];
  d.on_change  = on_change;
  d.val_label  = val_lbl;

  lv_obj_add_event_cb(sl, _slider_cb, LV_EVENT_VALUE_CHANGED, &d);
}

// ── Countdown timer ───────────────────────────────────────────────────────────
// Displays a live MM:SS countdown. Starts stopped — call CountdownStart()
// to begin counting (e.g. from a button on_tap or from autonomous()).

static void _countdown_task(void*) {
  while (true) {
    if (_building) { pros::delay(20); continue; }
    uint32_t now = pros::millis();
    for (int i = 0; i < _countdown_count; i++) {
      _CountdownRec& c = _countdowns[i];
      if (!c.running) continue;

      int elapsed = (int)(now - c.last_tick);
      if (elapsed < 0) elapsed = 0;
      c.last_tick = now;

      if (c.remaining_ms <= elapsed) {
        c.remaining_ms = 0;
        c.running      = false;
        if (c.on_expire) c.on_expire();
      } else {
        c.remaining_ms -= elapsed;
      }

      int total_secs = (c.remaining_ms + 999) / 1000;
      int mins = total_secs / 60;
      int secs = total_secs % 60;
      char buf[8];
      snprintf(buf, sizeof(buf), "%d:%02d", mins, secs);

      uint32_t color = (c.remaining_ms <= (uint32_t)c.warn_ms)
                       ? c.color_warn : c.color_normal;
      if (c.obj) {
        LabelSetText(c.obj, buf);
        LabelSetColor(c.obj, color);
      }
    }
    pros::delay(100);
  }
}

void CountdownAdd(const char* page,
                  int x, int y,
                  int seconds,
                  int font_size,
                  uint32_t color,
                  uint32_t warn_color,
                  int warn_seconds,
                  void (*on_expire)()) {
  if (_countdown_count >= _MAX_COUNTDOWNS) return;
  lv_obj_t* s = _find_screen(page);
  if (!s) return;

  char init_buf[8];
  snprintf(init_buf, sizeof(init_buf), "%d:%02d", seconds / 60, seconds % 60);
  lv_obj_t* lbl = CreateLabel(s, x, y, init_buf, font_size, color);

  _CountdownRec& c = _countdowns[_countdown_count++];
  c.obj          = lbl;
  c.total_ms     = seconds * 1000;
  c.remaining_ms = seconds * 1000;
  c.color_normal = color;
  c.color_warn   = warn_color;
  c.warn_ms      = warn_seconds * 1000;
  c.on_expire    = on_expire;
  c.running      = false;
  c.last_tick    = 0;

  if (!_countdown_task_started) {
    static pros::Task task(_countdown_task, nullptr, "Countdown");
    _countdown_task_started = true;
  }
}

void CountdownStart() {
  for (int i = 0; i < _countdown_count; i++) {
    if (!_countdowns[i].obj) continue;  // skip orphaned records
    _countdowns[i].remaining_ms = _countdowns[i].total_ms;
    _countdowns[i].last_tick    = pros::millis();
    _countdowns[i].running      = true;
  }
}

void CountdownStop() {
  for (int i = 0; i < _countdown_count; i++) {
    if (!_countdowns[i].obj) continue;
    _countdowns[i].running = false;
  }
}

int CountdownRemaining() {
  if (_countdown_count == 0) return 0;
  return (_countdowns[0].remaining_ms + 999) / 1000;
}

// ── Field map selector ────────────────────────────────────────────────────────
// Draws a square field diagram and places circular auton-selector buttons at
// normalized field coordinates (0.0 = top/left edge, 1.0 = bottom/right edge).
// Teams place one marker per starting position — tapping it selects that auton.

void FieldMapAdd(const char* page,
                  int x, int y, int size,
                  const ui_field_pos positions[], int count,
                  uint32_t field_color,
                  uint32_t border_color,
                  int marker_size) {
  // Field background — any game-specific zones/lines should be drawn by the
  // caller using BoxAdd() BEFORE this call so markers render on top.
  BoxAdd(page, x, y, size, size, field_color, 4, border_color, 2);

  // Circular position markers, clamped so they stay fully inside the field
  int range = size - marker_size;
  for (int i = 0; i < count; i++) {
    const ui_field_pos& p = positions[i];
    float fx = p.field_x < 0.0f ? 0.0f : (p.field_x > 1.0f ? 1.0f : p.field_x);
    float fy = p.field_y < 0.0f ? 0.0f : (p.field_y > 1.0f ? 1.0f : p.field_y);
    ButtonAdd(page,
           x + (int)(fx * range),
           y + (int)(fy * range),
           marker_size, marker_size,
           p.color,
           p.label    ? p.label    : "",
           p.goes_to,
           UI_ELEM_NONE,
           p.auton_idx,
           UI_SHAPE_PILL,
           p.on_tap);
  }
}

// ── Spinner ───────────────────────────────────────────────────────────────────
void SpinnerAdd(const char* page,
                int x, int y, int size,
                uint32_t color,
                int speed_ms,
                int arc_deg) {
  _PageRec* pr = _find_page_rec(page);
  if (!pr) return;

  lv_obj_t* sp = lv_spinner_create(pr->screen, (uint32_t)speed_ms, (uint32_t)arc_deg);
  lv_obj_set_size(sp, size, size);
  lv_obj_set_pos(sp, x, y);
  lv_obj_set_style_arc_color(sp, lv_color_hex(color),    LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(sp, size / 10 + 2,          LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(sp, lv_color_hex(0x3A3A3A), LV_PART_MAIN);
  lv_obj_set_style_arc_width(sp, size / 10 + 2,          LV_PART_MAIN);
  _build_breathe();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ScreenStart() — automatic two-tab screen builder (auton selector + port grid)
// ═══════════════════════════════════════════════════════════════════════════════

static const char** _auton_names        = nullptr;
static uint32_t     _port_dots_copy[20] = {};

static lv_obj_t*    _page1           = nullptr;
static lv_obj_t*    _page2           = nullptr;
static lv_obj_t*    _detail_screen   = nullptr;
static lv_obj_t*    _port_buttons[20] = {};

static lv_obj_t*    _detail_m_lbl    = nullptr;   // "Motors Connected: X/Y"
static lv_obj_t*    _detail_s_lbl    = nullptr;   // "Sensors Connected: X/Y"

static void _delete_detail() {
  if (_detail_screen) {
    _detail_m_lbl  = nullptr;
    _detail_s_lbl  = nullptr;
    lv_obj_del(_detail_screen);
    _detail_screen = nullptr;
  }
}

// Count ports matching a dot color in PORT_DOTS
static int _count_color(uint32_t color) {
  int n = 0;
  for (int i = 0; i < 20; i++) if (_port_dots_copy[i] == color) n++;
  return n;
}

// Count ports matching a dot color that are also physically connected
static int _count_color_connected(uint32_t color) {
  int n = 0;
  for (int i = 0; i < 20; i++)
    if (_port_dots_copy[i] == color && PortConnected(i + 1)) n++;
  return n;
}

// ── Auton detail screen ───────────────────────────────────────────────────────
static void _on_back_autons(lv_event_t* e) {
  _delete_detail();
  _scr_load(_page1);
}

static void _open_auton_detail(int idx) {
  _delete_detail();
  _detail_screen = ScreenCreate();

  CreateButton(_detail_screen, 10, 10, 80, 35, UI_GOLD, "< Back", _on_back_autons);

  lv_obj_t* ttl = CreateLabel(_detail_screen, 0, 0, _auton_names[idx], 24, UI_WHITE);
  lv_obj_align(ttl, LV_ALIGN_TOP_MID, 0, 18);

  int tm = _count_color(UI_GOLD),    mc = _count_color_connected(UI_GOLD);
  int ts = _count_color(UI_PURPLE),  sc = _count_color_connected(UI_PURPLE);

  lv_obj_t* box = CreateBox(_detail_screen, 10, 55, 460, 175, UI_GOLD, 8);
  CreateLabel(box, 10, 5, "Auto Selected", 18, UI_GREEN);

  char buf[48];
  snprintf(buf, sizeof(buf), "Motors Connected: %d/%d", mc, tm);
  _detail_m_lbl = CreateLabel(box, 10, 30, buf, 18, mc < tm ? UI_RED : UI_GREEN);

  snprintf(buf, sizeof(buf), "Sensors Connected: %d/%d", sc, ts);
  _detail_s_lbl = CreateLabel(box, 10, 55, buf, 18, sc < ts ? UI_RED : UI_GREEN);

  _scr_load(_detail_screen);
}

static void _on_auton_btn(lv_event_t* e) {
  _selected_auton = (int)(intptr_t)lv_event_get_user_data(e);
  _open_auton_detail(_selected_auton);
}

// ── Port detail screen ────────────────────────────────────────────────────────
static void _on_back_ports(lv_event_t* e) {
  _delete_detail();
  _scr_load(_page2);
}

static void _on_port_btn(lv_event_t* e) {
  int port = (int)(intptr_t)lv_event_get_user_data(e);
  _delete_detail();
  _detail_screen = ScreenCreate();

  CreateButton(_detail_screen, 10, 10, 80, 35, UI_GOLD, "< Back", _on_back_ports);

  // Title: use custom port name if set, otherwise "Port N"
  const char* custom_name = _global_port_names[port - 1];
  char pbuf[32];
  snprintf(pbuf, sizeof(pbuf), "%s", custom_name ? custom_name : "");
  if (!custom_name) snprintf(pbuf, sizeof(pbuf), "Port %d", port);
  lv_obj_t* ptitle = CreateLabel(_detail_screen, 0, 0, pbuf, 24, UI_WHITE);
  lv_obj_align(ptitle, LV_ALIGN_CENTER, 0, -30);

  // Sub-title: show "Port N" when a custom name is displayed above
  if (custom_name) {
    char sub[16];
    snprintf(sub, sizeof(sub), "Port %d", port);
    lv_obj_t* psub = CreateLabel(_detail_screen, 0, 0, sub, 14, UI_GRAY);
    lv_obj_align(psub, LV_ALIGN_CENTER, 0, 5);
  }

  bool connected = PortConnected(port);
  lv_obj_t* pstatus = CreateLabel(_detail_screen, 0, 0,
    connected ? "Device Connected" : "No device connected",
    18, connected ? UI_GREEN : UI_RED);
  lv_obj_align(pstatus, LV_ALIGN_CENTER, 0, custom_name ? 35 : 10);

  if (connected) {
    char tbuf[32];
    snprintf(tbuf, sizeof(tbuf), "Temperature: %d F", MotorTempF(port));
    lv_obj_t* tlbl = CreateLabel(_detail_screen, 0, 0, tbuf, 18, UI_WHITE);
    lv_obj_align(tlbl, LV_ALIGN_CENTER, 0, custom_name ? 65 : 40);
  }

  _scr_load(_detail_screen);
}

// ── Background update task ────────────────────────────────────────────────────
static void _bg_task(void* param) {
  int cycle = 0;
  while (true) {
    cycle++;

    // Refresh port grid colors every 100 ms
    if (cycle % 2 == 0)
      PortStatusUpdate(_port_buttons);

    // Refresh motor/sensor counts on the detail screen every 200 ms
    if (cycle % 4 == 0 && _detail_m_lbl) {
      int tm = _count_color(UI_GOLD),    mc = _count_color_connected(UI_GOLD);
      int ts = _count_color(UI_PURPLE),  sc = _count_color_connected(UI_PURPLE);
      LabelSetTextFmt(_detail_m_lbl,  "Motors Connected: %d/%d",  mc, tm);
      LabelSetTextFmt(_detail_s_lbl,  "Sensors Connected: %d/%d", sc, ts);
      LabelSetColor(_detail_m_lbl, mc < tm ? UI_RED : UI_GREEN);
      LabelSetColor(_detail_s_lbl, sc < ts ? UI_RED : UI_GREEN);
    }

    pros::delay(50);
  }
}

// ── Public API ────────────────────────────────────────────────────────────────
void ScreenStart(const char* auton_names[], int num_autons, uint32_t port_dots[20],
              void (*on_auton_page)(lv_obj_t*),
              void (*on_port_page) (lv_obj_t*)) {
  _auton_names = auton_names;
  for (int i = 0; i < 20; i++) _port_dots_copy[i] = port_dots[i];

  // Page 1: Auton Selector — animated buttons, 2 columns
  _page1 = ScreenCreate();
  const int bw = 140, bh = 60, gap = 10;
  const int sx = (480 - (2 * bw + gap)) / 2, sy = 50;
  for (int i = 0; i < num_autons; i++)
    CreateButtonAnimated(_page1,
      sx + (i % 2) * (bw + gap),
      sy + (i / 2) * (bh + gap),
      bw, bh, UI_GOLD, auton_names[i],
      i * 80, _on_auton_btn, (void*)(intptr_t)i);
  if (on_auton_page) on_auton_page(_page1);   // user hook

  // Page 2: Port Status grid
  _page2 = ScreenCreate();
  PortGridAdd(_page2, 8, 45, _port_dots_copy, _on_port_btn, _port_buttons);
  if (on_port_page) on_port_page(_page2);     // user hook

  // Tab bar across both pages
  TabBarAdd(_page1, _page2, "Auton Selector", "Port Status");

  ScreenLoad(_page1);
  static pros::Task bg(_bg_task, nullptr, "Screen Update");
}

int SelectedAuton() {
  return _selected_auton;
}

void ForceSelectAuton(int idx) {
  _selected_auton = idx;
}

// ── PageBgColor ───────────────────────────────────────────────────────────────
void PageBgColor(const char* page, uint32_t color) {
  _PageRec* pr = _find_page_rec(page);
  if (!pr) return;
  lv_obj_set_style_bg_color(pr->screen, lv_color_hex(color), 0);
}

// ── CtrlFlush ─────────────────────────────────────────────────────────────────
// Immediately send all controller rows using the proper clear + delay + print
// sequence.  Blocks the calling task for ~220 ms while writes complete.
void CtrlFlush() {
  _ctrl_dirty = false;
  _ctrl_send_all();
}

// ── EngineDriverMode ──────────────────────────────────────────────────────────
// Locks/unlocks the brain screen for match use.
// active=true : shows "MATCH IN PROGRESS", disables touch.
// active=false: restores the previous screen and re-enables touch.
static lv_obj_t* _driver_screen  = nullptr;
static lv_obj_t* _prev_screen    = nullptr;

bool DriverModeActive() {
  return _driver_mode_active;
}

void EngineDriverMode(bool active) {
  _driver_mode_active = active;
  if (active) {
    _prev_screen = lv_scr_act();

    if (!_driver_screen) {
      _driver_screen = lv_obj_create(nullptr);
      lv_obj_remove_style_all(_driver_screen);
      lv_obj_set_style_bg_color(_driver_screen, lv_color_hex(UI_DARK_BG), 0);
      lv_obj_set_style_bg_opa(_driver_screen, LV_OPA_COVER, 0);
      lv_obj_clear_flag(_driver_screen, LV_OBJ_FLAG_SCROLLABLE);

      lv_obj_t* title = lv_label_create(_driver_screen);
      lv_label_set_text(title, "MATCH IN PROGRESS");
      lv_obj_set_style_text_color(title, lv_color_hex(UI_RED), 0);
      lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
      lv_obj_align(title, LV_ALIGN_CENTER, 0, -20);

      lv_obj_t* sub = lv_label_create(_driver_screen);
      lv_label_set_text(sub, "Brain screen locked");
      lv_obj_set_style_text_color(sub, lv_color_hex(UI_GRAY), 0);
      lv_obj_set_style_text_font(sub, &lv_font_montserrat_18, 0);
      lv_obj_align(sub, LV_ALIGN_CENTER, 0, 20);
    }

    lv_scr_load(_driver_screen);

    lv_indev_t* indev = lv_indev_get_next(nullptr);
    while (indev) {
      if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER)
        lv_indev_enable(indev, false);
      indev = lv_indev_get_next(indev);
    }
  } else {
    lv_indev_t* indev = lv_indev_get_next(nullptr);
    while (indev) {
      if (lv_indev_get_type(indev) == LV_INDEV_TYPE_POINTER)
        lv_indev_enable(indev, true);
      indev = lv_indev_get_next(indev);
    }

    if (_prev_screen) lv_scr_load(_prev_screen);
  }
}
