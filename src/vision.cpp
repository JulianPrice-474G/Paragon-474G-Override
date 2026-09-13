#include "main.h"
#include "ui_engine.hpp"  // UI_GOLD / UI_GREEN / UI_GRAY, and PopupCanvasAdd

/////
// Reading the sensor
/////
// get_all_objects() hands back a mix of detection kinds in one vector, each
// carrying a union whose active member depends on the object's type.  Decode
// them into a plain struct so the rest of the code never touches the union.
VisionTarget vision_largest() {
  VisionTarget best;

  int count = ai_cam.get_object_count();
  if (count <= 0) return best;  // PROS_ERR (unplugged) or nothing in frame

  for (pros::AIVision::Object& o : ai_cam.get_all_objects()) {
    VisionTarget t;

    if (VISION_USE_COLORS &&
        pros::AIVision::is_type(o, pros::AivisionDetectType::color)) {
      t.is_color = true;
      t.x = o.object.color.xoffset;
      t.y = o.object.color.yoffset;
      t.width  = o.object.color.width;
      t.height = o.object.color.height;

    } else if (VISION_USE_AI_OBJECTS &&
               pros::AIVision::is_type(o, pros::AivisionDetectType::object)) {
      t.is_color = false;
      t.x = o.object.element.xoffset;
      t.y = o.object.element.yoffset;
      t.width  = o.object.element.width;
      t.height = o.object.element.height;

    } else {
      continue;  // a tag or a code - not something we aim at
    }

    t.found  = true;
    t.id     = o.id;
    t.area   = t.width * t.height;
    // x is the LEFT edge, so add half the width to get the middle of the object
    t.offset = (t.x + t.width / 2) - VISION_FRAME_CENTER;

    // Biggest wins, which in practice means closest.
    if (t.area > best.area) best = t;
  }

  return best;
}

/////
// Turning to face it
/////
// A plain proportional loop rather than chassis.pid_turn_set(): the target is
// measured in pixels and it moves, so there is no fixed heading to hand a PID.
// Every iteration re-reads the sensor and steers at whatever it sees now.
bool vision_align(int timeout_ms) {
  const uint32_t start = pros::millis();
  int lost_frames = 0;

  while ((int)(pros::millis() - start) < timeout_ms) {
    VisionTarget t = vision_largest();

    if (!t.found) {
      // Detections flicker, so coast briefly before giving up rather than
      // stuttering the drive on every dropped frame.
      if (++lost_frames > 25) break;   // ~250 ms with nothing at all
      chassis.drive_set(0, 0);
      pros::delay(ez::util::DELAY_TIME);
      continue;
    }
    lost_frames = 0;

    if (abs(t.offset) <= VISION_ALIGN_TOLERANCE) {
      chassis.drive_set(0, 0);
      return true;
    }

    int power = (int)(t.offset * VISION_ALIGN_KP);

    // Clamp into a band the drivetrain can actually act on.
    if (power >  VISION_ALIGN_MAX_POWER) power =  VISION_ALIGN_MAX_POWER;
    if (power < -VISION_ALIGN_MAX_POWER) power = -VISION_ALIGN_MAX_POWER;
    if (power > 0 && power <  VISION_ALIGN_MIN_POWER) power =  VISION_ALIGN_MIN_POWER;
    if (power < 0 && power > -VISION_ALIGN_MIN_POWER) power = -VISION_ALIGN_MIN_POWER;

    // Target right of centre (positive offset) means turn right: left side
    // forward, right side back.
    chassis.drive_set(power, -power);
    pros::delay(ez::util::DELAY_TIME);
  }

  chassis.drive_set(0, 0);
  return false;
}

/////
// Live view of what the sensor sees
/////
// The AI Vision sensor does NOT send video over the smart port - the only
// image-related call in the whole API is a USB overlay toggle, and that goes to
// a computer, not the brain.  So this is not a camera feed: it is every
// detection the sensor reports, drawn to scale in a box representing the frame.
// Position, size and kind are real; there is simply no photo behind them.
static lv_obj_t* _view_canvas = nullptr;
static lv_obj_t* _view_boxes[VISION_VIEW_MAX_BOXES] = {};
static lv_obj_t* _view_center = nullptr;

void vision_draw_view(lv_obj_t* canvas) {
  // A new container means the popup was reopened and the old children are gone.
  if (canvas != _view_canvas) {
    _view_canvas = canvas;

    lv_obj_set_style_bg_color(canvas, lv_color_hex(0x101010), 0);
    lv_obj_set_style_bg_opa(canvas, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(canvas, lv_color_hex(UI_GRAY), 0);
    lv_obj_set_style_border_width(canvas, 1, 0);

    for (int i = 0; i < VISION_VIEW_MAX_BOXES; i++) {
      lv_obj_t* b = lv_obj_create(canvas);
      lv_obj_remove_style_all(b);
      lv_obj_set_style_bg_opa(b, LV_OPA_TRANSP, 0);
      lv_obj_set_style_border_width(b, 2, 0);
      lv_obj_add_flag(b, LV_OBJ_FLAG_HIDDEN);
      _view_boxes[i] = b;
    }

    // Centre line LAST, so it sits above the detection boxes rather than under
    // them.  2 px and fully opaque: a 1 px half-transparent grey line on a near
    // black background is invisible on the brain's panel.
    _view_center = lv_obj_create(canvas);
    lv_obj_remove_style_all(_view_center);
    lv_obj_set_size(_view_center, 2, VISION_VIEW_H);
    lv_obj_set_pos(_view_center, VISION_VIEW_W / 2 - 1, 0);
    lv_obj_set_style_bg_color(_view_center, lv_color_hex(UI_WHITE), 0);
    lv_obj_set_style_bg_opa(_view_center, LV_OPA_COVER, 0);
  }

  int drawn = 0;
  int count = ai_cam.get_object_count();

  if (count > 0) {
    for (pros::AIVision::Object& o : ai_cam.get_all_objects()) {
      if (drawn >= VISION_VIEW_MAX_BOXES) break;

      int px, py, pw, ph;
      uint32_t color;
      if (pros::AIVision::is_type(o, pros::AivisionDetectType::color)) {
        px = o.object.color.xoffset;  py = o.object.color.yoffset;
        pw = o.object.color.width;    ph = o.object.color.height;
        color = UI_GOLD;                       // colour signature
      } else if (pros::AIVision::is_type(o, pros::AivisionDetectType::object)) {
        px = o.object.element.xoffset; py = o.object.element.yoffset;
        pw = o.object.element.width;   ph = o.object.element.height;
        color = UI_GREEN;                      // AI model object
      } else {
        continue;                              // tags and codes are not drawn
      }

      // Sensor pixels -> canvas pixels
      int x = px * VISION_VIEW_W / VISION_FRAME_WIDTH;
      int y = py * VISION_VIEW_H / VISION_FRAME_HEIGHT;
      int w = pw * VISION_VIEW_W / VISION_FRAME_WIDTH;
      int h = ph * VISION_VIEW_H / VISION_FRAME_HEIGHT;
      if (w < 3) w = 3;
      if (h < 3) h = 3;

      lv_obj_t* b = _view_boxes[drawn++];
      lv_obj_set_pos(b, x, y);
      lv_obj_set_size(b, w, h);
      lv_obj_set_style_border_color(b, lv_color_hex(color), 0);
      lv_obj_clear_flag(b, LV_OBJ_FLAG_HIDDEN);
    }
  }

  for (int i = drawn; i < VISION_VIEW_MAX_BOXES; i++)
    lv_obj_add_flag(_view_boxes[i], LV_OBJ_FLAG_HIDDEN);

  if (_view_center) lv_obj_move_foreground(_view_center);
}
