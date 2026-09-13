#pragma once

#include "api.h"

/////
// AI VISION TUNING - CHANGE THESE
/////
// The sensor's frame is 320 px wide, so dead centre is x = 160.  An object's
// "offset" below is how far right of centre it sits; negative means left.
constexpr int VISION_FRAME_WIDTH  = 320;
constexpr int VISION_FRAME_HEIGHT = 240;
constexpr int VISION_FRAME_CENTER = VISION_FRAME_WIDTH / 2;

// How close to centred counts as aligned, in pixels.  Smaller = fussier and
// slower to settle.  ~8 px is roughly 2 degrees.
constexpr int VISION_ALIGN_TOLERANCE = 8;

// Turn power per pixel of error.  Too high and it oscillates past the target,
// too low and it creeps.  Tune this the same way you'd tune a P constant.
constexpr double VISION_ALIGN_KP = 0.45;

// Below MIN the robot cannot overcome its own friction and just buzzes; above
// MAX it overshoots.  Both are in the -127..127 motor range.
constexpr int VISION_ALIGN_MIN_POWER = 18;
constexpr int VISION_ALIGN_MAX_POWER = 70;

// What counts as a target.  Turn one off to ignore that kind of detection.
constexpr bool VISION_USE_COLORS     = true;   // colour signatures you taught it
constexpr bool VISION_USE_AI_OBJECTS = true;   // the built-in game-element model

/////
// One detected thing, already decoded out of the sensor's union type.
/////
struct VisionTarget {
  bool found  = false;
  int  x      = 0;  // left edge, pixels
  int  y      = 0;  // top edge, pixels
  int  width  = 0;
  int  height = 0;
  int  area   = 0;  // width * height - used to pick the closest/biggest
  int  offset = 0;  // pixels right of frame centre; negative = left
  int  id     = 0;  // colour signature id, or AI class id
  bool is_color = false;  // true = colour signature, false = AI model object
};

// The largest thing the sensor can currently see.  Check .found first.
VisionTarget vision_largest();

// Turn in place until the largest target is centred.  Returns true if it got
// there, false on timeout or if nothing was ever seen.  Safe to call from an
// auton - it stops the drive before returning either way.
bool vision_align(int timeout_ms = 2000);

/////
// Live detection view (see the popup on the Status page)
/////
// Size of the drawn frame inside the popup, and how many boxes it will show at
// once.  Keep the same 4:3 ratio as the sensor or the boxes will look stretched.
constexpr int VISION_VIEW_W         = 264;
constexpr int VISION_VIEW_H         = 198;
constexpr int VISION_VIEW_MAX_BOXES = 8;

// Draw callback handed to PopupCanvasAdd() - you should not need to call this.
void vision_draw_view(lv_obj_t* canvas);
