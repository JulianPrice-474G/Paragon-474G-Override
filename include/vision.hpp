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

/////
// VISION DRIVE TUNING - CHANGE THESE
/////
// For vision_drive(): drive a set distance while steering toward the pin.
//
// VISION_DRIVE_KP is the SENSITIVITY - how hard it steers per pixel the pin is
// off centre.  Too high and it weaves side to side chasing the pin; too low
// and it barely reacts.  Start low and raise it.
constexpr double VISION_DRIVE_KP = 0.25;

// The most the correction can add to one side and take from the other.  This
// is what keeps it "generally straight": however far off the pin is, it can
// only bend the path this much.  Lower = straighter, higher = turns harder.
constexpr int VISION_DRIVE_MAX_STEER = 25;

// When the pin is out of view, hold the last heading instead.  Power per
// degree off that heading.
constexpr double VISION_HOLD_KP = 2.0;

constexpr double VISION_DRIVE_TOL  = 0.5;  // inches from the end to call it done
constexpr double VISION_DRIVE_SLOW = 8;    // inches from the end, start slowing
constexpr int    VISION_DRIVE_MIN  = 25;   // floor power so it doesn't stall short

// The largest thing the sensor can currently see.  Check .found first.
VisionTarget vision_largest();

// Turn in place until the largest target is centred.  Returns true if it got
// there, false on timeout or if nothing was ever seen.  Safe to call from an
// auton - it stops the drive before returning either way.
bool vision_align(int timeout_ms = 2000);

// Drive `inches` in a straight-ish line while steering toward the pin the
// sensor sees.  If the pin drops out of view it holds the last heading, so it
// never wanders off.  Blocks until it has gone the distance, like
// pid_drive_set + pid_wait together - no pid_wait() afterwards.
//
//   vision_drive(24, 90);            // 24 in at power 90, steering the whole way
//   vision_drive(24, 90, 1.5);       // steer 50% harder toward the pin
//   vision_drive(24, 90, 1.0, 50);   // steer for the first 50% of the distance,
//                                    //   then drive straight for the rest
//
// sensitivity multiplies VISION_DRIVE_KP for just this call.
//
// vision_percent is how much of the distance, from the start, the sensor is
// allowed to steer.  After that it locks the heading it had at that point and
// drives straight to the end - so it lines up early, then commits.  100 steers
// the whole way.
//
// Negative inches drives backwards on heading hold only - the camera faces
// forward, so it can't see a pin behind the robot.  Returns false on timeout.
bool vision_drive(double inches, int speed, double sensitivity = 1.0,
                  double vision_percent = 100, int timeout_ms = 4000);

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
