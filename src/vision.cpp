#include "main.h"

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
