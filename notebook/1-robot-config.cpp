// ============================================================================
//  1. Robot configuration
//  Ports, devices, and the numbers we tune most often.
// ============================================================================

// ---- Drivetrain -------------------------------------------------------------
// Four motors, two per side, with the IMU on port 5. The left side is reversed
// in the port list (negative numbers) so both sides drive forward together.
// Wheels are 3.125 in, geared to 360 rpm.
ez::Drive chassis(
    {-20, -12},   // left motors
    {10, 3},      // right motors
    5,            // IMU
    3.125,        // wheel diameter, inches
    360);         // wheel rpm


// ---- Cascade lift -----------------------------------------------------------
// Two motors on one shaft. They always get opposite commands, because they
// face each other on the lift. We do that in code instead of with a negative
// port so the relationship is written down where you can see it.
constexpr int8_t L_MOTOR_A_PORT = 13;
constexpr int8_t L_MOTOR_B_PORT = 6;

pros::Motor l_motor_a(L_MOTOR_A_PORT);
pros::Motor l_motor_b(L_MOTOR_B_PORT);

// Rotation sensor on the cascade shaft, added so the cascade heights would be
// the same from run to run. The motor encoders reset every time the brain
// reboots and read through the gearbox; this reads the shaft directly.
constexpr int8_t CASCADE_ROT_PORT = 14;
pros::Rotation cascade_rot(CASCADE_ROT_PORT);


// ---- Intake -----------------------------------------------------------------
// Four motors:
//   fin_1, fin_2   the two fins at the front. fin_2 is mounted the other way
//                  round, so it always gets the opposite sign.
//   dropdown       only allowed to run when the intake is lowered (see below)
//   upper_roller   only runs at the collect height, during the macro
constexpr int8_t FIN_1_PORT        = 1;
constexpr int8_t DROPDOWN_PORT     = 4;
constexpr int8_t UPPER_ROLLER_PORT = 19;
constexpr int8_t FIN_2_PORT        = 11;

pros::Motor fin_1(FIN_1_PORT);
pros::Motor dropdown(DROPDOWN_PORT);
pros::Motor upper_roller(UPPER_ROLLER_PORT);
pros::Motor fin_2(FIN_2_PORT);


// ---- Pneumatics -------------------------------------------------------------
// Four single-acting cylinders on the brain's 3-wire ports.
constexpr char HIGH_INTAKE_PORT   = 'F';
constexpr char MIDDLE_INTAKE_PORT = 'E';
constexpr char CLAW_PORT          = 'B';
constexpr char C_FLIP_PORT        = 'A';

constexpr bool PISTON_EXTENDED  = true;
constexpr bool PISTON_RETRACTED = false;

// The second argument is the state each one powers on in.
pros::adi::DigitalOut high_intake(HIGH_INTAKE_PORT,     PISTON_EXTENDED);
pros::adi::DigitalOut middle_intake(MIDDLE_INTAKE_PORT, PISTON_EXTENDED);
pros::adi::DigitalOut claw(CLAW_PORT,                   PISTON_EXTENDED);
pros::adi::DigitalOut c_flip(C_FLIP_PORT,               PISTON_RETRACTED);

// The brain has no way to read back what a solenoid is doing, so we keep our
// own record of what we last told each one. The dropdown's safety check and
// the toggle buttons both depend on these being right, which is why the rest
// of the code only changes pistons through the helper functions in file 2.
bool high_intake_extended   = true;
bool middle_intake_extended = true;
bool claw_extended          = true;
bool c_flip_extended        = false;

// The claw and the flip piston are plumbed differently, so "on" means a
// different valve state for each. We flipped these more than once on the
// robot before they were right.
constexpr bool CLAW_ON  = true;    // gripping
constexpr bool CLAW_OFF = false;   // open
constexpr bool FLIP_ON  = false;
constexpr bool FLIP_OFF = true;


// ---- Speeds -----------------------------------------------------------------
// Motor power is -127 to 127.
constexpr int L_SPEED  = 127;                  // cascade up (L1)
constexpr int L2_SPEED = L_SPEED * 50 / 100;   // cascade down (L2), half speed
constexpr int R_SPEED  = 127;                  // intake (R1 / R2)

// At the same power the cascade moved noticeably slower going down than going
// up, so down moves in the macro get their own number.
int CASCADE_DOWN_SPEED = 127 * 80 / 100;       // 80%


// ---- Cascade heights --------------------------------------------------------
// These are rotation sensor readings. To set one, drive the cascade to the
// spot by hand, read the number off the controller screen, and put it here.
double CASCADE_LOW     = 190;    // bottom
double CASCADE_COLLECT = 290;    // where the macro stops to pick up
double CASCADE_FLIP    = 340;    // above collect; the flip piston works here
double CASCADE_OUT     = 400;    // where the second press lifts to
double CASCADE_MAX     = 1000;   // L1 won't raise past this


// ---- Macro timing -----------------------------------------------------------
// All in milliseconds. These took more trial and error than anything else on
// the robot, since the pistons have to fire at the right point in the
// cascade's travel. They're kept here so we can adjust them without touching
// the macro itself.

// First press: how far into the rise the flip piston lets go.
int CASCADE_FLIP_BACK_MS    = 300;
// First press: how long to hold at the top after that, before going down.
int CASCADE_FLIP_RELEASE_MS = 0;

// Second press: how far into the rise the flip piston fires.
int CASCADE_FLIP_DELAY_MS   = 300;
// Second press: how long after the flip before the cascade moves again.
int CASCADE_AFTER_FLIP_MS   = 0;
