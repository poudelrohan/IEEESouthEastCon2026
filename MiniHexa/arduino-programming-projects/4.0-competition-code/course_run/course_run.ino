// ============================================================

// CRATER LAP TEST PROGRAM — 5 orbit strategies

//

// The crater is ~1 foot radius (30cm), sloped walls,

// antenna in the center. Robot must lap 360 degrees inside.

//

// Set TEST_TO_RUN to pick which strategy to test:

// 0 = Run ALL tests in sequence (5s pause between each)

// 1 = Dead-reckoning square

// 2 = Dead-reckoning hexagon

// 3 = Crab-walk circle (strafe + rotate simultaneously)

// 4 = Face-antenna ultrasonic orbit (most robust)

// 5 = IMU-tracked hexagon (smart dead-reckoning)

// ============================================================



#include "hiwonder_robot.h"



Robot minihexa;



// ==========================================

// WHICH TEST TO RUN (0=ALL, 1-5=specific)

// ==========================================

const int TEST_TO_RUN = 3;



// ==========================================

// SHARED PARAMETERS

// ==========================================

const int GAIT_MS = 600; // Base gait period (ms per step cycle)

const float WALK_SPEED = 2.0f; // Forward walking speed

const float STRAFE_SPEED = 2.0f; // Sideways walking speed

const int PAUSE_BETWEEN = 5000; // Pause between tests (ms)



// ==========================================

// TEST 1: DEAD-RECKONING SQUARE

// Walk 4 sides, turn 90 left between each

// Circumference = 4 * side ≈ 188cm → side ≈ 47cm

// ==========================================

const int SQ_SIDE_STEPS = 2; // Steps per side (tune for ~47cm)

const int SQ_TURN_STEPS = 4; // Steps to turn 90 degrees

const float SQ_TURN_SPEED = 1.8f; // Turn omega for 90 degrees



// ==========================================

// TEST 2: DEAD-RECKONING HEXAGON

// Walk 6 sides, turn 60 left between each

// Side ≈ radius ≈ 30cm

// ==========================================

const int HEX_SIDE_STEPS = 3; // Steps per side (tune for ~30cm)

const int HEX_TURN_STEPS = 3; // Steps to turn 60 degrees

const float HEX_TURN_SPEED = 1.5f; // Turn omega for 60 degrees



// ==========================================

// TEST 3: CRAB-WALK CIRCLE

// Strafe left + rotate CCW simultaneously

// The vx/omega ratio sets the orbit radius

// ==========================================

const float CRAB_VX = -1.5f; // Strafe left (negative = left)

const float CRAB_OMEGA = 0.8f; // Rotate CCW (positive = left)

const int CRAB_GAIT_MS = 700; // Gait period for crab walk

const int CRAB_TOTAL_MS = 15000; // Total duration to complete 360



// ==========================================

// TEST 4: FACE-ANTENNA ULTRASONIC ORBIT

// Face the antenna, strafe left, maintain distance

// with ultrasonic feedback. Track yaw via IMU.

// ==========================================

const int US_TARGET_DIST = 250; // Target distance to antenna (mm)

const int US_TOLERANCE = 50; // Dead band (mm) — no correction needed

const float US_STRAFE_SPEED = -1.5f; // Strafe left speed

const float US_CORRECT_SPEED = 0.5f; // Forward/backward correction speed

const int US_GAIT_MS = 600; // Gait period

const float US_YAW_TARGET = 360.0f;// Stop after this much total rotation



// ==========================================

// TEST 5: IMU-TRACKED HEXAGON

// Like test 2 but verifies each turn with IMU

// ==========================================

const int IMU_SIDE_STEPS = 3; // Steps per side

const float IMU_TURN_TARGET = 60.0f; // Degrees per turn

const float IMU_TURN_TOLERANCE = 5.0f; // Acceptable error (degrees)

const float IMU_TURN_SPEED = 1.2f; // Slower turn for accuracy

const int IMU_TURN_GAIT_MS = 800; // Gait period during IMU turns

const int IMU_MAX_TURN_MS = 5000; // Safety timeout per turn



// ==========================================

// MOTION STRUCTS

// ==========================================

Velocity_t vel = {0.0f, 0.0f, 0.0f};

Vector_t pos = {0.0f, 0.0f, 0.0f};

Euler_t att = {0.0f, 0.0f, 0.0f};



// ==========================================

// HELPER FUNCTIONS

// ==========================================



void stop_robot() {

vel = {0.0f, 0.0f, 0.0f};

minihexa.move(&vel, &pos, &att, 600);

delay(1000);

}



void walk_forward(int steps) {

vel = {0.0f, WALK_SPEED, 0.0f};

minihexa.move(&vel, &pos, &att, GAIT_MS, steps);

delay((steps * GAIT_MS) + 500);

stop_robot();

}



void turn_left(float omega, int steps, int gait_ms) {

vel = {0.0f, 0.0f, omega};

minihexa.move(&vel, &pos, &att, gait_ms, steps);

delay((steps * gait_ms) + 500);

stop_robot();

}



void strafe_left(float speed, int steps) {

vel = {-speed, 0.0f, 0.0f};

minihexa.move(&vel, &pos, &att, GAIT_MS, steps);

delay((steps * GAIT_MS) + 500);

stop_robot();

}



float read_yaw() {

float euler[3];

minihexa.board.get_imu_euler(euler);

return euler[2]; // yaw in degrees

}



// Normalize angle difference to -180..+180

float angle_diff(float current, float start) {

float diff = current - start;

while (diff > 180.0f) diff -= 360.0f;

while (diff < -180.0f) diff += 360.0f;

return diff;

}



// ==========================================

// TEST 1: DEAD-RECKONING SQUARE

// ==========================================

void test_square() {

Serial.println("=== TEST 1: Dead-Reckoning Square ===");



for (int side = 0; side < 4; side++) {

Serial.printf(" Side %d: walking %d steps\n", side + 1, SQ_SIDE_STEPS);

walk_forward(SQ_SIDE_STEPS);



Serial.printf(" Turning 90 left (%d steps, omega=%.1f)\n", SQ_TURN_STEPS, SQ_TURN_SPEED);

turn_left(SQ_TURN_SPEED, SQ_TURN_STEPS, 1000);

}



Serial.println("=== TEST 1 COMPLETE ===");

}



// ==========================================

// TEST 2: DEAD-RECKONING HEXAGON

// ==========================================

void test_hexagon() {

Serial.println("=== TEST 2: Dead-Reckoning Hexagon ===");



for (int side = 0; side < 6; side++) {

Serial.printf(" Side %d: walking %d steps\n", side + 1, HEX_SIDE_STEPS);

walk_forward(HEX_SIDE_STEPS);



Serial.printf(" Turning 60 left (%d steps, omega=%.1f)\n", HEX_TURN_STEPS, HEX_TURN_SPEED);

turn_left(HEX_TURN_SPEED, HEX_TURN_STEPS, 1000);

}



Serial.println("=== TEST 2 COMPLETE ===");

}



// ==========================================

// TEST 3: CRAB-WALK CIRCLE

// Strafe left + rotate CCW simultaneously

// ==========================================

void test_crab_circle() {

Serial.println("=== TEST 3: Crab-Walk Circle ===");

Serial.printf(" vx=%.1f, omega=%.1f, duration=%dms\n", CRAB_VX, CRAB_OMEGA, CRAB_TOTAL_MS);



// Start continuous crab-walk motion

vel = {CRAB_VX, 0.0f, CRAB_OMEGA};

minihexa.move(&vel, &pos, &att, CRAB_GAIT_MS, -1); // -1 = continuous



// Let it run for the specified duration

delay(CRAB_TOTAL_MS);



// Stop

stop_robot();

Serial.println("=== TEST 3 COMPLETE ===");

}



// ==========================================

// TEST 4: FACE-ANTENNA ULTRASONIC ORBIT

// Face antenna, strafe left, maintain distance

// with ultrasonic. Track yaw for 360 completion.

// ==========================================

void test_ultrasonic_orbit() {

Serial.println("=== TEST 4: Face-Antenna Ultrasonic Orbit ===");

Serial.printf(" Target dist=%dmm, tolerance=%dmm\n", US_TARGET_DIST, US_TOLERANCE);



float start_yaw = read_yaw();

float accumulated_rotation = 0.0f;

float prev_yaw = start_yaw;

unsigned long start_time = millis();

unsigned long max_time = 60000; // 60s safety timeout



Serial.printf(" Start yaw=%.1f\n", start_yaw);



while (accumulated_rotation < US_YAW_TARGET) {

// Safety timeout

if (millis() - start_time > max_time) {

Serial.println(" TIMEOUT — stopping");

break;

}



// Read sensors

uint16_t dist = minihexa.sensor.get_distance();

float current_yaw = read_yaw();



// Track accumulated rotation (CCW = positive yaw change)

float delta_yaw = angle_diff(current_yaw, prev_yaw);

// We're going CCW (left), so positive delta = progress

if (delta_yaw > 0) {

accumulated_rotation += delta_yaw;

}

// Small negative deltas are noise, large negative = went wrong way

else if (delta_yaw < -10.0f) {

// Went the wrong way significantly — still count it as absolute

accumulated_rotation += fabsf(delta_yaw);

}

prev_yaw = current_yaw;



// Compute correction

float vy_correct = 0.0f;

if (dist < US_TARGET_DIST - US_TOLERANCE) {

vy_correct = -US_CORRECT_SPEED; // Too close — back away

} else if (dist > US_TARGET_DIST + US_TOLERANCE) {

vy_correct = US_CORRECT_SPEED; // Too far — move closer

}



// Apply motion: strafe left + distance correction

vel = {US_STRAFE_SPEED, vy_correct, 0.0f};

minihexa.move(&vel, &pos, &att, US_GAIT_MS, 1); // 1 step at a time

delay(US_GAIT_MS + 200);



Serial.printf(" dist=%dmm yaw=%.1f accum=%.1f vy_corr=%.1f\n",

dist, current_yaw, accumulated_rotation, vy_correct);

}



stop_robot();

Serial.printf(" Total rotation: %.1f degrees\n", accumulated_rotation);

Serial.println("=== TEST 4 COMPLETE ===");

}



// ==========================================

// TEST 5: IMU-TRACKED HEXAGON

// Hexagon with IMU-verified 60-degree turns

// ==========================================

void imu_turn_left(float target_degrees) {

float start_yaw = read_yaw();

float accumulated = 0.0f;

float prev_yaw = start_yaw;

unsigned long start_time = millis();



Serial.printf(" IMU turn: target=%.0f, start_yaw=%.1f\n", target_degrees, start_yaw);



// Start turning

vel = {0.0f, 0.0f, IMU_TURN_SPEED};

minihexa.move(&vel, &pos, &att, IMU_TURN_GAIT_MS, -1); // continuous



while (accumulated < target_degrees - IMU_TURN_TOLERANCE) {

// Safety timeout

if (millis() - start_time > IMU_MAX_TURN_MS) {

Serial.printf(" Turn timeout at %.1f degrees\n", accumulated);

break;

}



delay(50); // Small polling interval



float current_yaw = read_yaw();

float delta = angle_diff(current_yaw, prev_yaw);

if (delta > 0) {

accumulated += delta;

}

prev_yaw = current_yaw;

}



stop_robot();

Serial.printf(" Turn result: %.1f degrees (target: %.0f)\n", accumulated, target_degrees);

}



void test_imu_hexagon() {

Serial.println("=== TEST 5: IMU-Tracked Hexagon ===");



for (int side = 0; side < 6; side++) {

Serial.printf(" Side %d: walking %d steps\n", side + 1, IMU_SIDE_STEPS);

walk_forward(IMU_SIDE_STEPS);



Serial.printf(" IMU-guided 60-degree left turn\n");

imu_turn_left(IMU_TURN_TARGET);

}



Serial.println("=== TEST 5 COMPLETE ===");

}



// ==========================================

// SETUP — runs selected test(s)

// ==========================================

void setup() {

delay(10000); // 10s safety delay — disconnect USB, put on ground



Serial.begin(115200);

minihexa.begin();



// Enable IMU for yaw tracking

minihexa.board.imu_update(true);



delay(3000); // Let robot stand up and IMU stabilize

Serial.println("Crater Lap Test Program starting...");

Serial.printf("TEST_TO_RUN = %d\n", TEST_TO_RUN);



// --- Test 1: Dead-Reckoning Square ---

if (TEST_TO_RUN == 0 || TEST_TO_RUN == 1) {

test_square();

if (TEST_TO_RUN == 0) delay(PAUSE_BETWEEN);

}



// --- Test 2: Dead-Reckoning Hexagon ---

if (TEST_TO_RUN == 0 || TEST_TO_RUN == 2) {

test_hexagon();

if (TEST_TO_RUN == 0) delay(PAUSE_BETWEEN);

}



// --- Test 3: Crab-Walk Circle ---

if (TEST_TO_RUN == 0 || TEST_TO_RUN == 3) {

test_crab_circle();

if (TEST_TO_RUN == 0) delay(PAUSE_BETWEEN);

}



// --- Test 4: Face-Antenna Ultrasonic Orbit ---

if (TEST_TO_RUN == 0 || TEST_TO_RUN == 4) {

test_ultrasonic_orbit();

if (TEST_TO_RUN == 0) delay(PAUSE_BETWEEN);

}



// --- Test 5: IMU-Tracked Hexagon ---

if (TEST_TO_RUN == 0 || TEST_TO_RUN == 5) {

test_imu_hexagon();

}



Serial.println("All selected tests complete!");

}



void loop() {

// Nothing — one-shot test program

}