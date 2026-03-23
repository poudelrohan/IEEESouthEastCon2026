// ============================================================
//  SIMPLE STRAIGHT LINE TEST
//  Walk forward 10 seconds, then backward 10 seconds.
//
//  RGB LEDs:
//    GREEN  = walking forward
//    RED    = walking backward
//    BLUE   = done
//
//  WARNING: Do NOT connect to laptop while running!
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ==========================================
//  TUNABLE PARAMETERS
// ==========================================
const float WALK_SPEED     = 1.5f;   // Forward/backward speed
const int   GAIT_MS        = 600;    // Gait period (ms per step cycle)
const int   WALK_TIME_MS   = 10000;  // 10 seconds each direction
const float BODY_HEIGHT    = 0.0f;   // Body height (0 = default)

// ==========================================
//  STATE
// ==========================================
Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t   pos = {0.0f, 0.0f, 0.0f};
Euler_t    att = {0.0f, 0.0f, 0.0f};

// RGB colors
uint8_t RGB_GREEN[3] = {0, 255, 0};
uint8_t RGB_RED[3]   = {255, 0, 0};
uint8_t RGB_BLUE[3]  = {0, 0, 255};
uint8_t RGB_OFF[3]   = {0, 0, 0};

void stop_robot() {
    vel = {0.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, BODY_HEIGHT};
    minihexa.move(&vel, &pos, &att, 600);
    delay(500);
}

void setup() {
    delay(10000);  // 10s safety — unplug USB, place on ground

    Serial.begin(115200);
    minihexa.begin();
    delay(3000);   // Stand up and stabilize

    // ---- FORWARD 10 SECONDS (GREEN) ----
    minihexa.sensor.set_ultrasound_rgb(0, RGB_GREEN, RGB_GREEN);

    vel = {0.0f, WALK_SPEED, 0.0f};
    pos = {0.0f, 0.0f, BODY_HEIGHT};
    minihexa.move(&vel, &pos, &att, GAIT_MS, -1);  // -1 = continuous
    delay(WALK_TIME_MS);
    stop_robot();

    delay(2000);  // 2s pause between directions

    // ---- BACKWARD 10 SECONDS (RED) ----
    minihexa.sensor.set_ultrasound_rgb(0, RGB_RED, RGB_RED);

    vel = {0.0f, -WALK_SPEED, 0.0f};
    pos = {0.0f, 0.0f, BODY_HEIGHT};
    minihexa.move(&vel, &pos, &att, GAIT_MS, -1);  // -1 = continuous
    delay(WALK_TIME_MS);
    stop_robot();

    // ---- DONE (BLUE) ----
    minihexa.sensor.set_ultrasound_rgb(0, RGB_BLUE, RGB_BLUE);
}

void loop() {
    // Done — robot is idle
}
