// square_walk.ino
// IEEE SoutheastCon 2026 - MiniHexa Competition Code
//
// First sketch: walk a square pattern to calibrate movement constants.
// Fully autonomous — no BLE/WiFi needed.
//
// How to calibrate:
//   1. Upload, open Serial Monitor at 115200 baud
//   2. Watch the robot — does it make a square?
//   3. Adjust TURN_STEPS until turns are exactly 90 degrees
//   4. Adjust STEPS_PER_SIDE for desired side length

#include "hiwonder_robot.h"

// ============================================================
//  TUNABLE PARAMETERS — calibrate these on the real robot
// ============================================================

// Forward movement
const int   STEPS_PER_SIDE  = 5;      // gait cycles per side of the square
const int   GAIT_TIME_MS    = 600;    // ms per gait cycle (forward walking)
const float FORWARD_SPEED   = 3.0f;   // vy value for forward movement

// Turning
const int   TURN_STEPS      = 4;      // gait cycles for a 90-degree turn (CALIBRATE THIS)
const int   TURN_TIME_MS    = 1000;   // ms per gait cycle (turning)
const float TURN_OMEGA      = -1.8f;  // rotation speed, negative = clockwise (right turn)

// ============================================================
//  GLOBALS
// ============================================================

Robot minihexa;

Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t   pos = {0.0f, 0.0f, 0.0f};
Euler_t    att = {0.0f, 0.0f, 0.0f};

// ============================================================
//  SETUP — runs the square walk once
// ============================================================

void setup() {
    Serial.begin(115200);
    minihexa.begin();
    delay(2000);  // let servos settle after standing up

    Serial.println("=== Square Walk Starting ===");
    Serial.printf("Config: %d steps/side, %d turn steps, omega=%.1f\n",
                  STEPS_PER_SIDE, TURN_STEPS, TURN_OMEGA);

    // Walk a square: 4 sides with right turns
    for (int side = 0; side < 4; side++) {

        // --- Walk forward ---
        Serial.printf("Side %d/4: walking forward %d steps...\n", side + 1, STEPS_PER_SIDE);
        vel = {0.0f, FORWARD_SPEED, 0.0f};
        minihexa.move(&vel, &pos, &att, GAIT_TIME_MS, STEPS_PER_SIDE);
        delay(STEPS_PER_SIDE * GAIT_TIME_MS + 200);

        // Brief pause to stabilize
        vel = {0.0f, 0.0f, 0.0f};
        minihexa.move(&vel, &pos, &att);
        delay(500);

        // --- Turn right 90 degrees ---
        Serial.printf("Side %d/4: turning right...\n", side + 1);
        vel = {0.0f, 0.0f, TURN_OMEGA};
        minihexa.move(&vel, &pos, &att, TURN_TIME_MS, TURN_STEPS);
        delay(TURN_STEPS * TURN_TIME_MS + 200);

        // Brief pause to stabilize
        vel = {0.0f, 0.0f, 0.0f};
        minihexa.move(&vel, &pos, &att);
        delay(500);
    }

    Serial.println("=== Square Complete! ===");
}

// ============================================================
//  LOOP — nothing to do after the square
// ============================================================

void loop() {
}
