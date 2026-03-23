// course_run.ino
// IEEE SoutheastCon 2026 - Phase 1 Course Navigation
//
// Route: Start → forward to Antenna #1 → right turn → forward into crater →
//        orbit antenna #3 → exit crater → go to wall → left turn → back to start
//
// ALL distances, speeds, and timing are tunable variables at the top.
// Adjust them after testing on the real board.

#include "hiwonder_robot.h"

// ============================================================
//  TUNABLE PARAMETERS — CHANGE THESE TO CALIBRATE
// ============================================================

// --- General ---
const int   STARTUP_DELAY_MS     = 10000;  // delay after upload before robot moves
const int   PAUSE_BETWEEN_MS     = 1500;   // pause between steps (ms)

// --- Forward walking ---
const float WALK_SPEED           = 2.0f;   // vy for normal forward walking
const float WALK_SPEED_SLOW      = 1.2f;   // vy for slow/careful walking (crater)
const int   WALK_GAIT_MS         = 600;    // gait cycle period for walking (ms)

// --- Turning ---
const int   TURN_90_STEPS        = 4;      // gait cycles for 90-degree turn (CALIBRATE!)
const int   TURN_180_STEPS       = 8;      // gait cycles for 180-degree turn (2x 90)
const int   TURN_GAIT_MS         = 1000;   // gait cycle period for turning (ms)
const float TURN_OMEGA_RIGHT     = -1.8f;  // CW turn speed (negative = right)
const float TURN_OMEGA_LEFT      = 1.8f;   // CCW turn speed (positive = left)

// --- Step 1: Walk forward to Antenna #1 ---
const int   ANTENNA1_STOP_DIST   = 150;    // stop this far from antenna #1 (mm)
const int   WALK_TIMEOUT_MS      = 20000;  // max time to walk before giving up (ms)

// --- Step 3: Walk forward toward crater ---
const int   CRATER_STOP_DIST     = 200;    // stop when ultrasonic reads this (mm) — crater edge
const int   CRATER_APPROACH_TIMEOUT = 15000;

// --- Step 4: Descend into crater ---
const int   ORBIT_TARGET_DIST    = 150;    // target distance from antenna during orbit (mm)
const int   ORBIT_CLOSE_DIST     = 120;    // too close — turn away
const int   ORBIT_FAR_DIST       = 180;    // too far — turn toward
const int   DESCENT_STEPS        = 4;      // gait cycles to walk into crater

// --- Step 5: Orbit around antenna ---
const float ORBIT_WALK_SPEED     = 1.5f;   // forward speed during orbit
const int   ORBIT_GAIT_MS        = 800;    // gait cycle during orbit
const float ORBIT_YAW_ADJUST     = 0.3f;   // yaw correction per reading (degrees)
const float ORBIT_MAX_YAW        = 15.0f;  // max yaw during orbit (clamped)
const unsigned long ORBIT_DURATION_MS = 25000; // max orbit time (one lap ~20-25 sec)
const int   ORBIT_POLL_MS        = 100;    // sensor poll interval during orbit

// --- Step 7: Exit crater ---
const float CRATER_EXIT_HEIGHT   = 2.5f;   // raise body z (cm) for crater exit
const int   CRATER_EXIT_STEPS    = 6;      // gait cycles to walk out of crater

// --- Step 8: Walk to west wall ---
const int   WALL_STOP_DIST       = 150;    // stop this far from wall (mm)
const int   WALL_APPROACH_TIMEOUT = 15000;

// --- Step 10: Walk back to start ---
const int   START_WALL_STOP_DIST = 120;    // stop this far from south wall (mm)
const int   RETURN_TIMEOUT_MS    = 20000;

// ============================================================
//  GLOBALS
// ============================================================

Robot minihexa;

Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t   pos = {0.0f, 0.0f, 0.0f};
Euler_t    att = {0.0f, 0.0f, 0.0f};

uint8_t step = 0;
bool course_done = false;

// ============================================================
//  HELPERS
// ============================================================

void stop_robot() {
    vel = {0.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, 0.0f};
    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att);
    delay(500);
}

void pause() {
    stop_robot();
    delay(PAUSE_BETWEEN_MS);
}

void log_step(uint8_t num, const char* description) {
    Serial.printf("\n>>> Step %d: %s\n", num, description);
}

// Walk forward continuously until ultrasonic reads <= stop_dist, or timeout
// Returns true if obstacle detected, false if timed out
bool walk_until_obstacle(float speed, int stop_dist, unsigned long timeout) {
    vel = {0.0f, speed, 0.0f};
    pos = {0.0f, 0.0f, 0.0f};
    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, WALK_GAIT_MS);

    unsigned long start = millis();
    while (millis() - start < timeout) {
        uint16_t dis = minihexa.sensor.get_distance();
        Serial.printf("  Walking... dist=%d mm (stop at %d)\n", dis, stop_dist);
        if (dis > 0 && dis <= stop_dist) {
            Serial.printf("  Obstacle detected at %d mm. Stopping.\n", dis);
            stop_robot();
            return true;
        }
        delay(100);
    }
    Serial.println("  Timeout reached. Stopping.");
    stop_robot();
    return false;
}

// Turn right by N steps (blocking)
void turn_right(int steps) {
    vel = {0.0f, 0.0f, TURN_OMEGA_RIGHT};
    pos = {0.0f, 0.0f, 0.0f};
    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, TURN_GAIT_MS, steps);
    delay(steps * TURN_GAIT_MS + 200);
    stop_robot();
}

// Turn left by N steps (blocking)
void turn_left(int steps) {
    vel = {0.0f, 0.0f, TURN_OMEGA_LEFT};
    pos = {0.0f, 0.0f, 0.0f};
    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, TURN_GAIT_MS, steps);
    delay(steps * TURN_GAIT_MS + 200);
    stop_robot();
}

// Walk forward N steps (blocking)
void walk_forward(int steps, float speed, int gait_ms) {
    vel = {0.0f, speed, 0.0f};
    pos = {0.0f, 0.0f, 0.0f};
    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, gait_ms, steps);
    delay(steps * gait_ms + 200);
    stop_robot();
}

// Walk forward with raised body height (for crater exit)
void walk_forward_raised(int steps, float speed, int gait_ms, float height) {
    vel = {0.0f, speed, 0.0f};
    pos = {0.0f, 0.0f, height};
    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, gait_ms, steps);
    delay(steps * gait_ms + 200);
    stop_robot();
}

// ============================================================
//  ORBIT: Walk around antenna maintaining ultrasonic distance
// ============================================================

void orbit_antenna() {
    Serial.println("  Starting orbit. Maintaining distance from antenna...");
    Serial.printf("  Target: %d mm, Close: %d mm, Far: %d mm\n",
                  ORBIT_TARGET_DIST, ORBIT_CLOSE_DIST, ORBIT_FAR_DIST);

    float current_yaw = 0.0f;
    unsigned long start = millis();

    // Start walking forward
    vel = {0.0f, ORBIT_WALK_SPEED, 0.0f};
    pos = {0.0f, 0.0f, 0.0f};
    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, ORBIT_GAIT_MS);

    while (millis() - start < ORBIT_DURATION_MS) {
        uint16_t dis = minihexa.sensor.get_distance();

        if (dis > 0 && dis < ORBIT_CLOSE_DIST) {
            // Too close to antenna — yaw away (turn left/outward)
            current_yaw += ORBIT_YAW_ADJUST;
        } else if (dis > ORBIT_FAR_DIST) {
            // Too far from antenna — yaw toward (turn right/inward)
            current_yaw -= ORBIT_YAW_ADJUST;
        }
        // else: in the sweet spot, keep current yaw

        // Clamp yaw
        if (current_yaw > ORBIT_MAX_YAW) current_yaw = ORBIT_MAX_YAW;
        if (current_yaw < -ORBIT_MAX_YAW) current_yaw = -ORBIT_MAX_YAW;

        // Apply yaw correction while continuing to walk forward
        vel = {0.0f, ORBIT_WALK_SPEED, 0.0f};
        att = {0.0f, 0.0f, current_yaw};
        minihexa.move(&vel, &pos, &att, ORBIT_GAIT_MS);

        Serial.printf("  Orbit: dist=%d mm, yaw=%.1f deg, elapsed=%lu ms\n",
                      dis, current_yaw, millis() - start);

        delay(ORBIT_POLL_MS);
    }

    Serial.println("  Orbit complete (duration reached).");
    stop_robot();
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
    Serial.begin(115200);

    // Give time after upload before robot starts moving
    Serial.println("Waiting before startup...");
    delay(STARTUP_DELAY_MS);

    minihexa.begin();
    delay(2000);

    Serial.println("============================================");
    Serial.println("  SoutheastCon 2026 - Course Run");
    Serial.println("============================================");
    Serial.printf("  Walk speed: %.1f, Slow: %.1f\n", WALK_SPEED, WALK_SPEED_SLOW);
    Serial.printf("  Turn 90: %d steps, Turn 180: %d steps\n", TURN_90_STEPS, TURN_180_STEPS);
    Serial.printf("  Antenna stop: %d mm, Crater stop: %d mm\n", ANTENNA1_STOP_DIST, CRATER_STOP_DIST);
    Serial.printf("  Orbit target: %d mm, duration: %lu ms\n", ORBIT_TARGET_DIST, ORBIT_DURATION_MS);
    Serial.println("============================================\n");
}

// ============================================================
//  LOOP — state machine for course navigation
// ============================================================

void loop() {
    if (course_done) return;

    switch (step) {

        // --------------------------------------------------------
        //  STEP 0: Leave starting area — walk forward toward Antenna #1
        // --------------------------------------------------------
        case 0:
            log_step(0, "Walk FORWARD toward Antenna #1 (ultrasonic stop)");
            walk_until_obstacle(WALK_SPEED, ANTENNA1_STOP_DIST, WALK_TIMEOUT_MS);
            pause();
            step++;
            break;

        // --------------------------------------------------------
        //  STEP 1: Turn RIGHT 90 degrees (now facing east toward crater)
        // --------------------------------------------------------
        case 1:
            log_step(1, "Turn RIGHT 90 degrees");
            turn_right(TURN_90_STEPS);
            pause();
            step++;
            break;

        // --------------------------------------------------------
        //  STEP 2: Walk toward crater (ultrasonic detects antenna #3)
        // --------------------------------------------------------
        case 2:
            log_step(2, "Walk FORWARD toward crater (ultrasonic stop)");
            walk_until_obstacle(WALK_SPEED, CRATER_STOP_DIST, CRATER_APPROACH_TIMEOUT);
            pause();
            step++;
            break;

        // --------------------------------------------------------
        //  STEP 3: Descend into crater slowly
        // --------------------------------------------------------
        case 3:
            log_step(3, "Descend into crater (slow walk forward)");
            walk_forward(DESCENT_STEPS, WALK_SPEED_SLOW, WALK_GAIT_MS);
            pause();
            step++;
            break;

        // --------------------------------------------------------
        //  STEP 4: Orbit antenna #3 — one full lap
        // --------------------------------------------------------
        case 4:
            log_step(4, "Orbit Antenna #3 (ultrasonic distance hold)");
            orbit_antenna();
            pause();
            step++;
            break;

        // --------------------------------------------------------
        //  STEP 5: After orbit — check facing direction
        //  If facing antenna (close reading), do 180. Otherwise skip.
        // --------------------------------------------------------
        case 5: {
            log_step(5, "Check direction after orbit");
            uint16_t dis = minihexa.sensor.get_distance();
            Serial.printf("  Post-orbit distance: %d mm\n", dis);

            if (dis > 0 && dis < ORBIT_FAR_DIST) {
                // Facing antenna — turn 180 to face outward
                Serial.println("  Facing antenna. Turning 180 degrees.");
                turn_right(TURN_180_STEPS);
            } else {
                Serial.println("  Already facing outward. No turn needed.");
            }
            pause();
            step++;
            break;
        }

        // --------------------------------------------------------
        //  STEP 6: Raise body and walk OUT of crater
        // --------------------------------------------------------
        case 6:
            log_step(6, "Raise body and walk OUT of crater");
            Serial.printf("  Raising body to z=%.1f cm\n", CRATER_EXIT_HEIGHT);
            walk_forward_raised(CRATER_EXIT_STEPS, WALK_SPEED_SLOW, WALK_GAIT_MS, CRATER_EXIT_HEIGHT);

            // Lower body back to normal
            pos = {0.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(800);
            pause();
            step++;
            break;

        // --------------------------------------------------------
        //  STEP 7: Walk toward WEST wall (ultrasonic stop)
        // --------------------------------------------------------
        case 7:
            log_step(7, "Walk toward WEST wall (ultrasonic stop)");
            walk_until_obstacle(WALK_SPEED, WALL_STOP_DIST, WALL_APPROACH_TIMEOUT);
            pause();
            step++;
            break;

        // --------------------------------------------------------
        //  STEP 8: Turn LEFT 90 degrees (now facing south toward start)
        // --------------------------------------------------------
        case 8:
            log_step(8, "Turn LEFT 90 degrees (face south toward start)");
            turn_left(TURN_90_STEPS);
            pause();
            step++;
            break;

        // --------------------------------------------------------
        //  STEP 9: Walk back to starting area (ultrasonic stop at wall)
        // --------------------------------------------------------
        case 9:
            log_step(9, "Walk SOUTH back to starting area (ultrasonic stop)");
            walk_until_obstacle(WALK_SPEED, START_WALL_STOP_DIST, RETURN_TIMEOUT_MS);
            pause();
            step++;
            break;

        // --------------------------------------------------------
        //  STEP 10: Done!
        // --------------------------------------------------------
        case 10:
            Serial.println("\n============================================");
            Serial.println("  COURSE RUN COMPLETE!");
            Serial.println("  Robot should be back in starting area.");
            Serial.println("============================================");
            Serial.printf("  Battery: %d mV\n", minihexa.board.bat_voltage);
            minihexa.reset();
            course_done = true;
            break;
    }
}
