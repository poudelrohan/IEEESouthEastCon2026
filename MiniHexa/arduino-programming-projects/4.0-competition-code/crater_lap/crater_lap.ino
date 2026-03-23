// ============================================================
//  SOUTHEASTCON 2026 - FULL MISSION SEQUENCE
//  Includes Height Adjustments and Square-Orbit Crater Navigation.
//
//  Phase 1: Robot faces north at start
//  Phase 2: Strafe left slowly
//  Phase 3: Walk forward to crater at max height (dead reckoning)
//  Phase 4: Drop low, square orbit inside crater (hard steps)
//  Phase 5: Rise high, face south, walk to wall, strafe to finish
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ==========================================
//  TUNABLE VARIABLES
// ==========================================

// --- General Speeds & Timings ---
const int   SLOW_GAIT_MS     = 800;   // Careful, slow walking
const int   HEAVY_GAIT_MS    = 1200;  // Very slow, deliberate steps for crater
const float SLOW_SPEED       = 1.5f;  // Forward speed
const float STRAFE_SPEED     = 1.5f;  // Left/Right speed

// --- Turn Parameters ---
const int   TURN_90_STEPS    = 4;     // Steps needed for 90 degrees
const float TURN_SPEED       = 1.8f;  // Turn omega

// --- Height Adjustments (Z-axis in cm, range is roughly -4 to +4) ---
const float MAX_HEIGHT       = 3.0f;  // Highest — for walking over crater slope
const float MIN_HEIGHT       = -1.5f; // Lowest — for stability inside crater
const float NORMAL_HEIGHT    = 0.0f;  // Default standing

// --- Phase 2: Initial Left Strafe ---
const int   PHASE2_LEFT_STEPS = 9;    // How far left to strafe at start

// --- Phase 3: Forward Approach to Crater ---
const int   PHASE3_FWD_STEPS  = 24;   // Steps forward to reach crater (dead reckoning)

// --- Phase 4: Crater Square Orbit ---
const int   CRATER_HALF_SIDE  = 5;    // Steps for the entry/exit half-side
const int   CRATER_FULL_SIDE  = 9;    // Steps for a full side of the square

// --- Phase 5: Return ---
const int   STOP_DIST_MM      = 70;   // Stop 3cm from border wall (ultrasonic)
const int   PHASE5_STRAFE_STEPS = 15; // Final strafe steps
const bool  PHASE5_STRAFE_LEFT = true; // true=strafe left, false=strafe right (tune during testing)

// ==========================================
//  STATE STRUCTS
// ==========================================
Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t   pos = {0.0f, 0.0f, 0.0f};
Euler_t    att = {0.0f, 0.0f, 0.0f};

float current_height = NORMAL_HEIGHT;

// ==========================================
//  HELPER FUNCTIONS
// ==========================================

void stop_robot() {
    vel = {0.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, 600);
    delay(500);
}

void robot_pause() {
    stop_robot();
    delay(1500);
}

void set_height(float target_z) {
    Serial.printf("Height -> %.1f cm\n", target_z);
    current_height = target_z;
    vel = {0.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, 1000);
    delay(1200);
}

void walk_forward(int steps, float speed, int gait_ms) {
    vel = {0.0f, speed, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, gait_ms, steps);
    delay((steps * gait_ms) + 500);
    stop_robot();
}

void walk_strafe(int steps, float speed, int gait_ms, bool left) {
    // left=true: negative vx (strafe left), left=false: positive vx (strafe right)
    float vx = left ? -speed : speed;
    vel = {vx, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, gait_ms, steps);
    delay((steps * gait_ms) + 500);
    stop_robot();
}

void turn_left_90() {
    vel = {0.0f, 0.0f, TURN_SPEED};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, 1000, TURN_90_STEPS);
    delay((TURN_90_STEPS * 1000) + 500);
    stop_robot();
}

void turn_right_90() {
    vel = {0.0f, 0.0f, -TURN_SPEED};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, 1000, TURN_90_STEPS);
    delay((TURN_90_STEPS * 1000) + 500);
    stop_robot();
}

// Walk forward step-by-step, stop when ultrasonic detects wall
// Returns true if stopped early due to obstacle
bool walk_until_wall(int max_steps, float speed, int gait_ms, int stop_dist_mm) {
    Serial.printf("Walking to wall (stop at %dmm)...\n", stop_dist_mm);

    for (int i = 0; i < max_steps; i++) {
        uint16_t dist = minihexa.sensor.get_distance();
        Serial.printf("  Step %d - Dist: %d mm\n", i + 1, dist);

        if (dist > 0 && dist <= stop_dist_mm) {
            Serial.println("  Wall detected! Stopping.");
            stop_robot();
            return true;
        }

        vel = {0.0f, speed, 0.0f};
        pos = {0.0f, 0.0f, current_height};
        minihexa.move(&vel, &pos, &att, gait_ms, 1);
        delay(gait_ms + 100);
    }

    stop_robot();
    return false;
}

// ==========================================
//  MAIN MISSION
// ==========================================

void setup() {
    delay(10000);  // 10s safety — disconnect USB, put on ground

    Serial.begin(115200);
    minihexa.begin();

    delay(3000);  // Stand up and stabilize
    Serial.println("=== MISSION START ===");

    // -----------------------------------------------
    //  PHASE 1: Robot is facing NORTH at start
    // -----------------------------------------------
    Serial.println("\n[Phase 1] Facing North. Ready.");

    // -----------------------------------------------
    //  PHASE 2: Strafe LEFT slowly
    // -----------------------------------------------
    Serial.println("\n[Phase 2] Strafing left...");
    walk_strafe(PHASE2_LEFT_STEPS, STRAFE_SPEED, SLOW_GAIT_MS, true);
    robot_pause();

    // -----------------------------------------------
    //  PHASE 3: Walk FORWARD to crater at MAX height
    //  Dead reckoning — no ultrasonic checking
    // -----------------------------------------------
    Serial.println("\n[Phase 3] Forward to crater (max height, dead reckoning)...");
    set_height(MAX_HEIGHT);
    walk_forward(PHASE3_FWD_STEPS, SLOW_SPEED, SLOW_GAIT_MS);
    robot_pause();

    // -----------------------------------------------
    //  PHASE 4: CRATER SQUARE ORBIT
    //  Drop to min height, heavy deliberate steps
    //
    //  Path (facing north at entry):
    //    Right 90 → 2 steps (east)
    //    Left 90  → 4 steps (north)
    //    Left 90  → 4 steps (west)
    //    Left 90  → 4 steps (south)
    //    Left 90  → 2 steps (east) → back to start
    //  Robot ends facing EAST
    // -----------------------------------------------
    Serial.println("\n[Phase 4] Crater orbit (max height, hard steps)...");
    // Stay at MAX_HEIGHT — no lowering inside the crater

    // Entry half-side: turn right, walk east
    turn_right_90();
    robot_pause();
    Serial.printf("  Half-side 1: %d steps east\n", CRATER_HALF_SIDE);
    walk_forward(CRATER_HALF_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Full side: turn left, walk north
    turn_left_90();
    robot_pause();
    Serial.printf("  Full side 2: %d steps north\n", CRATER_FULL_SIDE);
    walk_forward(CRATER_FULL_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Full side: turn left, walk west
    turn_left_90();
    robot_pause();
    Serial.printf("  Full side 3: %d steps west\n", CRATER_FULL_SIDE);
    walk_forward(CRATER_FULL_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Full side: turn left, walk south
    turn_left_90();
    robot_pause();
    Serial.printf("  Full side 4: %d steps south\n", CRATER_FULL_SIDE);
    walk_forward(CRATER_FULL_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Exit half-side: turn left, walk east back to entry point
    turn_left_90();
    robot_pause();
    Serial.printf("  Half-side 5: %d steps east (back to entry)\n", CRATER_HALF_SIDE);
    walk_forward(CRATER_HALF_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Robot is now back at crater entry, facing EAST

    // -----------------------------------------------
    //  PHASE 5: EXIT AND RETURN
    //  Rise to max height, face south, walk to wall,
    //  then strafe to finish position
    // -----------------------------------------------
    Serial.println("\n[Phase 5] Exit crater and return...");
    set_height(MAX_HEIGHT);

    // Turn right 90 (east → south) to face where we came from
    Serial.println("  Turning to face south...");
    turn_right_90();
    robot_pause();

    // Walk south until 30mm from the border wall
    walk_until_wall(100, SLOW_SPEED, SLOW_GAIT_MS, STOP_DIST_MM);
    robot_pause();

    // Final strafe (direction configurable — set PHASE5_STRAFE_LEFT at top)
    set_height(NORMAL_HEIGHT);
    Serial.printf("  Final strafe %s: %d steps\n",
                   PHASE5_STRAFE_LEFT ? "LEFT" : "RIGHT",
                   PHASE5_STRAFE_STEPS);
    walk_strafe(PHASE5_STRAFE_STEPS, STRAFE_SPEED, SLOW_GAIT_MS, PHASE5_STRAFE_LEFT);

    Serial.println("\n=== MISSION COMPLETE ===");
    stop_robot();
}

void loop() {
    // Mission complete — nothing to do
}
