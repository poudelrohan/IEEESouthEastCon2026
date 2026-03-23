// ============================================================
//  CRATER ESCAPE TESTER v3
//
//  Place the robot IN the crater by hand. Run this program.
//  It tries 6 escape strategies one after another.
//  After each attempt, IMU checks if the robot is level.
//  If level → escaped! Stops with CYAN LEDs.
//  If still tilted → tries the next strategy.
//
//  Strategy 1 (RED):    Forward pull — front legs claw uphill
//  Strategy 2 (GREEN):  Backward push — rear legs shove uphill
//  Strategy 3 (BLUE):   AG6 Obstacle Crossing x3
//  Strategy 4 (PURPLE): Rock body to break friction + forward pull
//  Strategy 5 (WHITE):  Medium height forward — the sweet spot
//  Strategy 6 (YELLOW): AG6 + backward push combo
//
//  If all fail: rapid RED blink, then loops and tries again.
//
//  *** KEY PHYSICS: ***
//  Height = 1.5cm is the sweet spot on the crater slope.
//  Too low (-2cm) = belly sits on slope, legs float in air.
//  Too high (3.5cm) = unstable, legs barely touch.
//  Medium height = ALL 6 legs contact the ground = traction.
//
//  *** NO SERIAL MONITOR — RGB LED FEEDBACK ***
//  WARNING: Do NOT connect USB while running!
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ==========================================
//  TUNABLE PARAMETERS
// ==========================================

// --- Strategy 1: Forward Pull (front legs claw uphill) ---
// Front 2 legs reach forward, grip, PULL the body up.
// Middle and rear legs push from behind.
// Like an insect climbing — front legs are the claws.
// Medium height so ALL legs touch the slope surface.
const float S1_HEIGHT       = 1.5f;    // *** SWEET SPOT — all legs touch ground
const float S1_PITCH        = 6.0f;    // Lean FORWARD — weight on front "claws"
const float S1_SPEED        = 1.0f;    // Forward (positive) — front legs pull
const int   S1_GAIT_MS      = 1500;    // Very slow = more time gripping
const int   S1_STEPS        = 15;      // Enough to cross crater radius (~30cm)

// --- Strategy 2: Backward Push (rear legs shove uphill) ---
// If robot is facing center, backward = toward the rim.
// Rear 2 legs push on the slope, middle+front legs pull.
// Slightly higher stance for more push leverage.
const float S2_HEIGHT       = 2.0f;    // Medium-tall — good leverage
const float S2_PITCH        = -6.0f;   // Lean BACK — weight on pushing rear legs
const float S2_SPEED        = 1.0f;    // Backward speed (will be negated)
const int   S2_GAIT_MS      = 1500;    // Slow and deliberate
const int   S2_STEPS        = 15;

// --- Strategy 3: AG6 Obstacle Crossing x3 ---
// Pre-recorded 21-frame climbing sequence.
// Direct servo control — different motion pattern than gait.
// Run 3 times to cover more distance.
const int   S3_AG6_RUNS     = 3;       // How many times to run AG6

// --- Strategy 4: Rock and Forward Pull ---
// Rock body back-and-forth to break static friction.
// Then immediately crawl FORWARD (front legs claw).
// Rocking shifts weight between legs, loosening grip.
const float S4_ROCK_PITCH   = 15.0f;   // Rock amplitude (degrees)
const int   S4_ROCK_COUNT   = 5;       // Number of rocks
const int   S4_ROCK_MS      = 350;     // Hold time per extreme
const float S4_HEIGHT       = 1.5f;    // Sweet spot height
const float S4_SPEED        = 1.2f;    // Forward crawl after rocking
const int   S4_GAIT_MS      = 1200;
const int   S4_STEPS        = 10;

// --- Strategy 5: Medium Height Forward (the balanced approach) ---
// The "Goldilocks" strategy. Not too low, not too high.
// No pitch lean — let the gait do its natural thing.
// Sometimes the simplest approach works best.
const float S5_HEIGHT       = 1.0f;    // Low-medium
const float S5_PITCH        = 0.0f;    // No lean — natural gait
const float S5_SPEED        = 1.5f;    // Moderate forward speed
const int   S5_GAIT_MS      = 1000;    // Normal-ish gait
const int   S5_STEPS        = 15;

// --- Strategy 6: AG6 + Backward Push Combo ---
// Run AG6 to loosen up, then immediately push backward.
// Combines the aggressive action group with directed walking.
const float S6_HEIGHT       = 2.0f;
const float S6_PITCH        = -8.0f;   // Lean back for push phase
const float S6_SPEED        = 1.5f;    // Backward push
const int   S6_GAIT_MS      = 1000;
const int   S6_STEPS        = 10;

// --- IMU Escape Detection ---
const float TILT_ESCAPE_TOL = 6.0f;    // Degrees — within this = "level" = escaped
const int   IMU_SETTLE_MS   = 500;     // Wait for IMU after motion stops

// --- Retry ---
const bool  LOOP_STRATEGIES = true;    // true = loop all strategies forever
                                        // false = try once and stop


// ==========================================
//  RGB LED COLORS
// ==========================================
uint8_t RGB_RED[3]    = {255, 0,   0  };
uint8_t RGB_GREEN[3]  = {0,   255, 0  };
uint8_t RGB_BLUE[3]   = {0,   0,   255};
uint8_t RGB_PURPLE[3] = {200, 0,   255};
uint8_t RGB_WHITE[3]  = {255, 255, 255};
uint8_t RGB_CYAN[3]   = {0,   255, 255};
uint8_t RGB_YELLOW[3] = {255, 255, 0  };
uint8_t RGB_OFF[3]    = {0,   0,   0  };


// ==========================================
//  STATE
// ==========================================
Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t   pos = {0.0f, 0.0f, 0.0f};
Euler_t    att = {0.0f, 0.0f, 0.0f};   // {pitch, roll, yaw}

float current_height = 0.0f;
float baseline_pitch = 0.0f;
bool  escaped = false;


// ==========================================
//  HELPERS
// ==========================================

void set_leds(uint8_t* color) {
    minihexa.sensor.set_ultrasound_rgb(0, color, color);
}

void blink_leds(uint8_t* color, int times, int interval_ms) {
    for (int i = 0; i < times; i++) {
        set_leds(color);
        delay(interval_ms);
        set_leds(RGB_OFF);
        delay(interval_ms);
    }
}

void stop_robot() {
    vel = {0.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, 600);
    delay(500);
}

void set_height(float z) {
    current_height = z;
    vel = {0.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, 1000);
    delay(1200);
}

float read_pitch() {
    float euler[3];
    minihexa.board.get_imu_euler(euler);
    return euler[1];
}

bool check_escaped() {
    delay(IMU_SETTLE_MS);
    float pitch_sum = 0.0f;
    for (int i = 0; i < 5; i++) {
        pitch_sum += read_pitch();
        delay(50);
    }
    float avg_pitch = pitch_sum / 5.0f;
    return (fabsf(avg_pitch - baseline_pitch) < TILT_ESCAPE_TOL);
}

// Crawl with body pitch maintained, step by step
void crawl(float speed, float height, float pitch, int gait_ms, int steps) {
    current_height = height;
    att.pitch = pitch;
    att.roll  = 0.0f;
    att.yaw   = 0.0f;

    for (int i = 0; i < steps; i++) {
        vel = {0.0f, speed, 0.0f};
        pos = {0.0f, 0.0f, current_height};
        minihexa.move(&vel, &pos, &att, gait_ms, 1);
        delay(gait_ms + 300);
    }

    stop_robot();
}

void announce(uint8_t* color) {
    blink_leds(color, 3, 300);
    set_leds(color);
    delay(500);
}


// ============================================================
//  STRATEGY 1: FORWARD PULL (RED)
//
//  The insect climbing strategy.
//  Front 2 legs = CLAWS. They reach forward up the slope,
//  dig in, and pull the body toward the rim.
//  Middle 4 legs = support and push from behind.
//
//  Height at 1.5cm = the sweet spot where ALL 6 legs
//  maintain ground contact on the concave crater slope.
//  Too low → belly on ground, legs float.
//  Too high → only leg tips touch, no grip.
//
//  Lean forward slightly to load the front "claw" legs
//  so they press harder into the slope = more friction.
//  Slow gait gives each leg max time gripping.
// ============================================================

void strategy_1() {
    announce(RGB_RED);
    crawl(S1_SPEED, S1_HEIGHT, S1_PITCH, S1_GAIT_MS, S1_STEPS);
}


// ============================================================
//  STRATEGY 2: BACKWARD PUSH (GREEN)
//
//  If facing the center, backward = toward the rim.
//  Rear legs push against the slope. Lean back to load them.
//  Slightly taller for more leverage on each push.
// ============================================================

void strategy_2() {
    announce(RGB_GREEN);
    crawl(-S2_SPEED, S2_HEIGHT, S2_PITCH, S2_GAIT_MS, S2_STEPS);
}


// ============================================================
//  STRATEGY 3: AG6 OBSTACLE CROSSING x3 (BLUE)
//
//  Pre-recorded 21-frame climbing sequence.
//  Uses direct servo control — completely different motion
//  pattern from the tripod gait. May keep more legs grounded.
//  Run 3 times to cover more distance up the slope.
// ============================================================

void strategy_3() {
    announce(RGB_BLUE);

    att = {0.0f, 0.0f, 0.0f};
    set_height(0.0f);
    delay(500);

    for (int i = 0; i < S3_AG6_RUNS; i++) {
        minihexa.action_group_run(6);
        delay(800);
    }
}


// ============================================================
//  STRATEGY 4: ROCK AND FORWARD PULL (PURPLE)
//
//  Static friction on smooth PLA is the enemy.
//  Rocking the body forward/backward rapidly shifts weight
//  between different legs, breaking their static friction.
//
//  Think: wiggling a heavy object before sliding it.
//  The rocking creates tiny forward impulses. Then
//  immediately crawl forward while the surface grip
//  is "loosened."
//
//  This also exercises the legs through their full range
//  of motion, which helps if any legs are jammed against
//  the crater wall.
// ============================================================

void strategy_4() {
    announce(RGB_PURPLE);

    set_height(S4_HEIGHT);
    delay(500);

    // Rock body back and forth aggressively
    for (int i = 0; i < S4_ROCK_COUNT; i++) {
        att.pitch = S4_ROCK_PITCH;
        vel = {0.0f, 0.0f, 0.0f};
        pos = {0.0f, 0.0f, current_height};
        minihexa.move(&vel, &pos, &att, 300);
        delay(S4_ROCK_MS);

        att.pitch = -S4_ROCK_PITCH;
        minihexa.move(&vel, &pos, &att, 300);
        delay(S4_ROCK_MS);
    }

    // Immediately crawl forward — front legs claw
    crawl(S4_SPEED, S4_HEIGHT, 6.0f, S4_GAIT_MS, S4_STEPS);
}


// ============================================================
//  STRATEGY 5: MEDIUM HEIGHT FORWARD — "GOLDILOCKS" (WHITE)
//
//  Sometimes overthinking it is the problem.
//  Just walk forward at a sensible height with no pitch lean.
//  Let the natural gait do its thing.
//  1.0cm height = legs comfortably reach the slope.
//  No pitch = even weight distribution across all 6 legs.
// ============================================================

void strategy_5() {
    announce(RGB_WHITE);
    crawl(S5_SPEED, S5_HEIGHT, S5_PITCH, S5_GAIT_MS, S5_STEPS);
}


// ============================================================
//  STRATEGY 6: AG6 + BACKWARD PUSH COMBO (YELLOW)
//
//  Run AG6 first to loosen things up and move a bit,
//  then immediately switch to backward push to drive
//  toward the rim. Combines the aggressive servo patterns
//  of the action group with directed walking.
// ============================================================

void strategy_6() {
    announce(RGB_YELLOW);

    // AG6 first — loosen up
    att = {0.0f, 0.0f, 0.0f};
    set_height(0.0f);
    delay(300);
    minihexa.action_group_run(6);
    delay(500);

    // Now backward push
    crawl(-S6_SPEED, S6_HEIGHT, S6_PITCH, S6_GAIT_MS, S6_STEPS);
}


// ============================================================
//  SETUP
// ============================================================

void setup() {
    delay(10000);   // *** 10s SAFETY — unplug USB, place in crater ***

    Serial.begin(115200);
    minihexa.begin();
    delay(3000);    // Stand up

    // Enable IMU
    minihexa.board.imu_update(true);
    delay(2000);    // Let Madgwick filter stabilize

    // Baseline: flat ground ≈ 0 degrees pitch
    baseline_pitch = 0.0f;

    // Signal: YELLOW 5x = "starting escape attempts"
    blink_leds(RGB_YELLOW, 5, 300);
    delay(2000);

    // ---- RUN ESCAPE STRATEGIES ----
    do {
        // Strategy 1: Forward Pull (RED)
        strategy_1();
        if (check_escaped()) { escaped = true; break; }
        delay(2000);

        // Strategy 2: Backward Push (GREEN)
        strategy_2();
        if (check_escaped()) { escaped = true; break; }
        delay(2000);

        // Strategy 3: AG6 x3 (BLUE)
        strategy_3();
        if (check_escaped()) { escaped = true; break; }
        delay(2000);

        // Strategy 4: Rock + Forward Pull (PURPLE)
        strategy_4();
        if (check_escaped()) { escaped = true; break; }
        delay(2000);

        // Strategy 5: Goldilocks Forward (WHITE)
        strategy_5();
        if (check_escaped()) { escaped = true; break; }
        delay(2000);

        // Strategy 6: AG6 + Backward Push (YELLOW)
        strategy_6();
        if (check_escaped()) { escaped = true; break; }

        // All 6 failed — distress signal
        blink_leds(RGB_RED, 10, 100);
        delay(3000);

    } while (LOOP_STRATEGIES && !escaped);

    // ---- RESULT ----
    if (escaped) {
        set_height(0.0f);
        set_leds(RGB_CYAN);
        blink_leds(RGB_CYAN, 10, 150);
        set_leds(RGB_CYAN);
    } else {
        set_leds(RGB_RED);
    }

    stop_robot();
}

void loop() {
    // Escaped: CYAN stays on
    // Stuck: RED stays on
}
