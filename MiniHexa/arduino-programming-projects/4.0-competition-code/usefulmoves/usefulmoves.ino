// ============================================================
//  USEFUL MOVES — 5 Basic Movement Tests
//
//  Runs 5 moves in sequence with LED color indicators.
//  3-second pause between each move.
//
//  Move 1 (RED):    Side to side strafe (left then right)
//  Move 2 (GREEN):  Straight forward, obstacle avoidance mode
//  Move 3 (BLUE):   Straight backward, obstacle avoidance mode
//  Move 4 (PURPLE): Rotate clockwise in place
//  Move 5 (WHITE):  Rotate clockwise while moving forward (arc)
//
//  CYAN = all done
//
//  WARNING: Do NOT connect USB while running!
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ==========================================
//  TUNABLE PARAMETERS
// ==========================================
const int   MOVE_DURATION_MS = 10000;   // 10 seconds per move
const int   PAUSE_MS         = 3000;    // 3 second pause between moves
const float WALK_SPEED       = 2.0f;    // Forward/backward speed
const float STRAFE_SPEED     = 2.5f;    // Side-to-side speed
const float TURN_SPEED       = 2.0f;    // Rotation speed (omega)
const float ARC_FORWARD      = 2.0f;    // Forward component during arc
const float ARC_TURN         = -1.5f;   // Clockwise turn during arc (negative = CW)
const int   GAIT_MS          = 600;     // Gait period (ms per step cycle)

// ==========================================
//  RGB COLORS
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
Euler_t    att = {0.0f, 0.0f, 0.0f};

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
    minihexa.move(&vel, &pos, &att, GAIT_MS);
    delay(500);
}

// Announce move: blink color 3 times, then go solid
void announce(uint8_t* color) {
    blink_leds(color, 3, 300);
    set_leds(color);
    delay(500);
}

void pause_between() {
    stop_robot();
    set_leds(RGB_YELLOW);
    delay(PAUSE_MS);
}


// ============================================================
//  MOVE 1: SIDE TO SIDE (RED)
//  Strafe left for half the time, then strafe right.
//  Pure sideways motion — no forward/backward, no rotation.
// ============================================================

void move1_side_to_side() {
    announce(RGB_RED);

    int half = MOVE_DURATION_MS / 2;

    // Strafe LEFT
    vel = {-STRAFE_SPEED, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, GAIT_MS, -1);  // continuous
    delay(half);

    // Strafe RIGHT
    vel = {STRAFE_SPEED, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, GAIT_MS, -1);
    delay(half);

    stop_robot();
}


// ============================================================
//  MOVE 2: FORWARD — OBSTACLE AVOIDANCE MODE (GREEN)
//  Walks straight forward using the built-in avoid() function.
//  The ultrasonic sensor watches ahead. If it sees an obstacle
//  within 200mm it turns, within 100mm it backs up.
//  This is the real obstacle avoidance state machine.
// ============================================================

void move2_forward_avoid() {
    announce(RGB_GREEN);

    unsigned long start = millis();
    while (millis() - start < (unsigned long)MOVE_DURATION_MS) {
        uint16_t dis = minihexa.sensor.get_distance();
        minihexa.avoid(dis);
        delay(50);
    }

    stop_robot();
}


// ============================================================
//  MOVE 3: BACKWARD — OBSTACLE AVOIDANCE STYLE (BLUE)
//  Walks backward step-by-step, checking ultrasonic each step.
//  Same speed/gait as avoid() uses (vy=2.0, gait=800ms).
//  Note: avoid() only works forward — this is the backward
//  equivalent using the same movement parameters.
// ============================================================

void move3_backward() {
    announce(RGB_BLUE);

    unsigned long start = millis();
    while (millis() - start < (unsigned long)MOVE_DURATION_MS) {
        // Walk backward — same speed as avoid() uses
        vel = {0.0f, -2.0f, 0.0f};
        minihexa.move(&vel, &pos, &att, 800, 1);  // 1 step at a time
        delay(850);
    }

    stop_robot();
}


// ============================================================
//  MOVE 4: ROTATE CLOCKWISE IN PLACE (PURPLE)
//  Spins clockwise (negative omega) without moving forward
//  or sideways. Like a tank turning in place.
// ============================================================

void move4_rotate_cw() {
    announce(RGB_PURPLE);

    // Negative omega = clockwise
    vel = {0.0f, 0.0f, -TURN_SPEED};
    minihexa.move(&vel, &pos, &att, GAIT_MS, -1);  // continuous
    delay(MOVE_DURATION_MS);

    stop_robot();
}


// ============================================================
//  MOVE 5: ROTATE CLOCKWISE + FORWARD (WHITE)
//  Walks forward while rotating clockwise simultaneously.
//  This creates a curved arc path — like driving in a circle.
//  Useful for orbiting around objects.
// ============================================================

void move5_arc_cw() {
    announce(RGB_WHITE);

    // Forward + clockwise rotation = right-curving arc
    vel = {0.0f, ARC_FORWARD, ARC_TURN};
    minihexa.move(&vel, &pos, &att, GAIT_MS, -1);  // continuous
    delay(MOVE_DURATION_MS);

    stop_robot();
}


// ============================================================
//  SETUP
// ============================================================

void setup() {
    delay(10000);   // 10s safety — unplug USB, place on ground

    Serial.begin(115200);
    minihexa.begin();
    delay(3000);    // Stand up and stabilize

    // Starting signal
    blink_leds(RGB_YELLOW, 5, 200);
    delay(1000);

    // ---- Move 1: Side to Side (RED) ----
    move1_side_to_side();
    pause_between();

    // ---- Move 2: Forward Obstacle Avoidance (GREEN) ----
    move2_forward_avoid();
    pause_between();

    // ---- Move 3: Backward (BLUE) ----
    move3_backward();
    pause_between();

    // ---- Move 4: Rotate Clockwise (PURPLE) ----
    move4_rotate_cw();
    pause_between();

    // ---- Move 5: Arc Forward + Clockwise (WHITE) ----
    move5_arc_cw();

    // ---- Done (CYAN) ----
    stop_robot();
    set_leds(RGB_CYAN);
}

void loop() {
    // All done
}
