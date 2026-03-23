// ============================================================
//  BUTTERFLY SWIM
//
//  Direct servo keyframe animation — like a butterfly stroke:
//
//  FRONT legs (3,4): Big sweeping arc — reach UP+FORWARD,
//                    pull DOWN+BACK. Full butterfly power stroke.
//  MIDDLE legs (2,5): Smaller butterfly stroke — assist the pull.
//  REAR legs (1,6):  RIGID — planted on the ground as anchors
//                    so the robot doesn't slide backward.
//
//  The front and middle pairs move IN SYNC (both sides together)
//  just like a swimmer's arms in butterfly — symmetrical power.
//
//  Servo mapping:
//    Leg1 rear-R:  servos 1(A), 2(B), 3(C)
//    Leg2 mid-R:   servos 4(A), 5(B), 6(C)
//    Leg3 front-R: servos 7(A), 8(B), 9(C)
//    Leg4 front-L: servos 10(A), 11(B), 12(C)
//    Leg5 mid-L:   servos 13(A), 14(B), 15(C)
//    Leg6 rear-L:  servos 16(A), 17(B), 18(C)
//
//  Joint A = hip swing (forward/backward)
//  Joint B = shoulder lift (up/down)
//  Joint C = elbow extend (in/out)
//
//  B direction: right legs < 1500 = UP, left legs > 1500 = UP
//  A direction: all legs > 1500 = swing forward(+Y)
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ============================================================
//  TUNABLES
// ============================================================

// --- Stroke geometry (servo duty offsets from home) ---

// Front legs (3,4) — big butterfly stroke
const int FRONT_B_LIFT    = 500;    // B-servo offset for UP phase (~70 deg)
const int FRONT_B_PULL    = 150;    // B-servo offset for DOWN/pull phase
const int FRONT_A_REACH   = 200;    // A-servo forward reach
const int FRONT_A_PULL    = 200;    // A-servo backward pull
const int FRONT_C_EXTEND  = 150;    // C-servo outward extension during reach

// Middle legs (2,5) — smaller butterfly stroke
const int MID_B_LIFT      = 300;    // B-servo offset for UP phase (~42 deg)
const int MID_B_PULL      = 100;    // B-servo offset for DOWN/pull phase
const int MID_A_REACH     = 150;    // A-servo forward reach
const int MID_A_PULL      = 150;    // A-servo backward pull
const int MID_C_EXTEND    = 100;    // C-servo outward extension during reach

// Rear legs (1,6) — RIGID anchor (press down slightly)
const int REAR_B_PRESS    = 50;     // slight downward press for grip

// --- Timing (ms per phase) ---
const int REACH_TIME      = 400;    // time for legs to swing UP + FORWARD
const int PULL_TIME       = 350;    // time for legs to pull DOWN + BACK (power stroke)
const int PLANT_TIME      = 200;    // brief pause with legs planted
const int HOME_TIME       = 300;    // return to neutral between strokes

// --- Repetition ---
const int NUM_STROKES     = 15;     // how many butterfly strokes to do

// --- Body height before starting (gait engine, then switch to direct) ---
const float START_HEIGHT  = 0.5f;   // low body

// ============================================================
//  HOME POSITIONS (from AG6 frame 1)
// ============================================================

// Right side homes
const int HOME_A     = 1500;
const int HOME_B_R   = 1409;   // right legs B home
const int HOME_C_R   = 1357;   // right legs C home

// Left side homes
const int HOME_B_L   = 1590;   // left legs B home
const int HOME_C_L   = 1642;   // left legs C home

// ============================================================
//  LED helper
// ============================================================
void set_led(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t c[3] = {r, g, b};
  minihexa.sensor.set_ultrasound_rgb(0, c, c);
}

// ============================================================
//  SEND ALL 18 SERVOS
// ============================================================
void send_frame(uint16_t duties[18], int move_time) {
  ServoArg_t servos[18];
  for (int i = 0; i < 18; i++) {
    servos[i].id = i + 1;
    servos[i].duty = duties[i];
  }
  minihexa.multi_servo_control(servos, 18, move_time);
  delay(move_time + 50);
}

// ============================================================
//  BUILD A FRAME
//  Sets all 18 servo duties based on offsets from home.
//  For B-servo: positive offset = LIFT for both sides
//    (internally flipped: right goes -, left goes +)
//  For A-servo: positive offset = FORWARD for both sides
//    (both sides use + for forward)
//  For C-servo: positive offset = EXTEND outward
//    (right goes +, left goes -)
// ============================================================
void build_frame(uint16_t out[18],
                 // Front legs (3=right, 4=left)
                 int front_a, int front_b_lift, int front_c,
                 // Mid legs (2=right, 5=left)
                 int mid_a, int mid_b_lift, int mid_c,
                 // Rear legs (1=right, 6=left)
                 int rear_a, int rear_b_lift, int rear_c) {

  // Leg1 rear-R: servos 0,1,2
  out[0]  = HOME_A    + rear_a;
  out[1]  = HOME_B_R  - rear_b_lift;     // right: subtract to lift
  out[2]  = HOME_C_R  + rear_c;

  // Leg2 mid-R: servos 3,4,5
  out[3]  = HOME_A    + mid_a;
  out[4]  = HOME_B_R  - mid_b_lift;      // right: subtract to lift
  out[5]  = HOME_C_R  + mid_c;

  // Leg3 front-R: servos 6,7,8
  out[6]  = HOME_A    + front_a;
  out[7]  = HOME_B_R  - front_b_lift;    // right: subtract to lift
  out[8]  = HOME_C_R  + front_c;

  // Leg4 front-L: servos 9,10,11
  out[9]  = HOME_A    + front_a;          // A same direction both sides
  out[10] = HOME_B_L  + front_b_lift;    // left: add to lift
  out[11] = HOME_C_L  - front_c;         // left: subtract to extend

  // Leg5 mid-L: servos 12,13,14
  out[12] = HOME_A    + mid_a;
  out[13] = HOME_B_L  + mid_b_lift;      // left: add to lift
  out[14] = HOME_C_L  - mid_c;

  // Leg6 rear-L: servos 15,16,17
  out[15] = HOME_A    + rear_a;
  out[16] = HOME_B_L  + rear_b_lift;     // left: add to lift (negative = press down)
  out[17] = HOME_C_L  - rear_c;
}

// ============================================================
//  ONE BUTTERFLY STROKE
//
//  Phase 1 — REACH: Front+mid legs swing UP and FORWARD
//            Rear legs press DOWN (anchor)
//
//  Phase 2 — PULL:  Front+mid legs pull DOWN and BACKWARD
//            (power stroke — propels robot forward)
//            Rear legs stay rigid
//
//  Phase 3 — PLANT: Brief hold with all legs down
//
//  Phase 4 — HOME:  Return to neutral
// ============================================================
void butterfly_stroke() {
  uint16_t frame[18];

  // --- Phase 1: REACH (legs swing up + forward) ---
  build_frame(frame,
    +FRONT_A_REACH, +FRONT_B_LIFT, +FRONT_C_EXTEND,   // front: up + forward + extend
    +MID_A_REACH,   +MID_B_LIFT,   +MID_C_EXTEND,     // mid: up + forward + extend
    0,              -REAR_B_PRESS,  0                   // rear: press down (anchor)
  );
  send_frame(frame, REACH_TIME);

  // --- Phase 2: PULL (power stroke — legs pull down + backward) ---
  build_frame(frame,
    -FRONT_A_PULL, -FRONT_B_PULL, 0,                   // front: down + backward
    -MID_A_PULL,   -MID_B_PULL,   0,                   // mid: down + backward
    0,             -REAR_B_PRESS,  0                    // rear: still anchored
  );
  send_frame(frame, PULL_TIME);

  // --- Phase 3: PLANT (brief hold, all legs grounded) ---
  build_frame(frame,
    -FRONT_A_PULL, -FRONT_B_PULL, 0,                   // front: stay in pulled position
    -MID_A_PULL,   -MID_B_PULL,   0,                   // mid: stay in pulled position
    0,             -REAR_B_PRESS,  0                    // rear: anchored
  );
  send_frame(frame, PLANT_TIME);

  // --- Phase 4: HOME (reset to neutral for next stroke) ---
  build_frame(frame,
    0, 0, 0,                                            // front: home
    0, 0, 0,                                            // mid: home
    0, -REAR_B_PRESS, 0                                 // rear: still slightly pressed
  );
  send_frame(frame, HOME_TIME);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  delay(10000);       // 10s safety — unplug USB!
  minihexa.begin();
  delay(2000);

  // Lower the body first using gait engine
  Velocity_t vel = {0.0f, 0.0f, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, START_HEIGHT};
  Euler_t    att = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, 800);
  delay(1500);

  // Go to home position with direct servo control
  uint16_t home[18];
  build_frame(home, 0, 0, 0, 0, 0, 0, 0, -REAR_B_PRESS, 0);
  send_frame(home, 500);

  set_led(0, 200, 200);  // CYAN — swimming

  // Butterfly swim!
  for (int i = 0; i < NUM_STROKES; i++) {
    butterfly_stroke();
  }

  // Done — rest at home
  set_led(0, 200, 0);  // GREEN — done
  build_frame(home, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  send_frame(home, 800);
  delay(3000);
}

void loop() {}
