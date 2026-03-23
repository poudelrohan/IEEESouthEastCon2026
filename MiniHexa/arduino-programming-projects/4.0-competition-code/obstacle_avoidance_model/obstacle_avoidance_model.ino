// ============================================================
//  OBSTACLE AVOIDANCE MODEL
//
//  Direct servo control — AG6 tripod pattern with ASYMMETRIC lifts.
//  The B-servo (outer shoulder joint) cranks the leg HIGH off the
//  ground, just like AG6 does with 500+ duty unit deflections.
//
//  Tripod A: legs 1(rear-R), 3(front-R), 5(mid-L)
//  Tripod B: legs 2(mid-R),  4(front-L), 6(rear-L)
//
//  Pattern per stride:
//    1. Tripod A lifts (B-servos crank UP)
//    2. All legs step forward (A-servos shift)
//    3. Tripod B lifts (B-servos crank UP)
//    4. All legs step forward (A-servos shift)
//
//  Asymmetric lifts:
//    Front legs (3,4) → HIGHEST B-servo lift
//    Mid legs (2,5)   → moderate B-servo lift
//    Rear legs (1,6)  → minimal B-servo lift (traction)
//
//  Servo joints:
//    A = hip swing (forward/backward)
//    B = shoulder lift (the BIG one — swings whole leg UP)
//    C = elbow (extend/retract)
//
//  B direction: right < home = UP, left > home = UP
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ============================================================
//  TUNABLES
// ============================================================

// --- BODY HEIGHT ---
// 0 = default AG6 height, positive = LOWER body, negative = RAISE body
// Range: -100 (tall) to +200 (very low/crouching)
// This shifts ALL B-servos toward "more bent" = body drops
const int HEIGHT_OFFSET = -90;

// --- SPEED ---
// Controls how fast each frame plays. BIGGER number = SLOWER.
// 200 = very fast, 300 = AG6 speed, 500 = slow, 800 = very slow
const int LIFT_MS       = 500;    // time for legs to swing UP
const int LOWER_MS      = 400;    // time for legs to come DOWN gently (prevents slamming)
const int STEP_MS       = 500;    // time for step forward

// --- B-servo lift amounts (duty units, NOT degrees) ---
// AG6 original uses ~500-560 for ALL legs. We make it asymmetric.
// 500 units ≈ 70 degrees, 300 ≈ 42 deg, 100 ≈ 14 deg
const int FRONT_B_LIFT  = 500;    // front legs — HIGHEST (like AG6)
const int MID_B_LIFT    = 300;    // middle legs — moderate
const int REAR_B_LIFT   = 300;    // rear legs — barely lift (grip!)

// --- C-servo extension during lift (helps leg clear obstacles) ---
const int FRONT_C_EXT   = 200;    // front legs extend outward when lifted
const int MID_C_EXT     = 130;    // middle legs extend a bit
const int REAR_C_EXT    = 30;      // rear legs stay retracted

// --- A-servo step size (how far legs swing forward/back) ---
// Bigger = longer strides. AG6 uses ~200.
// Min: 80 (tiny shuffle)  Max: 300 (big stride, legs may collide beyond this)
const int STEP_SIZE     = 300;    // A-servo offset for stepping

// --- Timing ---
const int HOME_MS       = 500;    // initial home frame

// --- How many strides (1 stride = tripod A + tripod B) ---
const int NUM_STRIDES   = 10;

// ============================================================
//  HOME SERVO VALUES (from AG6 frame 1)
//  HEIGHT_OFFSET shifts B-servos: right goes more negative (bent),
//  left goes more positive (bent) → body drops lower.
// ============================================================
const int HA  = 1500;                     // all legs A-servo home
const int HBR = 1409 - HEIGHT_OFFSET;     // right legs B home (lower value = more bent = lower body)
const int HBL = 1590 + HEIGHT_OFFSET;     // left legs B home (higher value = more bent = lower body)
const int HCR = 1357;                     // right legs C-servo home
const int HCL = 1642;                     // left legs C-servo home

// ============================================================
//  LED helper
// ============================================================
void set_led(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t c[3] = {r, g, b};
  minihexa.sensor.set_ultrasound_rgb(0, c, c);
}

// ============================================================
//  SEND FRAME — writes all 18 servos at once
// ============================================================
void send_frame(uint16_t d[18], int ms) {
  ServoArg_t s[18];
  for (int i = 0; i < 18; i++) {
    s[i].id = i + 1;
    s[i].duty = d[i];
  }
  minihexa.multi_servo_control(s, 18, ms);
  delay(ms + 30);
}

// ============================================================
//  FRAME BUILDERS
//
//  Servo index layout (0-based):
//    Leg1 rear-R:  [0]=A  [1]=B  [2]=C
//    Leg2 mid-R:   [3]=A  [4]=B  [5]=C
//    Leg3 front-R: [6]=A  [7]=B  [8]=C
//    Leg4 front-L: [9]=A  [10]=B [11]=C
//    Leg5 mid-L:   [12]=A [13]=B [14]=C
//    Leg6 rear-L:  [15]=A [16]=B [17]=C
// ============================================================

// Set all 18 servos to home position
void set_home(uint16_t d[18]) {
  // Leg1 rear-R
  d[0] = HA;   d[1] = HBR;  d[2] = HCR;
  // Leg2 mid-R
  d[3] = HA;   d[4] = HBR;  d[5] = HCR;
  // Leg3 front-R
  d[6] = HA;   d[7] = HBR;  d[8] = HCR;
  // Leg4 front-L
  d[9] = HA;   d[10] = HBL; d[11] = HCL;
  // Leg5 mid-L
  d[12] = HA;  d[13] = HBL; d[14] = HCL;
  // Leg6 rear-L
  d[15] = HA;  d[16] = HBL; d[17] = HCL;
}

// Lift a leg by cranking B-servo UP and extending C
// side: 0=right, 1=left
void lift_leg(uint16_t d[18], int b_idx, int c_idx, int b_lift, int c_ext, int side) {
  if (side == 0) {
    // Right leg: B goes DOWN (subtract) to lift UP
    d[b_idx] = HBR - b_lift;
    d[c_idx] = HCR + c_ext;
  } else {
    // Left leg: B goes UP (add) to lift UP
    d[b_idx] = HBL + b_lift;
    d[c_idx] = HCL - c_ext;
  }
}

// ============================================================
//  TRIPOD A LIFT: legs 1(rear-R), 3(front-R), 5(mid-L)
// ============================================================
void frame_tripod_a_lift(uint16_t d[18]) {
  set_home(d);
  // Leg1 rear-R: B=[1], C=[2], right side
  lift_leg(d, 1, 2, REAR_B_LIFT, REAR_C_EXT, 0);
  // Leg3 front-R: B=[7], C=[8], right side
  lift_leg(d, 7, 8, FRONT_B_LIFT, FRONT_C_EXT, 0);
  // Leg5 mid-L: B=[13], C=[14], left side
  lift_leg(d, 13, 14, MID_B_LIFT, MID_C_EXT, 1);
}

// ============================================================
//  TRIPOD B LIFT: legs 2(mid-R), 4(front-L), 6(rear-L)
// ============================================================
void frame_tripod_b_lift(uint16_t d[18]) {
  set_home(d);
  // Leg2 mid-R: B=[4], C=[5], right side
  lift_leg(d, 4, 5, MID_B_LIFT, MID_C_EXT, 0);
  // Leg4 front-L: B=[10], C=[11], left side
  lift_leg(d, 10, 11, FRONT_B_LIFT, FRONT_C_EXT, 1);
  // Leg6 rear-L: B=[16], C=[17], left side
  lift_leg(d, 16, 17, REAR_B_LIFT, REAR_C_EXT, 1);
}

// ============================================================
//  STEP FORWARD — all 6 legs shift A-servos to push body forward
//
//  Tripod that just lifted: A swings FORWARD (+)
//  Tripod on ground: A pushes BACKWARD (-) = propels body
//
//  After Tripod A lifted:
//    Legs 1,3,5 (in air) → A goes forward
//    Legs 2,4,6 (ground) → A goes backward (push)
//
//  After Tripod B lifted:
//    Legs 2,4,6 (in air) → A goes forward
//    Legs 1,3,5 (ground) → A goes backward (push)
// ============================================================
void frame_step_after_a(uint16_t d[18]) {
  set_home(d);
  // Legs 1,3,5 just lifted → swing forward
  d[0]  = HA + STEP_SIZE;   // Leg1 A forward
  d[6]  = HA + STEP_SIZE;   // Leg3 A forward
  d[12] = HA - STEP_SIZE;   // Leg5 A (left side: -=forward... wait)

  // Actually A-servo direction:
  // From AG6 stepping frames, forward means:
  //   Legs 1,3: A > 1500 (they had +201, +168)
  //   Legs 4,6: A > 1500 (they had +201, +168)  — same direction!
  //   Legs 2,5: A < 1500 (they had -214, -214)  — opposite
  //
  // Legs on ground push backward = opposite of above
  //
  // After tripod A lifted (1,3,5 in air):
  //   Leg1(rear-R) in air → A = HA + STEP   (swing forward)
  //   Leg3(front-R) in air → A = HA + STEP  (swing forward)
  //   Leg5(mid-L) in air → A = HA - STEP    (swing forward for left mid)
  //   Leg2(mid-R) ground → A = HA - STEP    (push backward)
  //   Leg4(front-L) ground → A = HA + STEP  (push backward for left front)
  //   Leg6(rear-L) ground → A = HA + STEP   (push backward for left rear)

  // Replicate AG6 step pattern exactly:
  // AG6 frame 3 after tripod A lift:
  //   Leg1: A=1701 (+201)  Leg2: A=1286 (-214)  Leg3: A=1668 (+168)
  //   Leg4: A=1701 (+201)  Leg5: A=1286 (-214)  Leg6: A=1668 (+168)

  d[0]  = HA + STEP_SIZE;   // Leg1 rear-R
  d[3]  = HA - STEP_SIZE;   // Leg2 mid-R
  d[6]  = HA + STEP_SIZE;   // Leg3 front-R
  d[9]  = HA + STEP_SIZE;   // Leg4 front-L
  d[12] = HA - STEP_SIZE;   // Leg5 mid-L
  d[15] = HA + STEP_SIZE;   // Leg6 rear-L
}

void frame_step_after_b(uint16_t d[18]) {
  set_home(d);
  // AG6 frame 5 after tripod B lift:
  //   Leg1: A=1331 (-169)  Leg2: A=1713 (+213)  Leg3: A=1298 (-202)
  //   Leg4: A=1331 (-169)  Leg5: A=1713 (+213)  Leg6: A=1298 (-202)
  // This is exactly the OPPOSITE of step_after_a

  d[0]  = HA - STEP_SIZE;   // Leg1 rear-R
  d[3]  = HA + STEP_SIZE;   // Leg2 mid-R
  d[6]  = HA - STEP_SIZE;   // Leg3 front-R
  d[9]  = HA - STEP_SIZE;   // Leg4 front-L
  d[12] = HA + STEP_SIZE;   // Leg5 mid-L
  d[15] = HA - STEP_SIZE;   // Leg6 rear-L
}

// ============================================================
//  ONE FULL STRIDE (tripod A + tripod B)
//
//  6 frames per stride (added LOWER phase to prevent slamming):
//    1. Tripod A swings UP
//    2. Tripod A lowers DOWN gently (legs go to home height)
//    3. All legs step forward
//    4. Tripod B swings UP
//    5. Tripod B lowers DOWN gently
//    6. All legs step forward
// ============================================================
void one_stride() {
  uint16_t f[18];

  // 1. Lift tripod A (legs 1,3,5) — swing UP
  frame_tripod_a_lift(f);
  send_frame(f, LIFT_MS);

  // 2. Lower tripod A gently — B-servos return to home (soft landing)
  set_home(f);
  send_frame(f, LOWER_MS);

  // 3. Step forward (A-servos shift)
  frame_step_after_a(f);
  send_frame(f, STEP_MS);

  // 4. Lift tripod B (legs 2,4,6) — swing UP
  frame_tripod_b_lift(f);
  send_frame(f, LIFT_MS);

  // 5. Lower tripod B gently — B-servos return to home (soft landing)
  set_home(f);
  send_frame(f, LOWER_MS);

  // 6. Step forward (A-servos shift)
  frame_step_after_b(f);
  send_frame(f, STEP_MS);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  delay(10000);       // 10s safety — unplug USB!
  minihexa.begin();
  delay(2000);

  // Start at home
  uint16_t home[18];
  set_home(home);
  send_frame(home, HOME_MS);

  set_led(0, 200, 0);  // GREEN — walking

  // Do N strides
  for (int i = 0; i < NUM_STRIDES; i++) {
    one_stride();
  }

  // Return to home
  set_led(0, 0, 200);  // BLUE — done
  set_home(home);
  send_frame(home, HOME_MS);
  delay(3000);
}

void loop() {}
