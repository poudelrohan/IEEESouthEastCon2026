// ============================================================
//  AG6 TUNABLE
//
//  EXACT Action Group 6 (Obstacle Crossing) servo values
//  extracted from the .rob file. Not an imitation — these
//  are the real AG6 numbers with HEIGHT and SPEED on top.
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ============================================================
//  TUNABLES
// ============================================================

// BODY HEIGHT:
//   0   = AG6 default height
//   -90 = TALL (raised body, more leg clearance)
//   +50 = LOW  (crouching, lower center of gravity)
const int HEIGHT = 50;

// SPEED (ms per walking frame):
//   200 = very fast
//   300 = AG6 original speed
//   500 = slow
//   700 = very slow
const int SPEED = 300;

// HOW MANY STRIDES (AG6 original does 5)
const int NUM_STRIDES = 5;

// STEERING — corrects drift
//   0  = no correction
//  +15 = robot was drifting RIGHT, correct it LEFT
//  -15 = robot was drifting LEFT, correct it RIGHT
//  Start with 10-15, increase if still drifting
const int STEERING = 0;

// ============================================================
//  EXACT AG6 FRAMES from No.6 Obstacle Crossing.rob
//
//  AG6 pattern per stride (4 frames):
//    Frame A: Tripod A lift (legs 1,3,5)
//    Frame B: Step (all legs shift A-servos)
//    Frame C: Tripod B lift (legs 2,4,6)
//    Frame D: Step (all legs shift A-servos, opposite)
//
//  Servo order: 1A,1B,1C, 2A,2B,2C, 3A,3B,3C,
//               4A,4B,4C, 5A,5B,5C, 6A,6B,6C
// ============================================================

// HOME position (AG6 frame 1)
const uint16_t HOME_FRAME[18] = {
  1500, 1409, 1356,   // Leg1 rear-R
  1500, 1409, 1357,   // Leg2 mid-R
  1500, 1409, 1357,   // Leg3 front-R
  1500, 1590, 1642,   // Leg4 front-L
  1500, 1590, 1642,   // Leg5 mid-L
  1500, 1590, 1642    // Leg6 rear-L
};

// Tripod A lift (AG6 frame 2) — legs 1,3,5 swing UP
const uint16_t LIFT_A[18] = {
  1500,  905, 1575,   // Leg1 rear-R:  B=905 (lift 504 units = 71 deg)
  1500, 1409, 1357,   // Leg2 mid-R:   home
  1500,  851, 1575,   // Leg3 front-R: B=851 (lift 558 units = 78 deg)
  1500, 1590, 1642,   // Leg4 front-L: home
  1500, 2176, 1506,   // Leg5 mid-L:   B=2176 (lift 586 units = 82 deg)
  1500, 1590, 1642    // Leg6 rear-L:  home
};

// Step after tripod A (AG6 frame 3)
const uint16_t STEP_A[18] = {
  1701, 1432, 1305,   // Leg1 rear-R:  A=+201
  1286, 1406, 1370,   // Leg2 mid-R:   A=-214
  1668, 1406, 1441,   // Leg3 front-R: A=+168
  1701, 1567, 1694,   // Leg4 front-L: A=+201
  1286, 1593, 1629,   // Leg5 mid-L:   A=-214
  1668, 1593, 1558    // Leg6 rear-L:  A=+168
};

// Tripod B lift (AG6 frame 4) — legs 2,4,6 swing UP
const uint16_t LIFT_B[18] = {
  1500, 1409, 1357,   // Leg1 rear-R:  home
  1500,  878, 1494,   // Leg2 mid-R:   B=878 (lift 531 units = 74 deg)
  1500, 1409, 1357,   // Leg3 front-R: home
  1500, 2068, 1452,   // Leg4 front-L: B=2068 (lift 478 units = 67 deg)
  1500, 1590, 1642,   // Leg5 mid-L:   home
  1500, 2149, 1452    // Leg6 rear-L:  B=2149 (lift 559 units = 78 deg)
};

// Step after tripod B (AG6 frame 5)
const uint16_t STEP_B[18] = {
  1331, 1406, 1441,   // Leg1 rear-R:  A=-169
  1713, 1406, 1370,   // Leg2 mid-R:   A=+213
  1298, 1423, 1305,   // Leg3 front-R: A=-202
  1331, 1593, 1558,   // Leg4 front-L: A=-169
  1713, 1593, 1629,   // Leg5 mid-L:   A=+213
  1298, 1593, 1694    // Leg6 rear-L:  A=-202
};

// ============================================================
//  LED helper
// ============================================================
void set_led(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t c[3] = {r, g, b};
  minihexa.sensor.set_ultrasound_rgb(0, c, c);
}

// ============================================================
//  SEND FRAME with HEIGHT offset applied to B-servos
//
//  B-servo indices: 1(Leg1-R), 4(Leg2-R), 7(Leg3-R),
//                  10(Leg4-L), 13(Leg5-L), 16(Leg6-L)
//
//  HEIGHT positive = lower body:
//    Right B: subtract HEIGHT (further from 1500 = more bent)
//    Left B:  add HEIGHT (further from 1500 = more bent)
// ============================================================
void send_ag6(const uint16_t src[18], int ms) {
  uint16_t d[18];
  for (int i = 0; i < 18; i++) d[i] = src[i];

  // Apply HEIGHT offset to all 6 B-servos
  d[1]  -= HEIGHT;   // Leg1 rear-R
  d[4]  -= HEIGHT;   // Leg2 mid-R
  d[7]  -= HEIGHT;   // Leg3 front-R
  d[10] += HEIGHT;   // Leg4 front-L
  d[13] += HEIGHT;   // Leg5 mid-L
  d[16] += HEIGHT;   // Leg6 rear-L

  // Apply STEERING offset to A-servos
  // Positive STEERING = right legs step more = robot curves LEFT
  // A-servo indices: 0(Leg1-R), 3(Leg2-R), 6(Leg3-R),
  //                  9(Leg4-L), 12(Leg5-L), 15(Leg6-L)
  d[0]  += STEERING;   // Leg1 rear-R
  d[3]  += STEERING;   // Leg2 mid-R
  d[6]  += STEERING;   // Leg3 front-R
  d[9]  -= STEERING;   // Leg4 front-L
  d[12] -= STEERING;   // Leg5 mid-L
  d[15] -= STEERING;   // Leg6 rear-L

  ServoArg_t s[18];
  for (int i = 0; i < 18; i++) {
    s[i].id = i + 1;
    s[i].duty = d[i];
  }
  minihexa.multi_servo_control(s, 18, ms);
  delay(ms + 30);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  delay(10000);
  minihexa.begin();
  delay(2000);

  // Frame 1: HOME (AG6 uses 800ms)
  send_ag6(HOME_FRAME, 800);

  set_led(0, 200, 0);  // GREEN — walking

  // Stride loop — exact AG6 pattern
  for (int i = 0; i < NUM_STRIDES; i++) {
    send_ag6(LIFT_A, SPEED);   // Tripod A lift
    send_ag6(STEP_A, SPEED);   // Step
    send_ag6(LIFT_B, SPEED);   // Tripod B lift
    send_ag6(STEP_B, SPEED);   // Step
  }

  // Final: return to HOME
  set_led(0, 0, 200);  // BLUE — done
  send_ag6(HOME_FRAME, SPEED);
  delay(3000);
}

void loop() {}
