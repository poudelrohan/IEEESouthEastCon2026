// ============================================================
//  CHEETAH RUN — Bounding gallop for hexapod
//
//  Instead of tripod alternation (3 legs at a time),
//  this uses PAIRED LEGS — both sides move together,
//  like a cheetah's gallop where front and rear pairs
//  alternate.
//
//  How it works:
//    - FRONT pair (legs 3,4): reach forward + lift HIGH
//    - REAR pair (legs 1,6): push backward on ground
//    - Then SWAP: front plants, rear lifts + swings forward
//    - MID pair (legs 2,5): always planted as stable anchors
//
//  4 phases per bound:
//    1. FRONT REACH: front legs in air reaching forward,
//                    rear legs on ground pushing backward
//    2. FRONT SLAM:  front legs slam down, rear lifts
//                    → body SURGES forward
//    3. REAR SWING:  rear legs in air swinging forward,
//                    front legs on ground pulling backward
//    4. REAR PLANT:  rear legs slam down, front lifts
//                    → body SURGES forward again
//
//  Stability: 4 legs always on ground (2 mid + 2 front or rear)
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ============================================================
//  TUNABLES
// ============================================================

// Body height: 0=default, -90=tall, +50=low
const int HEIGHT = 0;

// Speed (ms per phase): lower = faster gallop
//   150 = sprint,  200 = fast,  300 = trot,  400 = slow
const int STRIDE_MS  = 250;    // main phases (reach/swing)
const int SWAP_MS    = 150;    // transition phases (slam/plant) — FAST

// Leg reach and push distances (A-servo offsets)
const int REACH      = 250;    // front legs forward reach
const int PUSH       = 200;    // rear legs backward push

// Lift height (B-servo offset) — how high legs go in the air
const int LIFT       = 450;    // leg lift amount

// Elbow extension during lift (C-servo offset)
const int EXT        = 180;    // outward extension while airborne

// Mid legs press down for stability
const int MID_PRESS  = 30;     // extra B press on mid legs

// How many gallop cycles
const int NUM_BOUNDS = 10;

// ============================================================
//  HOME VALUES
// ============================================================
const int HA  = 1500;
const int HBR = 1409 - HEIGHT;
const int HBL = 1590 + HEIGHT;
const int HCR = 1357;
const int HCL = 1642;

// Mid legs pressed down slightly for stability
const int HB2 = HBR - MID_PRESS;   // Leg2 mid-R
const int HB5 = HBL + MID_PRESS;   // Leg5 mid-L

// ============================================================
//  LED helper
// ============================================================
void set_led(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t c[3] = {r, g, b};
  minihexa.sensor.set_ultrasound_rgb(0, c, c);
}

// ============================================================
//  SEND FRAME
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
//  BUILD FRAME — paired legs (both sides move together)
//
//  For each pair, specify:
//    a     = A-servo offset (positive = forward)
//    b_lift = B-servo lift (positive = leg goes UP)
//    c_ext  = C-servo extension (positive = extend out)
//
//  B direction handled internally:
//    Right: B - lift = UP
//    Left:  B + lift = UP
//  C direction handled internally:
//    Right: C + ext = extend
//    Left:  C - ext = extend
// ============================================================
void build_frame(uint16_t out[18],
                 int rear_a,  int rear_b_lift,  int rear_c_ext,
                 int mid_a,   int mid_b_lift,   int mid_c_ext,
                 int front_a, int front_b_lift,  int front_c_ext) {

  // Leg1 rear-R
  out[0]  = HA  + rear_a;
  out[1]  = HBR - rear_b_lift;
  out[2]  = HCR + rear_c_ext;

  // Leg2 mid-R
  out[3]  = HA  + mid_a;
  out[4]  = HB2 - mid_b_lift;
  out[5]  = HCR + mid_c_ext;

  // Leg3 front-R
  out[6]  = HA  + front_a;
  out[7]  = HBR - front_b_lift;
  out[8]  = HCR + front_c_ext;

  // Leg4 front-L (mirrored B and C)
  out[9]  = HA  + front_a;
  out[10] = HBL + front_b_lift;
  out[11] = HCL - front_c_ext;

  // Leg5 mid-L (mirrored)
  out[12] = HA  + mid_a;
  out[13] = HB5 + mid_b_lift;
  out[14] = HCL - mid_c_ext;

  // Leg6 rear-L (mirrored)
  out[15] = HA  + rear_a;
  out[16] = HBL + rear_b_lift;
  out[17] = HCL - rear_c_ext;
}

// ============================================================
//  ONE GALLOP BOUND (4 phases)
// ============================================================
void one_bound() {
  uint16_t f[18];

  // Phase 1: FRONT REACH
  // Front pair: HIGH in air, reaching far forward
  // Rear pair: on ground, pushing backward (body moves forward)
  // Mid pair: anchored
  build_frame(f,
    -PUSH, 0,    0,        // rear: behind body, planted, pushing
    0,     0,    0,        // mid: anchor
    +REACH, +LIFT, +EXT    // front: airborne, reaching forward
  );
  send_frame(f, STRIDE_MS);

  // Phase 2: FRONT SLAM + REAR LIFT (fast transition)
  // Front pair: plants at forward position (still far forward)
  // Rear pair: lifts off to swing forward
  // → BODY SURGES FORWARD
  build_frame(f,
    -PUSH, +LIFT, +EXT,    // rear: lifts, starting to swing forward
    0,     0,     0,        // mid: anchor
    +REACH, 0,    0         // front: slammed down at forward position
  );
  send_frame(f, SWAP_MS);

  // Phase 3: REAR SWING
  // Rear pair: in air, swinging to forward position
  // Front pair: on ground, pulling backward (body moves forward)
  // Mid pair: anchored
  build_frame(f,
    +REACH, +LIFT, +EXT,   // rear: airborne, swinging to forward
    0,      0,     0,       // mid: anchor
    -PUSH,  0,     0        // front: pulling back, planted
  );
  send_frame(f, STRIDE_MS);

  // Phase 4: REAR PLANT + FRONT LIFT (fast transition)
  // Rear pair: plants at forward position
  // Front pair: lifts off for next reach
  // → BODY SURGES FORWARD AGAIN
  build_frame(f,
    +REACH, 0,    0,        // rear: planted at forward position
    0,      0,    0,        // mid: anchor
    -PUSH,  +LIFT, +EXT     // front: lifts, preparing for next reach
  );
  send_frame(f, SWAP_MS);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  delay(10000);
  minihexa.begin();
  delay(2000);

  // Start at home
  uint16_t home[18];
  build_frame(home, 0,0,0, 0,0,0, 0,0,0);
  send_frame(home, 500);

  set_led(200, 100, 0);  // ORANGE — cheetah running

  // GALLOP!
  for (int i = 0; i < NUM_BOUNDS; i++) {
    one_bound();
  }

  // Return to home
  set_led(0, 200, 0);  // GREEN — done
  build_frame(home, 0,0,0, 0,0,0, 0,0,0);
  send_frame(home, 500);
  delay(3000);
}

void loop() {}
