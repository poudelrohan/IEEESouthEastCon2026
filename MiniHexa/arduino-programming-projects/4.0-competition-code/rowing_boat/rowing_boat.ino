// ============================================================
//  ROWING THE BOAT
//
//  Body LOW to the ground, legs sweep like oars in water.
//  All 6 legs move IN SYNC (both sides together):
//
//    1. CATCH:  Legs reach FORWARD, slightly lifted
//    2. DRIVE:  Legs sweep BACKWARD on the ground (power stroke)
//               → body glides forward like a boat
//    3. FEATHER: Legs lift off at the back
//    4. RECOVERY: Legs swing forward through the air
//               → preparing for next stroke
//
//  The B-servo (shoulder) stays LOW — legs barely leave the
//  ground. The motion is HORIZONTAL, not vertical.
//  Like oars skimming the water surface.
//
//  The C-servo (elbow) stays retracted — short oar stroke,
//  close to the body. NOT extended outward.
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ============================================================
//  TUNABLES
// ============================================================

// Body height: LOW = positive (legs bent, body close to ground)
const int HEIGHT      = 80;     // low body (like sitting in a boat)

// Rowing stroke geometry
const int STROKE      = 250;    // how far legs sweep forward/back (A-servo)
const int SKIM_LIFT   = 120;    // how high legs lift during recovery (small!)
const int C_RETRACT   = 0;      // elbow stays retracted (close to body)

// Rowing speed (ms per phase)
const int CATCH_MS    = 300;    // reaching forward (moderate)
const int DRIVE_MS    = 350;    // power stroke — slightly slower for grip
const int FEATHER_MS  = 150;    // quick lift at back (snap up like oar)
const int RECOVERY_MS = 300;    // swing forward through air

// Repetition
const int NUM_STROKES = 15;

// ============================================================
//  HOME VALUES (with low body)
// ============================================================
const int HA  = 1500;
const int HBR = 1409 - HEIGHT;    // right B home (low body)
const int HBL = 1590 + HEIGHT;    // left B home (low body)
const int HCR = 1357;
const int HCL = 1642;

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
//  BUILD FRAME — all 6 legs do the same motion
//
//  a_offset:  positive = forward, negative = backward
//  b_lift:    positive = lift UP (small values — skimming!)
//  c_ext:     positive = extend outward (kept small)
//
//  Both sides mirror B and C directions automatically.
// ============================================================
void build_row(uint16_t out[18], int a_offset, int b_lift, int c_ext) {
  // Leg1 rear-R
  out[0]  = HA  + a_offset;
  out[1]  = HBR - b_lift;       // right: subtract to lift
  out[2]  = HCR + c_ext;

  // Leg2 mid-R
  out[3]  = HA  + a_offset;
  out[4]  = HBR - b_lift;
  out[5]  = HCR + c_ext;

  // Leg3 front-R
  out[6]  = HA  + a_offset;
  out[7]  = HBR - b_lift;
  out[8]  = HCR + c_ext;

  // Leg4 front-L
  out[9]  = HA  + a_offset;
  out[10] = HBL + b_lift;       // left: add to lift
  out[11] = HCL - c_ext;

  // Leg5 mid-L
  out[12] = HA  + a_offset;
  out[13] = HBL + b_lift;
  out[14] = HCL - c_ext;

  // Leg6 rear-L
  out[15] = HA  + a_offset;
  out[16] = HBL + b_lift;
  out[17] = HCL - c_ext;
}

// ============================================================
//  ONE ROWING STROKE
//
//  Like an oar in water:
//    CATCH    → oar enters water far forward
//    DRIVE    → oar sweeps back through water (power!)
//    FEATHER  → oar lifts out at the back
//    RECOVERY → oar swings forward through air
// ============================================================
void one_stroke() {
  uint16_t f[18];

  // 1. CATCH — legs reach forward, planted on ground
  //    Like placing oars in the water ahead of the boat
  build_row(f, +STROKE, 0, C_RETRACT);
  send_frame(f, CATCH_MS);

  // 2. DRIVE — legs sweep backward on the ground
  //    THE POWER STROKE — pushes body forward
  //    Legs stay planted (no lift), sweeping from front to back
  build_row(f, -STROKE, 0, C_RETRACT);
  send_frame(f, DRIVE_MS);

  // 3. FEATHER — legs lift slightly at the back
  //    Like lifting oars out of the water at end of stroke
  build_row(f, -STROKE, +SKIM_LIFT, C_RETRACT);
  send_frame(f, FEATHER_MS);

  // 4. RECOVERY — legs swing forward through the air
  //    Oars glide back to the front for next stroke
  //    Legs stay slightly lifted to clear the ground
  build_row(f, +STROKE, +SKIM_LIFT, C_RETRACT);
  send_frame(f, RECOVERY_MS);
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  delay(10000);
  minihexa.begin();
  delay(2000);

  // Start at home (low body, legs centered)
  uint16_t home[18];
  build_row(home, 0, 0, 0);
  send_frame(home, 500);

  set_led(0, 100, 200);  // LIGHT BLUE — rowing

  // ROW!
  for (int i = 0; i < NUM_STROKES; i++) {
    one_stroke();
  }

  // Done — rest
  set_led(0, 200, 0);  // GREEN — done
  build_row(home, 0, 0, 0);
  send_frame(home, 500);
  delay(3000);
}

void loop() {}
