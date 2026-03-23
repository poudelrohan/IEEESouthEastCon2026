// ============================================================
//  UPHILL CLIMB 45-DEGREE
//
//  Direct servo control — AG6 tripod pattern optimized for
//  climbing steep inclines (up to 45 degrees).
//
//  RUNS ALL 4 STRATEGIES IN SEQUENCE:
//    Each strategy runs for 20 seconds.
//    5-second blinking LED transition between them.
//    LED color tells you which strategy is running.
//
//  Strategy 1 (GREEN):  AG6 + mild tilt + slow
//  Strategy 2 (YELLOW): AG6 + strong tilt + slow
//  Strategy 3 (BLUE):   AG6 + mild tilt + original speed
//  Strategy 4 (RED):    AG6 + strong tilt + original speed
//
//  All strategies use your proven obstacle_avoidance_model
//  AG6 lift values (500/300/300). Only tilt and speed differ.
//
//  Body posture: DOG STYLE — front looks UP, rear crouches.
//
//  Tripod A: legs 1(rear-R), 3(front-R), 5(mid-L)
//  Tripod B: legs 2(mid-R),  4(front-L), 6(rear-L)
//
//  Servo joints:
//    A = hip swing (forward/backward)
//    B = shoulder lift (swings whole leg UP)
//    C = elbow (extend/retract)
//
//  B direction: right < home = UP, left > home = UP
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ============================================================
//  TIMING
// ============================================================
const unsigned long STRATEGY_DURATION_MS = 20000;  // 20 sec per strategy
const int BLINK_DURATION_MS             = 5000;    // 5 sec blink between
const int HOME_MS                       = 500;

// ============================================================
//  RUNTIME PARAMETERS (set by apply_strategy)
// ============================================================
int HEIGHT_OFFSET, PITCH_OFFSET, MID_B_PRESS;
int FRONT_B_LIFT, MID_B_LIFT, REAR_B_LIFT;
int FRONT_C_EXT, MID_C_EXT, REAR_C_EXT;
int STEP_SIZE;
int LIFT_MS, LOWER_MS, STEP_MS, PLANT_MS;

// ============================================================
//  DERIVED HOME VALUES (recalculated per strategy)
// ============================================================
const int HA  = 1500;
const int HCR = 1357;
const int HCL = 1642;

int HBR_BASE, HBL_BASE;
int HB1, HB2, HB3, HB4, HB5, HB6;

// Recalculate all derived B-servo homes from current parameters
void recalc_homes() {
  HBR_BASE = 1409 - HEIGHT_OFFSET;
  HBL_BASE = 1590 + HEIGHT_OFFSET;

  // Dog posture: front RAISED (toward 1500), rear CROUCHED (away from 1500)
  HB1 = HBR_BASE - PITCH_OFFSET;     // Leg1 rear-R:  crouch (away from 1500)
  HB2 = HBR_BASE - MID_B_PRESS;      // Leg2 mid-R:   press down
  HB3 = HBR_BASE + PITCH_OFFSET;     // Leg3 front-R: look up (toward 1500)
  HB4 = HBL_BASE - PITCH_OFFSET;     // Leg4 front-L: look up (toward 1500)
  HB5 = HBL_BASE + MID_B_PRESS;      // Leg5 mid-L:   press down
  HB6 = HBL_BASE + PITCH_OFFSET;     // Leg6 rear-L:  crouch (away from 1500)
}

// ============================================================
//  APPLY STRATEGY — sets all parameters for strategy 1-4
// ============================================================
void apply_strategy(int s) {
  // All strategies use AG6 proven lift values
  FRONT_B_LIFT = 500;
  MID_B_LIFT   = 300;
  REAR_B_LIFT  = 300;
  FRONT_C_EXT  = 200;
  MID_C_EXT    = 130;
  REAR_C_EXT   = 30;
  HEIGHT_OFFSET = -90;

  switch (s) {
    case 1:  // AG6 + mild tilt + slow
      PITCH_OFFSET = 60;
      MID_B_PRESS  = 40;
      STEP_SIZE    = 250;
      LIFT_MS      = 600;
      LOWER_MS     = 500;
      STEP_MS      = 600;
      PLANT_MS     = 0;
      break;

    case 2:  // AG6 + strong tilt + slow
      PITCH_OFFSET = 120;
      MID_B_PRESS  = 70;
      STEP_SIZE    = 200;
      LIFT_MS      = 600;
      LOWER_MS     = 500;
      STEP_MS      = 600;
      PLANT_MS     = 100;
      break;

    case 3:  // AG6 + mild tilt + original speed
      PITCH_OFFSET = 60;
      MID_B_PRESS  = 40;
      STEP_SIZE    = 300;
      LIFT_MS      = 500;
      LOWER_MS     = 400;
      STEP_MS      = 500;
      PLANT_MS     = 0;
      break;

    case 4:  // AG6 + strong tilt + original speed
      PITCH_OFFSET = 120;
      MID_B_PRESS  = 70;
      STEP_SIZE    = 250;
      LIFT_MS      = 500;
      LOWER_MS     = 400;
      STEP_MS      = 500;
      PLANT_MS     = 0;
      break;
  }

  recalc_homes();
}

// ============================================================
//  LED helpers
// ============================================================
void set_led(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t c[3] = {r, g, b};
  minihexa.sensor.set_ultrasound_rgb(0, c, c);
}

// Blink the next strategy's color for transition
void blink_transition(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < 5; i++) {
    set_led(r, g, b);
    delay(500);
    set_led(0, 0, 0);
    delay(500);
  }
}

// Set solid LED color for strategy number
void set_strategy_led(int s) {
  switch (s) {
    case 1: set_led(0, 200, 0);     break;  // GREEN
    case 2: set_led(200, 200, 0);   break;  // YELLOW
    case 3: set_led(0, 0, 200);     break;  // BLUE
    case 4: set_led(200, 0, 0);     break;  // RED
  }
}

// Get LED color for a strategy (for blinking)
void get_strategy_color(int s, uint8_t &r, uint8_t &g, uint8_t &b) {
  switch (s) {
    case 1: r=0;   g=200; b=0;   break;  // GREEN
    case 2: r=200; g=200; b=0;   break;  // YELLOW
    case 3: r=0;   g=0;   b=200; break;  // BLUE
    case 4: r=200; g=0;   b=0;   break;  // RED
  }
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

void set_home(uint16_t d[18]) {
  d[0] = HA;   d[1] = HB1;  d[2] = HCR;
  d[3] = HA;   d[4] = HB2;  d[5] = HCR;
  d[6] = HA;   d[7] = HB3;  d[8] = HCR;
  d[9] = HA;   d[10] = HB4; d[11] = HCL;
  d[12] = HA;  d[13] = HB5; d[14] = HCL;
  d[15] = HA;  d[16] = HB6; d[17] = HCL;
}

void lift_leg(uint16_t d[18], int b_idx, int c_idx,
              int b_lift, int c_ext, int side, int b_home) {
  if (side == 0) {
    d[b_idx] = b_home - b_lift;
    d[c_idx] = HCR + c_ext;
  } else {
    d[b_idx] = b_home + b_lift;
    d[c_idx] = HCL - c_ext;
  }
}

// Tripod A: legs 1(rear-R), 3(front-R), 5(mid-L)
void frame_tripod_a_lift(uint16_t d[18]) {
  set_home(d);
  lift_leg(d, 1, 2, REAR_B_LIFT, REAR_C_EXT, 0, HB1);
  lift_leg(d, 7, 8, FRONT_B_LIFT, FRONT_C_EXT, 0, HB3);
  lift_leg(d, 13, 14, MID_B_LIFT, MID_C_EXT, 1, HB5);
}

// Tripod B: legs 2(mid-R), 4(front-L), 6(rear-L)
void frame_tripod_b_lift(uint16_t d[18]) {
  set_home(d);
  lift_leg(d, 4, 5, MID_B_LIFT, MID_C_EXT, 0, HB2);
  lift_leg(d, 10, 11, FRONT_B_LIFT, FRONT_C_EXT, 1, HB4);
  lift_leg(d, 16, 17, REAR_B_LIFT, REAR_C_EXT, 1, HB6);
}

// Step after tripod A lifted
void frame_step_after_a(uint16_t d[18]) {
  set_home(d);
  d[0]  = HA + STEP_SIZE;
  d[3]  = HA - STEP_SIZE;
  d[6]  = HA + STEP_SIZE;
  d[9]  = HA + STEP_SIZE;
  d[12] = HA - STEP_SIZE;
  d[15] = HA + STEP_SIZE;
}

// Step after tripod B lifted
void frame_step_after_b(uint16_t d[18]) {
  set_home(d);
  d[0]  = HA - STEP_SIZE;
  d[3]  = HA + STEP_SIZE;
  d[6]  = HA - STEP_SIZE;
  d[9]  = HA - STEP_SIZE;
  d[12] = HA + STEP_SIZE;
  d[15] = HA - STEP_SIZE;
}

// ============================================================
//  ONE FULL STRIDE
// ============================================================
void one_stride() {
  uint16_t f[18];

  frame_tripod_a_lift(f);
  send_frame(f, LIFT_MS);

  set_home(f);
  send_frame(f, LOWER_MS);

  if (PLANT_MS > 0) delay(PLANT_MS);

  frame_step_after_a(f);
  send_frame(f, STEP_MS);

  frame_tripod_b_lift(f);
  send_frame(f, LIFT_MS);

  set_home(f);
  send_frame(f, LOWER_MS);

  if (PLANT_MS > 0) delay(PLANT_MS);

  frame_step_after_b(f);
  send_frame(f, STEP_MS);
}

// ============================================================
//  RUN ONE STRATEGY FOR A FIXED DURATION
// ============================================================
void run_strategy(int s, unsigned long duration_ms) {
  apply_strategy(s);

  // Go to this strategy's home position
  uint16_t home[18];
  set_home(home);
  send_frame(home, HOME_MS);

  // Solid LED = strategy running
  set_strategy_led(s);

  // Stride until time runs out
  unsigned long start = millis();
  while (millis() - start < duration_ms) {
    one_stride();
  }

  // Return to home after this strategy
  set_home(home);
  send_frame(home, HOME_MS);
}

// ============================================================
//  SETUP — runs all 4 strategies in sequence
// ============================================================
void setup() {
  delay(10000);       // 10s safety — unplug USB!
  minihexa.begin();
  delay(2000);

  for (int s = 1; s <= 4; s++) {
    // Blink transition: show next strategy's color
    uint8_t r, g, b;
    get_strategy_color(s, r, g, b);
    blink_transition(r, g, b);

    // Run this strategy for 20 seconds
    run_strategy(s, STRATEGY_DURATION_MS);
  }

  // All done
  set_led(200, 200, 200);  // WHITE — finished
  delay(5000);
}

void loop() {}
