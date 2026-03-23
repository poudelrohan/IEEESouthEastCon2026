// ============================================================
//  CRATER SEQUENCE CLIMB
//
//  Caterpillar/sequential gait walking BACKWARD up the crater:
//
//    1. Rear legs (1, 6) step first  — claw HIGH into the slope
//    2. Middle legs (2, 5) step next — share the load
//    3. Front legs (3, 4) step last  — trailing, just reposition
//    Repeat.
//
//  Only 2 legs swing at a time — 4 always grounded.
//  This gives maximum grip: 4 contact points pushing uphill
//  while 2 legs reposition (vs 3 in tripod).
//
//  Bigger steps than crater_reverse:
//    max_half_step_length raised to 4.5 in local library
//    (was 3.0) for 50% more reach per step.
//
//  Stomp profile on each swinging leg:
//    30% fast rise → 40% hold at peak → 30% fast drop
//    Foot comes straight DOWN to dig into the surface.
//
//  ---- VARIABLES TO TUNE ----
//
//  BACKWARD_SPEED   amplitude scale — bigger = longer steps
//                   2.0 = moderate, 3.0 = full reach
//
//  LIFT_REAR        rear leg lift height  (legs 1, 6) — must clear 30° slope
//  LIFT_MID         middle leg lift height (legs 2, 5)
//  LIFT_FRONT       front leg lift height  (legs 3, 4) — small reposition
//
//  GAIT_PERIOD      ms per full 3-phase cycle
//                   2500 = deliberate, 3500 = very slow
//                   (slower → more time for each foot to dig in)
//
//  PITCH_NOSE_UP    degrees nose-up tilt (presses rear legs in)
//                   8 = recommended, 12 = aggressive
//
//  RUN_TIME_MS      how long to run before stopping
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ==========================================
// TUNE THESE
// ==========================================
// Leg position reference (Y coordinate):
//   Rear  legs = 1 (right) & 6 (left)  — Y = -5.65  — step first, lift highest
//   Middle legs = 2 (right) & 5 (left) — Y =  0
//   Front legs = 3 (right) & 4 (left)  — Y = +5.65  — step last, minimal lift
//
// Robot starts tilted ~30° (nose down in crater).
// Rear legs must clear slope surface rising at sin(30°)=0.5 per unit of
// backward reach.  At full 4.5 cm amplitude → slope rises ~4.5 cm under
// the foot during the backward sweep.
//
// BODY LEAN direction: POSITIVE pitch = nose DOWN = body leans FORWARD into slope.
//   Front legs anchor into the slope. Rear legs get more upward reach.
//   (Library convention: negative pitch = nose UP, positive = nose DOWN)

const float    BACKWARD_SPEED = 3.0f;   // amplitude — 3.0 = max library reach
const float    LIFT_REAR      = 8.0f;   // rear legs (1, 6) — near physical max
const float    LIFT_MID       = 5.0f;   // middle legs (2, 5)
const float    LIFT_FRONT     = 3.0f;   // front legs (3, 4) — small reposition only
const int      GAIT_PERIOD    = 3000;   // ms per full 3-phase cycle
const float    BODY_HEIGHT    = 1.5f;   // confirmed sweet spot — all 6 legs touch; taller = more swing range
const float    LEAN_FORWARD   = 8.0f;   // positive = nose DOWN = lean INTO slope (correct for backward climb)
const uint32_t RUN_TIME_MS    = 30000;  // total run time (ms)

// ==========================================
// SETUP
// ==========================================
void setup() {
  delay(10000);  // safety — disconnect USB, place robot in crater

  minihexa.begin();
  delay(2000);   // servos stand up

  // Green LED = running (distinguishes from crater_reverse blue)
  uint8_t green[3] = {0, 200, 0};
  minihexa.sensor.set_ultrasound_rgb(0, green, green);

  // Enable caterpillar gait: rear first, then mid, then front
  minihexa.set_caterpillar(true, LIFT_REAR, LIFT_MID, LIFT_FRONT);

  // vy negative = walk backward
  // LEAN_FORWARD positive = nose DOWN = body leans into slope
  //   → anchors front legs, gives rear legs more upward range
  Velocity_t vel = {0.0f, -BACKWARD_SPEED, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, BODY_HEIGHT};
  Euler_t    att = {LEAN_FORWARD, 0.0f, 0.0f};

  // Continuous gait — no STOP/REST stutter between cycles
  minihexa.move(&vel, &pos, &att, GAIT_PERIOD, -1);
  delay(RUN_TIME_MS);

  // Stop
  minihexa.set_caterpillar(false);
  Velocity_t vel_stop = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel_stop, &pos, &att, 800);
  delay(1200);

  // White LED = done
  uint8_t white[3] = {200, 200, 200};
  minihexa.sensor.set_ultrasound_rgb(0, white, white);
}

void loop() {}
