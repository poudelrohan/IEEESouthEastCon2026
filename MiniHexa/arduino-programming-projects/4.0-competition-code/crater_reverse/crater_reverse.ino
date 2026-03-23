// ============================================================
//  CRATER REVERSE CLIMB
//
//  Walks BACKWARD up the crater wall using the normal tripod
//  gait with per-leg lift heights tuned for backward climbing.
//
//  Why backward?
//    Rear legs are longer-reaching and have more leverage.
//    Backing up the slope keeps the body low and stable.
//
//  Why different lift heights per leg position?
//    Rear legs  (1, 6): highest lift — they step back and grip the slope.
//    Middle legs (2, 5): medium lift — share the load.
//    Front legs  (3, 4): lowest lift — trailing, just reposition.
//
//  Body lean:
//    Positive pitch = nose DOWN = body leans FORWARD into the slope.
//    Front legs anchor. Rear legs get more vertical range to clear slope.
//
//  ---- VARIABLES TO TUNE ----
//
//  BACKWARD_SPEED   how fast it walks backward
//                   1.0 = slow, 2.0 = good start, 3.0 = max reach
//
//  LIFT_REAR        lift height for rear legs (1, 6) — near physical max
//  LIFT_MID         lift height for middle legs (2, 5)
//  LIFT_FRONT       lift height for front legs (3, 4) — trailing
//
//  GAIT_PERIOD      time for one full tripod cycle (ms)
//                   2000 = deliberate,  3000 = very slow
//
//  BODY_HEIGHT      position.z — 1.5 is sweet spot where all 6 legs touch
//
//  LEAN_FORWARD     positive = nose DOWN = lean into slope (correct direction)
//                   0 = flat, 8 = moderate lean, 15 = aggressive
//
//  RUN_TIME_MS      total run time before stopping
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ==========================================
// TUNE THESE
// ==========================================
// Leg position reference (Y coordinate):
//   Rear  legs = 1 (right) & 6 (left) — Y = -5.65 — lift highest
//   Middle legs = 2 (right) & 5 (left) — Y = 0
//   Front legs  = 3 (right) & 4 (left) — Y = +5.65 — trailing, low lift
//
// BODY LEAN: positive pitch = nose DOWN = lean INTO slope.
//   Anchors front legs, gives rear legs more upward reach.
//   (Library convention: negative = nose UP, positive = nose DOWN)

const float    BACKWARD_SPEED = 3.0f;   // walking speed (backward)
const float    LIFT_REAR      = 8.0f;   // rear legs (1, 6) — near physical max
const float    LIFT_MID       = 5.0f;   // middle legs (2, 5)
const float    LIFT_FRONT     = 3.0f;   // front legs (3, 4) — trailing
const int      GAIT_PERIOD    = 2000;   // ms per tripod cycle
const float    BODY_HEIGHT    = 1.5f;   // sweet spot — all 6 legs touch ground
const float    LEAN_FORWARD   = 8.0f;   // positive = nose DOWN = lean into slope
const uint32_t RUN_TIME_MS    = 30000;  // total run time (ms)

// ==========================================
// SETUP
// ==========================================
void setup() {
  delay(10000);  // safety — disconnect USB, place robot in crater

  minihexa.begin();
  delay(2000);   // servos stand up

  // Blue LED = running
  uint8_t blue[3] = {0, 0, 200};
  minihexa.sensor.set_ultrasound_rgb(0, blue, blue);

  // Set per-leg lift heights for tripod gait:
  //   front legs low (trailing), rear legs high (doing the work)
  minihexa.set_leg_lifts(LIFT_FRONT, LIFT_MID, LIFT_REAR);

  // vy negative = walk backward
  // LEAN_FORWARD positive = nose DOWN = body leans into slope
  //   → front legs anchor to slope, rear legs get more upward reach
  Velocity_t vel = {0.0f, -BACKWARD_SPEED, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, BODY_HEIGHT};
  Euler_t    att = {LEAN_FORWARD, 0.0f, 0.0f};

  // Walk backward continuously for RUN_TIME_MS.
  // step_num = -1  →  gait runs forever (no STOP/REST between cycles).
  // A single delay() holds here while the FreeRTOS timer ticks the servos.
  // This avoids the stutter that step_num=1 causes (leg reset every cycle).
  minihexa.move(&vel, &pos, &att, GAIT_PERIOD, -1);
  delay(RUN_TIME_MS);

  // Stop and stand still
  Velocity_t vel_stop = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel_stop, &pos, &att, 800);
  delay(1200);

  // White LED = done
  uint8_t white[3] = {200, 200, 200};
  minihexa.sensor.set_ultrasound_rgb(0, white, white);
}

void loop() {}
