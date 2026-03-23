// ============================================================
//  CRATER ESCAPE — All-Strategies Sequencer  (v3)
//
//  Place robot IN the crater and power on (10s delay).
//  Runs ALL 8 strategies one after another, every loop.
//  NO early exit — each strategy always runs its full time.
//  After each attempt the robot slides back to the bottom
//  naturally; a settle pause lets it settle before the next.
//
//  Strategy list (LED colour while running):
//    1 MAGENTA  Wave gait, 8° pitch
//    2 CYAN     Wave gait, 15° aggressive pitch
//    3 BLUE     Wave gait, extra slow + big stomp
//    4 YELLOW   Slow tripod (baseline comparison)
//    5 RED      Power surge  (crouch → explode)
//    6 ORANGE   Avoid-chaos  (built-in avoid() 15 s)
//    7 PURPLE   Rocking inchworm
//    8 DIM WHT  Diagonal crab (45° left / right)
//
//  Between strategies:
//    Slow green pulse = settling / waiting for next strategy
//
//  After a full loop of all 8:
//    Fast WHITE flash = at least one run looked promising
//    (just indicates end of loop, then loops again)
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ==========================================
// TIMING — tune these between runs
// ==========================================
const uint32_t ESCAPE_TIME_MS  = 18000; // how long each strategy runs
const uint32_t SETTLE_TIME_MS  =  5000; // pause between strategies

// ==========================================
// MOVEMENT PARAMETERS
// ==========================================
const float BODY_HEIGHT     = 1.5f;   // z = sweet spot, all 6 legs touch
const float PITCH_NORMAL    = 8.0f;   // degrees forward lean
const float PITCH_HARD      = 15.0f;
const float STOMP_NORMAL    = 5.0f;   // wave gait leg lift (cm-units)
const float STOMP_BIG       = 7.0f;
const float FORWARD_SPEED   = 0.8f;
const int   WAVE_PERIOD_STD  = 2000;  // ms per gait cycle
const int   WAVE_PERIOD_SLOW = 2800;
const int   TRIPOD_PERIOD    = 1200;

// ==========================================
// HELPERS
// ==========================================

void set_led(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t c[3] = {r, g, b};
  minihexa.sensor.set_ultrasound_rgb(0, c, c);
}

void stop_and_hold() {
  Velocity_t vel = {0.0f, 0.0f, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, BODY_HEIGHT};
  Euler_t    att = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, 600);
  delay(1200);
}

// Green slow pulse — settling between strategies
void settle_pause() {
  stop_and_hold();
  uint32_t end = millis() + SETTLE_TIME_MS;
  while (millis() < end) {
    set_led(0, 120, 0); delay(600);
    set_led(0,   0, 0); delay(600);
  }
}

// ==========================================
// STRATEGY 1 — Wave gait, normal pitch  (MAGENTA)
// ==========================================
void strategy1() {
  set_led(180, 0, 180);

  minihexa.set_wave_gait(true, STOMP_NORMAL);
  Velocity_t vel = {0.0f, FORWARD_SPEED, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, BODY_HEIGHT};
  Euler_t    att = {PITCH_NORMAL, 0.0f, 0.0f};

  uint32_t end = millis() + ESCAPE_TIME_MS;
  while (millis() < end) {
    minihexa.move(&vel, &pos, &att, WAVE_PERIOD_STD, 1);
    delay(WAVE_PERIOD_STD + 150);
  }

  minihexa.set_wave_gait(false);
}

// ==========================================
// STRATEGY 2 — Wave gait, hard lean  (CYAN)
// Body pitched aggressively forward into the slope.
// ==========================================
void strategy2() {
  set_led(0, 180, 180);

  minihexa.set_wave_gait(true, STOMP_NORMAL);
  Velocity_t vel = {0.0f, FORWARD_SPEED, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, BODY_HEIGHT};
  Euler_t    att = {PITCH_HARD, 0.0f, 0.0f};

  uint32_t end = millis() + ESCAPE_TIME_MS;
  while (millis() < end) {
    minihexa.move(&vel, &pos, &att, WAVE_PERIOD_STD, 1);
    delay(WAVE_PERIOD_STD + 150);
  }

  minihexa.set_wave_gait(false);
}

// ==========================================
// STRATEGY 3 — Wave gait, very slow + big stomp  (BLUE)
// Maximum traction: 2.8 s cycle, 7 cm lift.
// ==========================================
void strategy3() {
  set_led(0, 0, 220);

  minihexa.set_wave_gait(true, STOMP_BIG);
  Velocity_t vel = {0.0f, FORWARD_SPEED, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, BODY_HEIGHT};
  Euler_t    att = {PITCH_NORMAL, 0.0f, 0.0f};

  uint32_t end = millis() + ESCAPE_TIME_MS;
  while (millis() < end) {
    minihexa.move(&vel, &pos, &att, WAVE_PERIOD_SLOW, 1);
    delay(WAVE_PERIOD_SLOW + 150);
  }

  minihexa.set_wave_gait(false);
}

// ==========================================
// STRATEGY 4 — Slow tripod baseline  (YELLOW)
// Standard tripod gait for comparison.
// ==========================================
void strategy4() {
  set_led(200, 200, 0);

  minihexa.set_wave_gait(false);
  Velocity_t vel = {0.0f, FORWARD_SPEED, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, BODY_HEIGHT};
  Euler_t    att = {PITCH_NORMAL, 0.0f, 0.0f};

  uint32_t end = millis() + ESCAPE_TIME_MS;
  while (millis() < end) {
    minihexa.move(&vel, &pos, &att, TRIPOD_PERIOD, 1);
    delay(TRIPOD_PERIOD + 150);
  }
}

// ==========================================
// STRATEGY 5 — Power Surge  (RED)
// Crouch body down, then explode forward at max speed.
// Repeated rapidly. Closest thing to a jump the
// servos allow (400 ms minimum move_time).
// ==========================================
void strategy5() {
  set_led(220, 0, 0);

  Velocity_t vel_stop  = {0.0f, 0.0f, 0.0f};
  Velocity_t vel_surge = {0.0f, 3.0f, 0.0f};  // max forward speed
  Vector_t   pos_low   = {0.0f, 0.0f, -2.0f}; // crouch
  Vector_t   pos_high  = {0.0f, 0.0f,  3.5f}; // full extend
  Euler_t    att       = {PITCH_HARD, 0.0f, 0.0f};

  uint32_t end = millis() + ESCAPE_TIME_MS;
  while (millis() < end) {
    // Crouch
    minihexa.move(&vel_stop, &pos_low, &att, 500, 1);
    delay(700);
    // Explode
    minihexa.move(&vel_surge, &pos_high, &att, 400, 1);
    delay(600);
  }
}

// ==========================================
// STRATEGY 6 — Avoid Chaos  (ORANGE)
// Runs the built-in obstacle avoidance state machine.
// In the crater the wall = obstacle → random turning
// and reversing.  Chaotic but occasionally finds the exit.
// ==========================================
void strategy6() {
  set_led(220, 100, 0);

  uint32_t end = millis() + 15000; // fixed 15 s regardless of ESCAPE_TIME_MS
  while (millis() < end) {
    uint16_t dis = minihexa.sensor.get_distance();
    minihexa.avoid(dis);
    delay(50);
  }

  stop_and_hold();
}

// ==========================================
// STRATEGY 7 — Rocking Inchworm  (PURPLE)
// Alternates between steep (12°) and shallow (3°) lean
// every 2 steps while wave-gait walking.
// Rocking the CoM rhythmically like rocking a car out of mud.
// ==========================================
void strategy7() {
  set_led(100, 0, 180);

  minihexa.set_wave_gait(true, STOMP_NORMAL);
  Velocity_t vel = {0.0f, FORWARD_SPEED, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, BODY_HEIGHT};

  uint32_t end   = millis() + ESCAPE_TIME_MS;
  int      phase = 0;

  while (millis() < end) {
    float pitch = (phase % 2 == 0) ? 12.0f : 3.0f;
    Euler_t att = {pitch, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, WAVE_PERIOD_STD, 2);
    delay(WAVE_PERIOD_STD * 2 + 200);
    phase++;
  }

  minihexa.set_wave_gait(false);
}

// ==========================================
// STRATEGY 8 — Diagonal Crab  (DIM WHITE)
// First half: 45° right diagonal.
// Second half: 45° left diagonal.
// Angled approach contacts different parts of each
// foot on the crater surface — sometimes finds more grip.
// ==========================================
void strategy8() {
  set_led(100, 100, 100);

  minihexa.set_wave_gait(true, STOMP_NORMAL);
  Vector_t pos = {0.0f, 0.0f, BODY_HEIGHT};
  Euler_t  att = {PITCH_NORMAL, 0.0f, 0.0f};
  uint32_t half = ESCAPE_TIME_MS / 2;

  // Right diagonal
  Velocity_t vel_r = {1.0f, 1.0f, 0.0f};
  uint32_t end_r = millis() + half;
  while (millis() < end_r) {
    minihexa.move(&vel_r, &pos, &att, WAVE_PERIOD_STD, 1);
    delay(WAVE_PERIOD_STD + 150);
  }

  // Left diagonal
  Velocity_t vel_l = {-1.0f, 1.0f, 0.0f};
  uint32_t end_l = millis() + half;
  while (millis() < end_l) {
    minihexa.move(&vel_l, &pos, &att, WAVE_PERIOD_STD, 1);
    delay(WAVE_PERIOD_STD + 150);
  }

  minihexa.set_wave_gait(false);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  delay(10000);  // safety — disconnect USB, place robot in crater

  minihexa.begin();
  minihexa.board.imu_update(true);
  delay(3000);   // stand up + IMU stabilise

  // Run all 8 strategies forever, in sequence.
  // Every strategy always runs its full time — no early exit.
  while (true) {
    strategy1(); settle_pause();
    strategy2(); settle_pause();
    strategy3(); settle_pause();
    strategy4(); settle_pause();
    strategy5(); settle_pause();
    strategy6(); settle_pause();
    strategy7(); settle_pause();
    strategy8(); settle_pause();
  }
}

void loop() {}
