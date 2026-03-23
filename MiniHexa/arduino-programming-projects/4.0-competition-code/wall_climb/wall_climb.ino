// ============================================================
//  WALL CLIMB — FORWARD SLOPE ASCENT
//
//  8 strategies for climbing a ~30° slope going FORWARD.
//  Each strategy runs for STRATEGY_TIME ms, then the next
//  starts automatically. Cycles forever so you can observe
//  which one works best.
//
//  LED colour identifies the active strategy at a glance.
//
//  Physical leg positions (Y coordinate):
//    Rear  legs = 1 (right) & 6 (left)  — Y = -5.65
//    Middle legs = 2 (right) & 5 (left) — Y =  0
//    Front legs = 3 (right) & 4 (left)  — Y = +5.65
//
//  Going FORWARD up a slope:
//    Front legs (3, 4) reach UP the slope → lift highest
//    Rear legs (1, 6) push from behind   → lift lowest
//    Body leans FORWARD (positive pitch = nose down)
//      to keep centre of mass over the slope.
//
//  ---- STRATEGIES ----
//
//  1  GREEN     Front-claw caterpillar
//               Front pair reaches high, plants on slope, then
//               mid follows, rear pushes. 4 legs always grounded.
//               Most stable, best traction.
//
//  2  CYAN      Metachronal crawl (1 leg at a time)
//               Only 1 leg moves at once, 5 always grounded.
//               Maximum stability — inspired by real insects on
//               steep/slippery slopes. Slowest but most sure-footed.
//
//  3  BLUE      Power tripod
//               Standard tripod with max front lift, fast cadence.
//               Momentum-based — trades stability for speed.
//
//  4  YELLOW    Wave gait diagonal stomp
//               Diagonal pair stepping (wave gait) with high stomp.
//               Different grouping from caterpillar — pairs legs
//               across diagonals for balanced weight distribution.
//
//  5  MAGENTA   Zigzag switchback
//               Walk diagonally across the slope face, alternating
//               left/right every 5 s. Reduces effective slope angle
//               from 30° to ~21° (cos 45°).
//
//  6  RED       Low crawl
//               Body LOW, very slow caterpillar, aggressive forward
//               lean. Maximum ground contact, max traction, slowest.
//
//  7  ORANGE    Stomp-pause-surge
//               Walk 2s with high front lift, PAUSE 1s to let feet
//               grip, then surge at 130% speed. Prevents sliding
//               by giving the robot grip time between pushes.
//
//  8  WHITE     Body surge
//               Tripod walk with oscillating body pitch and speed.
//               Creates momentum pulses — leans hard forward and
//               pushes fast, then relaxes. Mimics insect body
//               undulation during slope climbing.
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ==========================================
// GLOBAL TUNABLES
// ==========================================
const float    CLIMB_SPEED     = 2.5f;   // base forward speed
const float    BODY_HEIGHT     = 2.5f;   // sweet spot — all 6 legs touch
const float    LEAN_FORWARD    = 0.0f;  // positive = nose DOWN = lean into slope
const uint32_t STRATEGY_TIME   = 20000;  // ms per strategy
const uint32_t PAUSE_TIME      = 3000;   // ms pause between strategies

// ==========================================
// HELPERS
// ==========================================
void set_led(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t c[3] = {r, g, b};
  minihexa.sensor.set_ultrasound_rgb(0, c, c);
}

// Brief stop between strategies — white blink marks the transition
void pause_between() {
  Velocity_t stop = {0, 0, 0};
  Vector_t   pos  = {0, 0, BODY_HEIGHT};
  Euler_t    flat = {0, 0, 0};
  minihexa.move(&stop, &pos, &flat, 800);

  for (int i = 0; i < 2; i++) {
    set_led(200, 200, 200);
    delay(300);
    set_led(0, 0, 0);
    delay(300);
  }
  delay(PAUSE_TIME - 1200);
}

// ==========================================
// STRATEGY 1 — FRONT-CLAW CATERPILLAR
// ==========================================
// Front pair (3,4) step first — reach HIGH up the slope.
// Mid (2,5) follow.  Rear (1,6) push last.
// 4 legs grounded at all times → maximum traction.
// Lift profile: 15% rise / 70% hold / 15% drop (slope-optimised).
void strategy_caterpillar() {
  set_led(0, 200, 0);  // GREEN

  // front_first=true: front lifts 8 (highest), rear lifts 3 (push only)
  minihexa.set_caterpillar(true, 3.0f, 5.0f, 8.0f, true);

  Velocity_t vel = {0, CLIMB_SPEED, 0};
  Vector_t   pos = {0, 0, BODY_HEIGHT};
  Euler_t    att = {LEAN_FORWARD, 0, 0};

  minihexa.move(&vel, &pos, &att, 3000, -1);
  delay(STRATEGY_TIME);

  minihexa.set_caterpillar(false);
}

// ==========================================
// STRATEGY 2 — METACHRONAL CRAWL
// ==========================================
// 1 leg at a time, 5 always grounded.
// Maximum stability — real insects switch to this gait
// on steep/slippery slopes.
// 6-phase cycle: front-R → front-L → mid-R → mid-L → rear-R → rear-L
// Longer period (4.5s) because each leg only gets 1/6 of the cycle.
void strategy_metachronal() {
  set_led(0, 200, 200);  // CYAN

  // front_first=true: front legs step first for forward climb
  minihexa.set_metachronal(true, 3.0f, 5.0f, 8.0f, true);

  Velocity_t vel = {0, CLIMB_SPEED * 0.6f, 0};  // slow — stability over speed
  Vector_t   pos = {0, 0, BODY_HEIGHT};
  Euler_t    att = {LEAN_FORWARD, 0, 0};

  minihexa.move(&vel, &pos, &att, 4500, -1);  // 4.5s per full 6-phase cycle
  delay(STRATEGY_TIME);

  minihexa.set_metachronal(false);
}

// ==========================================
// STRATEGY 3 — POWER TRIPOD
// ==========================================
// Standard tripod gait with per-leg lift heights tuned
// for forward climb: front legs high, rear low.
// Faster cadence (1500ms) → uses momentum to push uphill.
void strategy_power_tripod() {
  set_led(0, 0, 200);  // BLUE

  // Front legs (3,4)=8, Mid (2,5)=5, Rear (1,6)=3
  minihexa.set_leg_lifts(3.0f, 5.0f, 8.0f);

  Velocity_t vel = {0, CLIMB_SPEED * 1.2f, 0};
  Vector_t   pos = {0, 0, BODY_HEIGHT};
  Euler_t    att = {LEAN_FORWARD * 0.8f, 0, 0};  // less lean, more speed

  minihexa.move(&vel, &pos, &att, 1500, -1);
  delay(STRATEGY_TIME);

  minihexa.set_leg_lifts(3.0f, 3.0f, 3.0f);  // reset
}

// ==========================================
// STRATEGY 4 — WAVE GAIT DIAGONAL STOMP
// ==========================================
// Uses the wave/ripple gait (diagonal pair stepping):
//   Phase 0: leg1 (rear-R) + leg4 (front-L)   — diagonal
//   Phase 1: leg3 (front-R) + leg6 (rear-L)   — diagonal
//   Phase 2: leg2 (mid-R)  + leg5 (mid-L)     — sides
// Different grouping from caterpillar (which uses parallel pairs).
// Each diagonal pair provides balanced left-right support.
// High stomp height for slope surface clearance.
void strategy_wave() {
  set_led(200, 200, 0);  // YELLOW

  minihexa.set_wave_gait(true, 7.0f);

  Velocity_t vel = {0, CLIMB_SPEED * 0.8f, 0};
  Vector_t   pos = {0, 0, BODY_HEIGHT};
  Euler_t    att = {LEAN_FORWARD, 0, 0};

  minihexa.move(&vel, &pos, &att, 2500, -1);
  delay(STRATEGY_TIME);

  minihexa.set_wave_gait(false);
}

// ==========================================
// STRATEGY 5 — ZIGZAG SWITCHBACK
// ==========================================
// Walk diagonally across the slope face, alternating
// left and right every 5 seconds.  Like switchbacks on
// a hiking trail — reduces effective slope angle.
// At 45° traverse angle on a 30° slope the effective
// incline drops to about 21°.
void strategy_zigzag() {
  set_led(200, 0, 200);  // MAGENTA

  minihexa.set_leg_lifts(3.0f, 5.0f, 7.0f);

  Vector_t pos = {0, 0, BODY_HEIGHT};
  Euler_t  att = {LEAN_FORWARD, 0, 0};

  uint32_t start = millis();
  bool go_right = true;

  while (millis() - start < STRATEGY_TIME) {
    float vx = go_right ? 1.8f : -1.8f;              // lateral component
    Velocity_t vel = {vx, CLIMB_SPEED * 0.7f, 0};    // reduced forward + lateral
    minihexa.move(&vel, &pos, &att, 2000, -1);
    delay(5000);                                       // 5 s in each direction
    go_right = !go_right;
  }

  minihexa.set_leg_lifts(3.0f, 3.0f, 3.0f);
}

// ==========================================
// STRATEGY 6 — LOW CRAWL
// ==========================================
// Body LOW to the ground for maximum ground contact.
// Very slow front-first caterpillar gait (4s per cycle).
// Aggressive forward lean (14°).
// Research: insects keep CoM low on steep slopes and
// switch to metachronal (sequential) gait for stability.
void strategy_low_crawl() {
  set_led(200, 0, 0);  // RED

  minihexa.set_caterpillar(true, 3.0f, 5.0f, 8.0f, true);

  Velocity_t vel = {0, CLIMB_SPEED * 0.5f, 0};       // slow
  Vector_t   pos = {0, 0, 0.8f};                      // body LOW
  Euler_t    att = {LEAN_FORWARD * 1.4f, 0, 0};       // aggressive lean (14°)

  minihexa.move(&vel, &pos, &att, 4000, -1);          // 4s per cycle — very deliberate
  delay(STRATEGY_TIME);

  minihexa.set_caterpillar(false);
}

// ==========================================
// STRATEGY 7 — STOMP-PAUSE-SURGE
// ==========================================
// Walk forward with high front lift for 2s,
// then PAUSE for 1s to let feet grip the surface,
// then SURGE at 130% speed for 2s.
// The pause prevents accumulated sliding by giving
// the rubber feet time to grip before the next push.
void strategy_stomp_pause() {
  set_led(200, 100, 0);  // ORANGE

  minihexa.set_leg_lifts(3.0f, 5.0f, 8.0f);

  Vector_t pos = {0, 0, BODY_HEIGHT};
  Euler_t  att = {LEAN_FORWARD, 0, 0};

  uint32_t start = millis();
  int phase = 0;

  while (millis() - start < STRATEGY_TIME) {
    switch (phase) {
      case 0: {
        // WALK: normal speed, high front lift
        Velocity_t vel = {0, CLIMB_SPEED, 0};
        minihexa.move(&vel, &pos, &att, 2000, -1);
        delay(2000);
        break;
      }
      case 1: {
        // PAUSE: stop and let feet grip
        Velocity_t vel = {0, 0, 0};
        minihexa.move(&vel, &pos, &att, 800);
        delay(1000);
        break;
      }
      case 2: {
        // SURGE: fast push forward
        Velocity_t vel = {0, CLIMB_SPEED * 1.3f, 0};
        Euler_t surge_att = {LEAN_FORWARD * 1.3f, 0, 0};  // lean harder
        minihexa.move(&vel, &pos, &surge_att, 1500, -1);
        delay(2000);
        break;
      }
    }
    phase = (phase + 1) % 3;
  }

  minihexa.set_leg_lifts(3.0f, 3.0f, 3.0f);
}

// ==========================================
// STRATEGY 8 — BODY SURGE
// ==========================================
// Walk forward with oscillating body lean and speed.
// Every 2s the body SURGES: leans hard forward (15°),
// shifts body CG forward, and pushes at 130% speed.
// Then RELAXES: eases pitch to 5°, shifts CG back,
// slows to 60% speed.
// Creates momentum pulses that help push the robot
// over the slope lip.  Inspired by insect body
// undulation during slope climbing.
void strategy_body_surge() {
  set_led(200, 200, 200);  // WHITE

  minihexa.set_leg_lifts(3.0f, 5.0f, 7.0f);

  uint32_t start = millis();
  bool surge = true;

  while (millis() - start < STRATEGY_TIME) {
    if (surge) {
      // SURGE: lean hard, push fast, shift body CG forward
      Velocity_t vel = {0, CLIMB_SPEED * 1.3f, 0};
      Vector_t   pos = {0, 0.5f, BODY_HEIGHT};          // body shifted forward
      Euler_t    att = {LEAN_FORWARD * 1.5f, 0, 0};     // 15° lean
      minihexa.move(&vel, &pos, &att, 1500, -1);
    } else {
      // RELAX: ease off, let body settle
      Velocity_t vel = {0, CLIMB_SPEED * 0.6f, 0};
      Vector_t   pos = {0, -0.3f, BODY_HEIGHT};          // body shifted back
      Euler_t    att = {LEAN_FORWARD * 0.5f, 0, 0};      // 5° lean
      minihexa.move(&vel, &pos, &att, 2000, -1);
    }
    delay(2000);
    surge = !surge;
  }

  minihexa.set_leg_lifts(3.0f, 3.0f, 3.0f);
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  delay(10000);  // safety — disconnect USB, place robot at base of slope

  minihexa.begin();
  delay(2000);

  // Cycle through all 8 strategies forever.
  // Robot will slide back between runs — that is expected.
  // Watch the LED colour to know which strategy is active.
  while (true) {
    strategy_caterpillar();   // 1 GREEN
    pause_between();

    strategy_metachronal();   // 2 CYAN
    pause_between();

    strategy_power_tripod();  // 3 BLUE
    pause_between();

    strategy_wave();          // 4 YELLOW
    pause_between();

    strategy_zigzag();        // 5 MAGENTA
    pause_between();

    strategy_low_crawl();     // 6 RED
    pause_between();

    strategy_stomp_pause();   // 7 ORANGE
    pause_between();

    strategy_body_surge();    // 8 WHITE
    pause_between();
  }
}

void loop() {}
