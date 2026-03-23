// ============================================================
//  CRATER MISSION — Score 55 points (enter/exit + full lap)
//
//  Strategy: SIDE STRADDLE ORBIT
//  - Robot parks at crater rim with RIGHT side over the edge
//  - 3 left legs stay on flat ground (anchored)
//  - 3 right legs reach into crater past the 3" crater line
//  - Robot walks FORWARD along the rim (arc_left to follow curve)
//  - Body ROLL tilts right side down for scoring
//
//  Crater geometry (from IEEE SoutheastCon 2026 ruleset):
//  - 2 foot diameter bowl (1 foot = 12" radius)
//  - Crater line: 3 inches from the lip
//  - 3D printed grey (smooth surface)
//  - Antenna #3 at the bottom center
//
//  Scoring:
//  - 20 pts: enter crater (feet touch crater line) AND exit
//  - 35 pts: full lap around crater (feet below line throughout)
//
//  Euler_t = {pitch, roll, yaw}
//  Velocity_t = {vx, vy, omega}   vy>0 = forward
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ============================================================
//  TUNABLES — adjust these on the real robot
// ============================================================

// --- Approach ---
const float  APPROACH_SPEED   = 2.0f;   // forward speed to reach crater
const float  APPROACH_HEIGHT  = 1.5f;   // normal walking height
const int    APPROACH_PERIOD  = 800;    // gait cycle ms
const int    APPROACH_CYCLES  = 8;      // cycles to reach crater edge (tune!)

// --- Side straddle orbit ---
const float  ROLL_ANGLE       = 0.25f;  // body roll in radians (~15 deg)
                                         // positive = tilt RIGHT side down (test & flip sign if wrong)
const float  ORBIT_HEIGHT     = 1.2f;   // low body for stability
const float  ORBIT_SPEED      = 1.5f;   // forward speed along rim
const float  ORBIT_OMEGA      = 0.043f; // angular velocity for orbit (speed/radius)
                                         // 1.5 / 35.0 ≈ 0.043
const int    ORBIT_PERIOD     = 1000;   // ms per gait cycle
const int    ORBIT_CYCLES     = 55;     // full lap (~223cm circumference)

// --- Turn ---
const float  TURN_SPEED       = 1.5f;
const float  TURN_HEIGHT      = 1.5f;
const int    TURN_PERIOD      = 800;
const int    TURN_90_CYCLES   = 6;      // cycles for ~90 degree turn (tune!)

// --- Front dip (backup) ---
const float  DIP_PITCH        = 0.35f;  // ~20 degrees forward lean
const float  DIP_HEIGHT       = 0.8f;   // low body so front legs reach down

// --- Exit ---
const int    EXIT_CYCLES      = 6;

// ============================================================
//  LED HELPERS
// ============================================================
void set_led(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t c[3] = {r, g, b};
  minihexa.sensor.set_ultrasound_rgb(0, c, c);
}

// ============================================================
//  MOVEMENT HELPERS
// ============================================================

// Stand still at given height for duration
void stand(float height, uint32_t ms) {
  Velocity_t vel = {0.0f, 0.0f, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, height};
  Euler_t    att = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, 800);
  delay(ms);
}

// Walk forward
void go_forward(float speed, float height, int period, int cycles) {
  Velocity_t vel = {0.0f, speed, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, height};
  Euler_t    att = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// Walk backward
void go_backward(float speed, float height, int period, int cycles) {
  Velocity_t vel = {0.0f, -speed, 0.0f};
  Vector_t   pos = {0.0f,  0.0f, height};
  Euler_t    att = {0.0f,  0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// In-place left turn (CCW)
void turn_left(float speed, float height, int period, int cycles) {
  Velocity_t vel = {0.001f, 0.001f, speed};
  Vector_t   pos = {0.0f,   0.0f,   height};
  Euler_t    att = {0.0f,   0.0f,   0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// In-place right turn (CW)
void turn_right(float speed, float height, int period, int cycles) {
  Velocity_t vel = {0.001f, 0.001f, -speed};
  Vector_t   pos = {0.0f,   0.0f,    height};
  Euler_t    att = {0.0f,   0.0f,    0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// ============================================================
//  STRATEGY 1: SIDE STRADDLE ORBIT (PRIMARY)
//
//  The robot is already at the crater edge, facing so that
//  its RIGHT side is over the crater.
//
//  1. Apply body roll to tilt right side down
//  2. Walk forward with left-arc to follow the circular rim
//  3. Right-side feet dip below the 3" crater line
//  4. Left-side feet stay on flat ground (anchored)
//  5. After full lap, remove roll and exit
// ============================================================
void side_straddle_orbit() {
  set_led(0, 200, 0);  // GREEN — orbit active

  // Phase 1: Tilt into position (stand still, apply roll)
  // This scores the "enter" — right feet drop below crater line
  {
    Velocity_t vel = {0.0f, 0.0f, 0.0f};
    Vector_t   pos = {0.0f, 0.0f, ORBIT_HEIGHT};
    Euler_t    att = {0.0f, ROLL_ANGLE, 0.0f};
    minihexa.move(&vel, &pos, &att, 800);
    delay(2000);  // hold 2s to let body settle and score entry
  }

  set_led(0, 0, 200);  // BLUE — orbiting

  // Phase 2: Orbit — walk forward with right-arc curvature
  // Forward (vy) + negative omega = curves RIGHT = follows crater rim clockwise
  // The orbit center is to the robot's RIGHT = crater center
  {
    Velocity_t vel = {0.0f, ORBIT_SPEED, -ORBIT_OMEGA};
    Vector_t   pos = {0.0f, 0.0f, ORBIT_HEIGHT};
    Euler_t    att = {0.0f, ROLL_ANGLE, 0.0f};
    minihexa.move(&vel, &pos, &att, ORBIT_PERIOD, ORBIT_CYCLES);
    delay((uint32_t)ORBIT_PERIOD * ORBIT_CYCLES + 500);
  }

  set_led(200, 200, 0);  // YELLOW — orbit complete, exiting

  // Phase 3: Remove roll, stand upright
  {
    Velocity_t vel = {0.0f, 0.0f, 0.0f};
    Vector_t   pos = {0.0f, 0.0f, APPROACH_HEIGHT};
    Euler_t    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, 800);
    delay(1500);
  }
}

// ============================================================
//  STRATEGY 2: FRONT DIP (BACKUP — enter/exit only, 20 pts)
//
//  Robot approaches head-on, pitches forward so front legs
//  reach down past the crater line, then backs away.
// ============================================================
void front_dip() {
  set_led(200, 0, 200);  // MAGENTA — front dip active

  // Phase 1: Pitch forward and lower body
  // Positive pitch = nose DOWN (library convention)
  // Front legs (3,4) naturally extend lower
  {
    Velocity_t vel = {0.0f, 0.0f, 0.0f};
    Vector_t   pos = {0.0f, 0.0f, DIP_HEIGHT};
    Euler_t    att = {DIP_PITCH, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, 800);
    delay(3000);  // hold 3s for scoring
  }

  set_led(0, 200, 0);  // GREEN — scored

  // Phase 2: Return to neutral
  {
    Velocity_t vel = {0.0f, 0.0f, 0.0f};
    Vector_t   pos = {0.0f, 0.0f, APPROACH_HEIGHT};
    Euler_t    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, 800);
    delay(1500);
  }
}

// ============================================================
//  STRATEGY 3: CRANE DIP (FALLBACK — direct servo control)
//
//  Uses multi_servo_control() for precise keyframe animation.
//  Anchor rear/mid legs, front legs (3,4) reach down into crater.
//
//  Servo mapping:
//    Leg3 (front-R): servos 7(A), 8(B), 9(C)
//    Leg4 (front-L): servos 10(A), 11(B), 12(C)
//    B-servo: right goes BELOW 1500 to reach down
//             left goes ABOVE 1500 to reach down
//
//  Home B values: right=1409, left=1590
//  We add DIP offset to reach further down.
// ============================================================

// B-servo deviation for front leg dip (start conservative)
const int FRONT_B_DIP = 400;  // ~56 degrees; increase to 500 (~70 deg) for more reach

void crane_dip() {
  set_led(200, 100, 0);  // ORANGE — crane dip

  // All 18 servos: home position
  // Servo order: 1A,1B,1C, 2A,2B,2C, 3A,3B,3C, 4A,4B,4C, 5A,5B,5C, 6A,6B,6C
  uint16_t home[18] = {
    1500, 1409, 1356,  // Leg1 rear-R
    1500, 1409, 1357,  // Leg2 mid-R
    1500, 1409, 1357,  // Leg3 front-R
    1500, 1590, 1642,  // Leg4 front-L
    1500, 1590, 1642,  // Leg5 mid-L
    1500, 1590, 1642   // Leg6 rear-L
  };

  ServoArg_t servos[18];

  // --- Frame 1: HOME (prepare) ---
  for (int i = 0; i < 18; i++) {
    servos[i].id = i + 1;
    servos[i].duty = home[i];
  }
  minihexa.multi_servo_control(servos, 18, 800);
  delay(1000);

  // --- Frame 2: Front legs reach DOWN into crater ---
  // Leg3 (front-R): B goes BELOW home by FRONT_B_DIP (reach down)
  // Leg4 (front-L): B goes ABOVE home by FRONT_B_DIP (reach down)
  // Also extend C-servo slightly for outward reach
  for (int i = 0; i < 18; i++) {
    servos[i].id = i + 1;
    servos[i].duty = home[i];
  }
  // Leg3 front-R: servo 8(B) = home_B - DIP
  servos[7].duty = 1409 - FRONT_B_DIP;   // B lower = reach down for right leg
  servos[8].duty = 1357 + 150;            // C extend outward slightly

  // Leg4 front-L: servo 11(B) = home_B + DIP
  servos[10].duty = 1590 + FRONT_B_DIP;  // B higher = reach down for left leg
  servos[11].duty = 1642 - 150;           // C extend outward slightly

  minihexa.multi_servo_control(servos, 18, 800);
  delay(1000);

  set_led(0, 200, 0);  // GREEN — in position, scoring

  // --- Frame 3: HOLD for scoring ---
  delay(2000);

  // --- Frame 4: Retract to home ---
  for (int i = 0; i < 18; i++) {
    servos[i].id = i + 1;
    servos[i].duty = home[i];
  }
  minihexa.multi_servo_control(servos, 18, 800);
  delay(1000);

  set_led(200, 200, 0);  // YELLOW — retracted
}

// ============================================================
//  MISSION PHASES
// ============================================================

void approach_crater() {
  set_led(200, 200, 200);  // WHITE — approaching
  go_forward(APPROACH_SPEED, APPROACH_HEIGHT, APPROACH_PERIOD, APPROACH_CYCLES);
  stand(APPROACH_HEIGHT, 500);
}

void align_for_straddle() {
  // Turn 90 degrees LEFT so RIGHT side faces the crater
  // (assuming robot approached crater from the left/south side)
  // Tune TURN_90_CYCLES for exactly 90 degrees
  turn_left(TURN_SPEED, TURN_HEIGHT, TURN_PERIOD, TURN_90_CYCLES);
  stand(TURN_HEIGHT, 500);
}

void exit_crater_area() {
  set_led(200, 0, 0);  // RED — exiting

  // Turn LEFT 90 to face back the way we came
  // (after the lap we face the same direction as post-initial-left-turn)
  turn_left(TURN_SPEED, TURN_HEIGHT, TURN_PERIOD, TURN_90_CYCLES);
  stand(APPROACH_HEIGHT, 500);

  // Walk away from crater
  go_forward(APPROACH_SPEED, APPROACH_HEIGHT, APPROACH_PERIOD, EXIT_CYCLES);
  stand(APPROACH_HEIGHT, 500);
}

// ============================================================
//  MAIN MISSION
// ============================================================
void run_mission() {

  // ---- STRATEGY SELECT ----
  // Uncomment ONE strategy block:

  // // === STRATEGY 1: Side Straddle Orbit (55 pts) ===
  // approach_crater();
  // align_for_straddle();
  // side_straddle_orbit();
  // exit_crater_area();

  //=== STRATEGY 2: Front Dip Only (20 pts) ===
  // approach_crater();
  // front_dip();
  // go_backward(APPROACH_SPEED, APPROACH_HEIGHT, APPROACH_PERIOD, EXIT_CYCLES);

  // === STRATEGY 3: Crane Dip Only (20 pts) ===
  approach_crater();
  crane_dip();
  go_backward(APPROACH_SPEED, APPROACH_HEIGHT, APPROACH_PERIOD, EXIT_CYCLES);

  // Done — rest
  set_led(0, 200, 0);  // GREEN — mission complete
  stand(1.0f, 5000);
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
  delay(10000);       // 10s safety — disconnect USB before this runs!
  minihexa.begin();
  delay(2000);        // let servos settle

  run_mission();      // run once then stop
}

void loop() {}
