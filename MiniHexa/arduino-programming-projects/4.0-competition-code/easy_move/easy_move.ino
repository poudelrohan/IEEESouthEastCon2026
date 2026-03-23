// ============================================================
//  EASY MOVE — Simple motion helper library
//
//  All movements are functions.  To build a routine, just
//  call them in order inside run_mission().
//
//  ---- QUICK REFERENCE ----
//
//  go_forward  (speed, height, period, cycles)
//  go_backward (speed, height, period, cycles)
//  go_left     (speed, height, period, cycles)
//  go_right    (speed, height, period, cycles)
//  turn_left   (speed, height, period, cycles)   pure in-place rotation
//  turn_right  (speed, height, period, cycles)
//  arc_left    (radius, speed, height, period, cycles)   forward + curve
//  arc_right   (radius, speed, height, period, cycles)
//  stand       (height, ms)                       stand still at height
//  rest_flat   (ms)                               sit low and wait
//
//  ---- PARAMETER GUIDE ----
//
//  speed      — how fast  (0.5=slow, 2.0=normal, 3.5=max)
//  height     — body height z (0.5=low crouch, 1.5=normal, 2.5=tall)
//  period     — ms per gait cycle (600=fast steps, 1000=medium, 1500=slow)
//  cycles     — how many gait cycles (each ~one step pair)
//               1 cycle ≈ tiny shuffle
//               3 cycles ≈ half a body-length
//               6 cycles ≈ one body-length
//  radius     — arc radius for arc_left/arc_right (cm-ish units)
//               2.0=tight U-turn  5.0=gentle curve  10.0=wide arc
//
//  *** NEVER connect USB while robot is running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ============================================================
//  HELPER FUNCTIONS
//  Each function sets up a movement, runs it, then stops.
//  You can call them in any order inside run_mission().
// ============================================================

// --- FORWARD ---
// speed   : forward velocity (positive Y)
// height  : body z position
// period  : gait cycle time in ms
// cycles  : number of steps
void go_forward(float speed, float height, int period, int cycles) {
  Velocity_t vel = {0.0f, speed, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, height};
  Euler_t    att = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// --- BACKWARD ---
void go_backward(float speed, float height, int period, int cycles) {
  Velocity_t vel = {0.0f, -speed, 0.0f};
  Vector_t   pos = {0.0f,  0.0f, height};
  Euler_t    att = {0.0f,  0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// --- STRAFE LEFT ---
// Moves sideways to the left (negative X direction)
void go_left(float speed, float height, int period, int cycles) {
  Velocity_t vel = {-speed, 0.0f, 0.0f};
  Vector_t   pos = { 0.0f, 0.0f, height};
  Euler_t    att = { 0.0f, 0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// --- STRAFE RIGHT ---
void go_right(float speed, float height, int period, int cycles) {
  Velocity_t vel = {speed, 0.0f, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, height};
  Euler_t    att = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// --- IN-PLACE LEFT TURN ---
// speed controls angular rate (0.5=slow, 1.5=normal, 2.5=fast)
void turn_left(float speed, float height, int period, int cycles) {
  Velocity_t vel = {0.001f, 0.001f, speed};   // omega > 0 = left/CCW
  Vector_t   pos = {0.0f,   0.0f,   height};
  Euler_t    att = {0.0f,   0.0f,   0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// --- IN-PLACE RIGHT TURN ---
void turn_right(float speed, float height, int period, int cycles) {
  Velocity_t vel = {0.001f, 0.001f, -speed};  // omega < 0 = right/CW
  Vector_t   pos = {0.0f,   0.0f,    height};
  Euler_t    att = {0.0f,   0.0f,    0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// --- ARC LEFT (forward + left curve) ---
// radius : arc radius in body units (~cm)
//          small radius (2) = tight circle, large (10) = gentle curve
// speed  : forward component (vy)
// The library computes omega from the vy/radius ratio automatically.
// We set omega = speed / radius to approximate the desired arc.
void arc_left(float radius, float speed, float height, int period, int cycles) {
  float omega = speed / radius;
  Velocity_t vel = {0.0f, speed, omega};
  Vector_t   pos = {0.0f, 0.0f, height};
  Euler_t    att = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// --- ARC RIGHT ---
void arc_right(float radius, float speed, float height, int period, int cycles) {
  float omega = speed / radius;
  Velocity_t vel = {0.0f, speed, -omega};
  Vector_t   pos = {0.0f, 0.0f, height};
  Euler_t    att = {0.0f, 0.0f,  0.0f};
  minihexa.move(&vel, &pos, &att, period, cycles);
  delay((uint32_t)period * cycles + 300);
}

// --- STAND STILL ---
// Robot stands at the given height for the given duration (ms).
// Good for pausing between moves.
void stand(float height, uint32_t ms) {
  Velocity_t vel = {0.0f, 0.0f, 0.0f};
  Vector_t   pos = {0.0f, 0.0f, height};
  Euler_t    att = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel, &pos, &att, 800);
  delay(ms);
}

// --- REST FLAT ---
// Sits down low (z=0) and waits.  Good for end of routine.
void rest_flat(uint32_t ms) {
  stand(1.0f, ms);
}

// ============================================================
//  YOUR MISSION — edit this function to build your sequence
// ============================================================
void run_mission() {

  // ---------- ADJUSTABLE PARAMETERS ----------
  // Change these numbers to tune each movement.

  //                       speed  height  period  cycles
  go_backward(           1.5f,  2.0f,   1000,   3    );  // slower, taller
  stand      (           1.5f,  1000                   );  // pause

  go_left    (           2.0f,  1.5f,   800,    6    );
  stand      (           1.5f,  1000                   );  // pause


  arc_left   (       10.0f,  2.0f,  1.0f,   800,    25    );
  stand      (           1.5f,  1000                   );  // pause

  go_right   (           2.0f,  1.5f,   800,    3    );
  stand      (           1.5f,  1000                   );  // pause


  go_forward (           2.0f,  1.5f,   800,    4    );
  stand      (           1.5f,  1000                   );  // pause
  // stand      (           1.5f,  500                   );  // pause

  // go_backward(           1.5f,  2.0f,   1000,   3    );  // slower, taller
  // stand      (           2.0f,  500                   );

  // go_left    (           2.0f,  1.5f,   800,    3    );
  // go_right   (           2.0f,  1.5f,   800,    3    );

  // // In-place left turn, then right turn
  // turn_left  (           1.5f,  1.5f,   800,    4    );
  // turn_right (           1.5f,  1.5f,   800,    4    );

  // // Arc — radius=4 means a moderately tight circle 4 means 4 cm so for 1 foot use 30f because 30cm
  // //                 radius  speed  height  period  cycles
  // arc_left   (       4.0f,  2.0f,  1.5f,   800,    6    );
  // arc_right  (       4.0f,  2.0f,  1.5f,   800,    6    );

  // rest_flat  (       2000                              );
}

// ============================================================
//  SETUP / LOOP
// ============================================================
void setup() {
  delay(10000);       // 10s safety — disconnect USB before this runs
  minihexa.begin();
  delay(2000);        // let servos settle

  run_mission();      // run once then stop
}

void loop() {}
