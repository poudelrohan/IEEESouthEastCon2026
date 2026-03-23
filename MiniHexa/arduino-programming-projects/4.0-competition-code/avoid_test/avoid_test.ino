// ============================================================
//  OBSTACLE AVOIDANCE TEST — 20 seconds
//
//  Runs the built-in obstacle avoidance state machine for
//  exactly 20 seconds with the body raised to maximum height
//  so the legs reach as high as possible.
//
//  How avoid() works (state machine):
//    FORWARD  → walks forward at full speed
//    < 200 mm → obstacle close    → TURN (rotate left 4 steps)
//    < 100 mm → obstacle very close → BACK (reverse first)
//    clear    → back to FORWARD
//
//  Body height:
//    BODY_HEIGHT = 3.0  → legs fully extended (maximum height)
//    Reduce toward 1.5 if the robot becomes unstable.
//
//  LED colours:
//    CYAN    = running avoid  (pulses every reading)
//    WHITE   = done (20 s elapsed)
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"

Robot minihexa;

// ==========================================
// SETTINGS
// ==========================================
const uint32_t RUN_TIME_MS = 20000;  // 20 seconds
const float    BODY_HEIGHT  = 3.0f;  // position.z — max ≈ 3-4

// ==========================================
// SETUP
// ==========================================
void setup() {
  delay(10000);  // safety — disconnect USB first

  minihexa.begin();
  delay(2000);   // let servos stand up

  // Raise body to maximum height.
  // avoid() copies the internal position each call, so
  // setting it here makes the avoid loop run with legs high.
  Velocity_t vel_stop = {0.0f, 0.0f, 0.0f};
  Vector_t   pos_high = {0.0f, 0.0f, BODY_HEIGHT};
  Euler_t    att_flat = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel_stop, &pos_high, &att_flat, 1000);
  delay(1500);  // wait for body to rise

  // LED: cyan = running
  uint8_t cyan[3] = {0, 180, 180};
  minihexa.sensor.set_ultrasound_rgb(0, cyan, cyan);

  // ---- Run avoid() for 20 seconds ----
  uint32_t start = millis();
  while (millis() - start < RUN_TIME_MS) {
    uint16_t dis = minihexa.sensor.get_distance();
    minihexa.avoid(dis);
    delay(50);
  }

  // Stop and lower body back to normal
  Vector_t pos_normal = {0.0f, 0.0f, 0.0f};
  minihexa.move(&vel_stop, &pos_normal, &att_flat, 800);
  delay(1000);

  // LED: white = done
  uint8_t white[3] = {200, 200, 200};
  minihexa.sensor.set_ultrasound_rgb(0, white, white);
}

void loop() {}
