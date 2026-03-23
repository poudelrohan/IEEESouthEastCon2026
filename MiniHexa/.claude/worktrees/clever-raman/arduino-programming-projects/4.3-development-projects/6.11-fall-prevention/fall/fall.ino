#include "hiwonder_robot.h"

Robot minihexa;

uint8_t ir1_state;
uint8_t ir2_state;

Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t pos = {0.0f, 0.0f, 0.0f};
Euler_t att = {0.0f, 0.0f, 0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
  delay(1000);
}

void loop() {
  ir1_state = minihexa.sensor.get_ir1_state();
  ir2_state = minihexa.sensor.get_ir2_state();
  if(ir1_state == 1 || ir2_state == 1) {
    vel = {0.0f, -3.0f, 0.0f};
    minihexa.move(&vel, &pos, &att, 600, 3);
    delay(2400);
    vel = {0.0f, 0.0f, 2.0f};
    minihexa.move(&vel, &pos, &att, 600, 4);
    delay(3000);
  }
  else {
    vel = {0.0f, 3.0f, 0.0f};
    minihexa.move(&vel, &pos, &att);
  }
}
