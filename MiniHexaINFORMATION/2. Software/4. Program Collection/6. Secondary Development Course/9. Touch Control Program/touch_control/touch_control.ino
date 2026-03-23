#include "hiwonder_robot.h"

Robot minihexa;

uint8_t touch_state;
Velocity_t velocity = {0.0f, 0.0f, 0.0f};
Vector_t position = {0.0f, 0.0f, 0.0f};
Euler_t _euler = {0.0f, 0.0f, 0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}
void loop() {

  touch_state = minihexa.sensor.get_touch_state();
  if(touch_state == 0) {
    velocity = {0.0f, 0.02f, 0.0f};
    minihexa.move(&velocity, &position, &_euler, 800, 1);
    delay(2000);
  }
}
