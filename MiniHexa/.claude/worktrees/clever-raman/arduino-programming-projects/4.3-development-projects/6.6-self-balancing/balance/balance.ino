#include "hiwonder_robot.h"

Robot minihexa;

Velocity_t velocity = {0.0f, 0.0f, 0.0f};
Vector_t position = {0.0f, 0.0f, 0.0f};
Euler_t _euler = {0.0f, 0.0f, 0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
  delay(2000);
  position = {0.0f, 0.0f, 1.5f};
  minihexa.move(&velocity, &position, &_euler, 1000);
  delay(1000);
}
void loop() {
  minihexa.balance();
}
