#include "hiwonder_robot.h"

Robot minihexa;

uint8_t val[4];
float yaw;
float amplitude;

Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}
void loop() {
  minihexa.sensor.camera.face_data_receive(val, sizeof(val));
  if(val[0] != 0) {
    minihexa.acting_cute();
  }
  delay(20);
}
