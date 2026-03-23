#include "hiwonder_robot.h"

Robot minihexa;

uint8_t val[4];
uint8_t rgb1[3] = {0};
uint8_t rgb2[3] = {0};

Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
  delay(100);
  minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb1, rgb2);
}
void loop() {
  minihexa.sensor.camera.region2_red_block_detection(val, sizeof(val));
  Serial.println(val[0]);
  if(val[0] > 120) {
    vel = {0.0f, 1.0f, -0.1f};
  }
  else if(val[0] < 40) {
    vel = {0.0f, 1.0f, 0.1f};
  }
  else{
    vel = {0.0f, 1.0f, 0.0f};
  }
  minihexa.move(&vel, &pos, &att, 700);
  delay(20);
}
