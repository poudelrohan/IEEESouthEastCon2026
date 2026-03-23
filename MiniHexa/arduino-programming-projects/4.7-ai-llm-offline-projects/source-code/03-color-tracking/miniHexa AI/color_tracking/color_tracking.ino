#include "hiwonder_robot.h"

Robot minihexa;

float increase;
float yaw;
uint8_t val[4];
uint8_t rgb1[3] = {0};
uint8_t rgb2[3] = {0};

Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
  delay(1000);
  minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb1, rgb2); 
}
void loop() {
  minihexa.sensor.camera.green_block_detection(val, sizeof(val));
  if(val[0] != 0 && val[1] != 0 && val[2] != 0 && val[3] != 0) {
    increase = fmap((float)val[0], 0, 160, -1.0f, 1.0f);
    yaw = yaw > 20.0f ? 20.0f : yaw < -20.0f ? -20.0f : yaw + increase;

  }
  att = {0.0f, 0.0f, yaw};
  minihexa.move(&vel, &pos, &att, 50); 
  delay(50);
}
