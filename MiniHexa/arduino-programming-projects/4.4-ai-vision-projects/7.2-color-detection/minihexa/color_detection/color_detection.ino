#include "hiwonder_robot.h"

Robot minihexa;

uint8_t val[4];
uint8_t rgb1[3] = {0};
uint8_t rgb2[3] = {0};

Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  delay(1000);
  Serial.begin(115200);
  minihexa.begin();
  delay(1000);
  minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb1, rgb2);
}
void loop() {
  Serial.printf("%d %d %d %d\n", val[0], val[1], val[2], val[3]);
  minihexa.sensor.camera.color_id_detection(val, sizeof(val));
  if(val[0] == 1 && val[1] == 0 && val[2] == 0 && val[3] == 0) {
    rgb1[0] = 10;
    rgb1[1] = 0;
    rgb1[2] = 0;
    memcpy(rgb2, rgb1, sizeof(rgb1));
    minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb1, rgb2);
    delay(500);

  }
  else if(val[0] == 0 && val[1] == 2 && val[2] == 0 && val[3] == 0) {
    rgb1[0] = 0;
    rgb1[1] = 10;
    rgb1[2] = 0;
    memcpy(rgb2, rgb1, sizeof(rgb1));
    minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb1, rgb2);
  }
  else if(val[0] == 0 && val[1] == 0 && val[2] == 3 && val[3] == 0) {
    rgb1[0] = 0;
    rgb1[1] = 0;
    rgb1[2] = 10;
    memcpy(rgb2, rgb1, sizeof(rgb1));
    minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb1, rgb2);
  }
  delay(20);
}
