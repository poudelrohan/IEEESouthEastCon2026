#include "hiwonder_robot.h"

Robot minihexa;

uint16_t dis;
uint8_t rgb1[3] = {0, 0, 100};
uint8_t rgb2[3] = {0, 0, 100};
Velocity_t velocity = {0.0f, 0.0f, 0.0f};
Vector_t position = {0.0f, 0.0f, 0.0f};
Euler_t euler = {0.0f, 0.0f, 0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
  delay(1000);
  minihexa.sensor.set_ultrasound_rgb(0, rgb1, rgb2);
}
void loop() {
  dis = minihexa.sensor.get_distance();
  if(dis > 200) {
    velocity = {0.0f, 2.0f, 0.0f};
  }
  else if(dis < 100) {
    velocity = {0.0f, -2.0f, 0.0f};
  }
  else {
    velocity = {0.0f, 0.0f, 0.0f};
  }
  minihexa.move(&velocity, &position, &euler);
  delay(100);
}
