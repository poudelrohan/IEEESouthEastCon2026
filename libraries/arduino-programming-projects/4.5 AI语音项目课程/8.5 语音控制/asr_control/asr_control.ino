#include "hiwonder_robot.h"

Robot minihexa;

uint8_t result;

Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  result = minihexa.sensor.asr.rec_recognition();
  switch(result) {
    case 1:  /* 前进 */
      vel = {0.0f, 2.0f, 0.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 2:  /* 后退 */
      vel = {0.0f, -2.0f, 0.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 3:  /* 左转 */
      vel = {0.0f, 0.0f, 2.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 4:  /* 右转 */
      vel = {0.0f, 0.0f, -2.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 9:  /* 停止 */
      vel = {0.0f, 0.0f, 0.0f};
      minihexa.move(&vel, &pos, &att);
      break;

    case 29:  /* 走两步 */
      vel = {0.0f, 2.0f, 0.0f};
      minihexa.move(&vel, &pos, &att, 1000, 2); 
      delay(2100);
      break;

    default:
      break;
  }
  delay(100);
}
