#include "hiwonder_robot.h"

Robot minihexa;

uint8_t count = 0;
//初始化机体运动
Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  switch(count) {
    case 0://原地左转最慢速度
      count++;
      vel = {0.0f, 0.0f, 1.0f};
      delay(5000);
      break;
  
    case 1://原地左转较慢速度
      count++;
      vel = {0.0f, 0.0f, 1.5f};
      delay(5000);
      break;

    case 2://原地左转中速
      count++;
      vel = {0.0f, 0.0f, 2.0f};
      delay(5000);
      break;

    case 3://原地左转高速
      count = 0;
      vel = {0.0f, 0.0f, 2.5f};
      delay(5000);
      break;
  }

  minihexa.move(&vel, &pos, &att);//执行运动
}
