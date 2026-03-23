#include "hiwonder_robot.h"

//初始化MiniHexa对象
Robot minihexa;

//定义变量count来记录动作的模式
uint8_t count = 0;
//初始化运动状态
Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  switch(count) {
    case 0://前进
      count++;
      vel = {0.0f, 3.0f, 0.0f};
      break;
  
    case 1://右前进
      count++;
      vel = {2.0f, 2.0f, 0.0f};
      break;

    case 2://右移
      count++;
      vel = {3.0f, 0.0f, 0.0f};
      break;

    case 3://右后退
      count++;
      vel = {2.0f, -2.0f, 0.0f};
      break;

    case 4://后退
      count++;
      vel = {0.0f, -3.0f, 0.0f};
      break;

    case 5://左后退
      count++;
      vel = {-2.0f, -2.0f, 0.0f};
      break;

    case 6://左移动
      count++;
      vel = {-3.0f, 0.0f, 0.0};
      break;

    case 7://左前进
      count++;
      vel = {-2.0f, 2.0f, 0.0f};
      break;

    case 8://原地左转
      count++;
      vel = {0.0f, 0.0f, 2.0f};
      break;

    case 9://原地右转
      count = 0;
      vel = {0.0f, 0.0f, -2.0f};
      break;
  }
  delay(5500);
  minihexa.move(&vel, &pos, &att, 1800, 3);
}
