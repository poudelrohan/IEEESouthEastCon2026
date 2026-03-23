#include "hiwonder_robot.h"
//初始化minihexa对象
Robot minihexa;

//定义运动模式变量count
uint8_t count = 0;
//离散化过程的迭代次数（落脚次数）
int step_num = -1;
//初始化运动时间
uint32_t move_time = 1000;
//抬腿高度
float leg_lift = 2.0f;
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
    case 0:
      count++;
      vel = {0.0f, 2.0f, 0.0f};//前进
      move_time = 600;//定义运行时间
      step_num = 3;
      break;
  
    case 1:
      count++;
      vel = {0.0f, 2.0f, 0.0f};
      move_time = 1000;
      step_num = 2;
      break;

    case 2:
      count++;
      vel = {0.0f, -2.0f, 0.0f};
      move_time = 600;
      step_num = 3;
      break;

    case 3:
      count++;
      vel = {0.0f, -2.0f, 0.0f};
      move_time = 1000;
      step_num = 2;
      break;

    case 4:
      count++;
      vel = {0.0f, 0.0f, 2.0f};
      move_time = 600;
      step_num = 2;
      break;

    case 5:
      vel = {0.0f, 0.0f, -2.0f};
      move_time = 1000;
      step_num = -1;
      break;
  }

  minihexa.move(&vel, &pos, &att, move_time, step_num);
  delay(4000);
}
