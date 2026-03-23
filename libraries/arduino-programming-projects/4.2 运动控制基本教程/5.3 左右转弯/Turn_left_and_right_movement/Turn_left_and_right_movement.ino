#include "hiwonder_robot.h"

//初始化MiniHexa对象
Robot minihexa;

//初始化运动状态
Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
    vel = {0.0f, 5.0f, 0.2f};//弧形左前进
    minihexa.move(&vel, &pos, &att);//执行移动
    delay(5000);

    vel = {0.0f, 5.0f, -0.2f};//弧形右前进
    minihexa.move(&vel, &pos, &att);//执行移动
    delay(5000);
}

