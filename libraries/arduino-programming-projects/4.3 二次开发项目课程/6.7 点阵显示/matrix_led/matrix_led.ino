#include "hiwonder_robot.h"
#include "hiwonder_sensor.h"
#include "WMMatrixLED.h"

//创建minihexa对象
Robot minihexa;
//初始化点阵模块引脚
WMMatrixLed matrix(14, 32);  //  SCK / DIN 引脚编号

void setup() {
  Serial.begin(115200);
  minihexa.begin();
  matrix.setBrightness(5);//设置亮度
  matrix.clearScreen();//清屏幕
}
int x;
void loop() {
  const char* text = "Hiwonder";
  int textWidth = 6 * strlen(text); // 每个字符约6像素
  for (int x = 16; x > -textWidth; x--) {
    matrix.drawStr(x, 8, text);       //开始显示
    delay(100);
  }
}
