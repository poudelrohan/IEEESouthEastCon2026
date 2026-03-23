#include "hiwonder_robot.h"

Robot minihexa;

uint16_t dis;
uint8_t rgb1[3] = {0};
uint8_t rgb2[3] = {0};
uint32_t tickstart = 0;

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  dis = minihexa.sensor.get_distance();
  Serial.println(dis);
  if (dis < 100) {         // Breathing LED mode, 0.1s period, red color
    rgb1[0] = 255;
    rgb1[1] = 0;
    rgb1[2] = 0;
    memcpy(rgb2, rgb1, sizeof(rgb1));
    minihexa.sensor.set_ultrasound_rgb(1, rgb1, rgb2);
    if(millis() - tickstart > 3000) {
      minihexa.sensor.asr.speak(ASR_ANNOUNCER, 5);
      tickstart = millis();
    }
  }
  else {   // Red gradient
    rgb1[0] = 0;
    rgb1[1] = 255;
    rgb1[2] = 0;
    memcpy(rgb2, rgb1, sizeof(rgb1));
    minihexa.sensor.set_ultrasound_rgb(1, rgb1, rgb2);
  }
  delay(20);
}
