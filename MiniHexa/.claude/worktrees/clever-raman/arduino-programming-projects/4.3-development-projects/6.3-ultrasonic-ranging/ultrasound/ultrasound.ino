#include "hiwonder_robot.h"

Robot minihexa;

uint8_t s;
uint16_t dis;
uint8_t rgb1[3] = {0};
uint8_t rgb2[3] = {0};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}
void loop() {
  dis = minihexa.sensor.get_distance();
  if (dis > 0 && dis <= 80){         // Breathing LED mode, 0.1s period, red color
    rgb1[0] = 1;
    rgb1[1] = 0;
    rgb1[2] = 0;
    memcpy(rgb2, rgb1, sizeof(rgb1));
    minihexa.sensor.set_ultrasound_rgb(1, rgb1, rgb2);
  }
  else if (dis > 80 && dis <= 180){   // Red gradient
    s = map(dis,80,180,0,255);
    rgb1[0] = 255-s;
    rgb1[1] = 0;
    rgb1[2] = 0;
    memcpy(rgb2, rgb1, sizeof(rgb1));
  }
  else if (dis > 180 && dis <= 320){              // Blue gradient
    s = map(dis,180,320,0,255);
    rgb1[0] = 0;
    rgb1[1] = 0;
    rgb1[2] = s;   
    memcpy(rgb2, rgb1, sizeof(rgb1)); 
  }
  else if (dis > 320 && dis <= 500){     // Green gradient
    s = map(dis,320,500,0,255);
    rgb1[0] = 0;
    rgb1[1] = s;
    rgb1[2] = 255-s;
    memcpy(rgb2, rgb1, sizeof(rgb1));
  }
  else if (dis > 500){         // Green
    rgb1[0] = 0;
    rgb1[1] = 255;
    rgb1[2] = 0;    
    memcpy(rgb2, rgb1, sizeof(rgb1));
  }
  minihexa.sensor.set_ultrasound_rgb(1, rgb1, rgb2);
  Serial.printf("Distance: %d mm\n", dis);
}
