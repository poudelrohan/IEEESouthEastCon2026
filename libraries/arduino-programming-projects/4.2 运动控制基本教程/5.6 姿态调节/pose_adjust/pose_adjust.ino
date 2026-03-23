#include "hiwonder_robot.h"

Robot minihexa;

uint8_t count = 0;
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
      pos = {3.0f, 0.0f, 0.0f};
      att = {0.0f, 0.0f, 0.0f};
      delay(1000);
      break;
  
    case 1:
      count++;
      pos = {0.0f, 3.0f, 0.0f};
      delay(1000);
      break;

    case 2:
      count++;
      pos = {-3.0f, 0.0f, 0.0f};
      delay(1000);
      break;

    case 3:
      count++;
      pos = {0.0f, -3.0f, 0.0f};
      delay(1000);
      break;

    case 4:
      count++;
      pos = {0.0f, 0.0f, 3.0f};
      delay(1000);
      break;

    case 5:
      count++;
      pos = {0.0f, -2.0f, -1.0f};
      delay(1000);
      break;

    case 6:
      count++;
      att = {8.0f, 0.0f, 0.0f};
      pos = {0.0f, 0.0f, 0.0f};
      delay(1000);
      break;

    case 7:
      count++;
      att = {-8.0f, 0.0f, 0.0};
      delay(1000);
      break;

    case 8:
      count++;
      att = {0.0f, 12.0f, 0.0f};
      delay(1000);
      break;

    case 9:
      count++;
      att = {0.0f, -12.0f, 0.0f};
      delay(1000);
      break;

    case 10:
      count++;
      att = {0.0f, 0.0f, 12.0f};
      delay(1000);
      break;
  
    case 11:
      count = 0;
      att = {0.0f, 0.0f, -12.0f};
      delay(1000);
      break;
  }

  minihexa.move(&vel, &pos, &att, 600);
}
