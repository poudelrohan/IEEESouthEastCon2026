#include "hiwonder_robot.h"

Robot minihexa;

uint8_t count = 0;
// Initialize body motion
Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  switch(count) {
    case 0: // Turn left in place - slowest speed
      count++;
      vel = {0.0f, 0.0f, 1.0f};
      delay(5000);
      break;
  
    case 1: // Turn left in place - slow speed
      count++;
      vel = {0.0f, 0.0f, 1.5f};
      delay(5000);
      break;

    case 2: // Turn left in place - medium speed
      count++;
      vel = {0.0f, 0.0f, 2.0f};
      delay(5000);
      break;

    case 3: // Turn left in place - high speed
      count = 0;
      vel = {0.0f, 0.0f, 2.5f};
      delay(5000);
      break;
  }

  minihexa.move(&vel, &pos, &att); // Execute movement
}
