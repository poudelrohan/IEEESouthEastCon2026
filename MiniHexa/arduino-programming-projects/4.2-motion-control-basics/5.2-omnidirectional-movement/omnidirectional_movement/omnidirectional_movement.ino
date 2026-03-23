#include "hiwonder_robot.h"

// Initialize MiniHexa object
Robot minihexa;

// Define variable count to track movement mode
uint8_t count = 0;
// Initialize motion state
Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  switch(count) {
    case 0: // Move forward
      count++;
      vel = {0.0f, 3.0f, 0.0f};
      break;
  
    case 1: // Move forward-right
      count++;
      vel = {2.0f, 2.0f, 0.0f};
      break;

    case 2: // Move right
      count++;
      vel = {3.0f, 0.0f, 0.0f};
      break;

    case 3: // Move backward-right
      count++;
      vel = {2.0f, -2.0f, 0.0f};
      break;

    case 4: // Move backward
      count++;
      vel = {0.0f, -3.0f, 0.0f};
      break;

    case 5: // Move backward-left
      count++;
      vel = {-2.0f, -2.0f, 0.0f};
      break;

    case 6: // Move left
      count++;
      vel = {-3.0f, 0.0f, 0.0};
      break;

    case 7: // Move forward-left
      count++;
      vel = {-2.0f, 2.0f, 0.0f};
      break;

    case 8: // Turn left in place
      count++;
      vel = {0.0f, 0.0f, 2.0f};
      break;

    case 9: // Turn right in place
      count = 0;
      vel = {0.0f, 0.0f, -2.0f};
      break;
  }
  delay(5500);
  minihexa.move(&vel, &pos, &att, 1800, 3);
}
