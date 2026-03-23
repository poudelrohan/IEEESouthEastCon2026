#include "hiwonder_robot.h"
// Initialize MiniHexa object
Robot minihexa;

// Define movement mode variable count
uint8_t count = 0;
// Number of iterations in the discretization process (footfall count)
int step_num = -1;
// Initialize movement duration
uint32_t move_time = 1000;
// Leg lift height
float leg_lift = 2.0f;
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
    case 0:
      count++;
      vel = {0.0f, 2.0f, 0.0f}; // Move forward
      move_time = 600; // Set movement duration
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
