#include "hiwonder_robot.h"

// Initialize MiniHexa object
Robot minihexa;

// Initialize motion state
Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
    vel = {0.0f, 5.0f, 0.2f}; // Arc forward-left
    minihexa.move(&vel, &pos, &att); // Execute movement
    delay(5000);

    vel = {0.0f, 5.0f, -0.2f}; // Arc forward-right
    minihexa.move(&vel, &pos, &att); // Execute movement
    delay(5000);
}

