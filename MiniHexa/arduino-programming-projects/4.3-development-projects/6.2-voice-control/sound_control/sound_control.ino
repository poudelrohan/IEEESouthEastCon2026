#include "hiwonder_robot.h"

Robot minihexa;

uint16_t sound_value;

Velocity_t velocity = {0.0f, 0.0f, 0.0f};
Vector_t position = {0.0f, 0.0f, 0.0f};
Euler_t _euler = {0.0f, 0.0f, 0.0f};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}
void loop() {
  sound_value = minihexa.board.get_sound_val();
  if(sound_value >= 600) {
    velocity = {0.0f, 2.0f, 0.0f};
    minihexa.move(&velocity, &position, &_euler, 1000, 1);
    delay(1000);
  }
}
