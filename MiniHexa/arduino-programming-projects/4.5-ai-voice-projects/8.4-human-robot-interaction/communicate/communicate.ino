#include "hiwonder_robot.h"

Robot minihexa;

uint8_t result;

void setup() {
  Serial.begin(115200);
  minihexa.begin();
}

void loop() {
  result = minihexa.sensor.asr.rec_recognition();
  switch(result) {
    case 26:  /* Recognized "Hello" */
      minihexa.action_group_run(14);
      break;

    case 27:  /* Recognized "Introduce yourself" */
      minihexa.acting_cute();
      break;

    case 28:  /* Recognized "Show a trick" */
      minihexa.action_group_run(7);
      break;
    
    default:
      break;
  }
}
