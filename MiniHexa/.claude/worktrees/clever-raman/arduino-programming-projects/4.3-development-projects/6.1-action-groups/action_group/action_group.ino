#include "hiwonder_robot.h"

Robot minihexa;

uint8_t result;

uint8_t write_data[2][60] = {{0,2,1,18,200,0,1,173,6,2,58,4,3,69,2,4,127,6,5,205,4,6,88,2,7,190,6,8,164,3,9,246,3,10,173,6,11,86,6,12,206,8,13,127,6,14,234,6,15,95,9,16,190,6,17,161,7,18,194,9},
                             {0,2,2,18,144,1,1,225,5,2,204,4,3,71,2,4,226,5,5,235,4,6,77,2,7,220,5,8,233,4,9,97,2,10,8,6,11,206,6,12,70,9,13,235,5,14,203,6,15,115,9,16,220,5,17,234,6,18,89,9}};

void setup() {
  Serial.begin(115200);
  minihexa.begin();
  /* List action group files */
  minihexa.list_action_group_dir();
  /* Download action groups */
  minihexa.action_group_download(0, write_data[0], 60);
  minihexa.action_group_download(0, write_data[1], 60);
  /* Run action group */
  minihexa.action_group_run(0);
}
void loop() {
}
