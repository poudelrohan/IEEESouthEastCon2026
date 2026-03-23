// ============================================================
//  OBSTACLE CROSSING — Action Group 6, run for 20 seconds
//
//  Plays the pre-recorded "No.6 Obstacle Crossing" action
//  group on repeat for exactly 20 seconds.
//
//  The sketch is SELF-CONTAINED — it uploads AG6 to SPIFFS
//  automatically on startup.  No need to run the uploader
//  sketch first.
//
//  AG6 is the high-leg walking move: the robot lifts every
//  leg to maximum height and places it down deliberately,
//  like stepping over an obstacle.
//
//  LED colours:
//    YELLOW = uploading AG6 to SPIFFS
//    BLUE   = running action group
//    WHITE  = done (20 s elapsed)
//
//  RUN_TIME_MS — change to run longer or shorter.
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"
#include "SPIFFS.h"
#include "action_group_data.h"

Robot minihexa;

const uint32_t RUN_TIME_MS = 20000;  // 20 seconds

// Upload one action group's binary data to SPIFFS
void upload_ag(uint8_t id, const uint8_t* progmem_data, size_t total_size) {
  String path = "/ActionGroup" + String(id) + ".rob";
  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) return;

  uint8_t buf[64];
  size_t written = 0;
  while (written < total_size) {
    size_t chunk = min((size_t)64, total_size - written);
    for (size_t i = 0; i < chunk; i++) {
      buf[i] = pgm_read_byte(progmem_data + written + i);
    }
    file.write(buf, chunk);
    written += chunk;
  }
  file.close();
}

void setup() {
  delay(10000);  // safety — disconnect USB before placing robot

  minihexa.begin();  // also calls SPIFFS.begin() internally
  delay(2000);       // servos stand up

  // Yellow LED = uploading to SPIFFS
  uint8_t yellow[3] = {200, 150, 0};
  minihexa.sensor.set_ultrasound_rgb(0, yellow, yellow);

  // Write AG6 binary to SPIFFS so action_group_run(6) can find it
  upload_ag(6, ag_6_data, ag_6_size);
  delay(500);

  // Blue LED = running
  uint8_t blue[3] = {0, 0, 200};
  minihexa.sensor.set_ultrasound_rgb(0, blue, blue);

  // Repeat AG6 until time is up.
  // action_group_run() is fully blocking: one call = one full
  // stride cycle.  Loop replays it continuously.
  uint32_t start = millis();
  while (millis() - start < RUN_TIME_MS) {
    minihexa.action_group_run(6);
  }

  // White LED = done
  uint8_t white[3] = {200, 200, 200};
  minihexa.sensor.set_ultrasound_rgb(0, white, white);
}

void loop() {}
