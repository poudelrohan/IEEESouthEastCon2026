// ============================================================
//  CRATER HIGH-STEP — Action Group 6 with speed control
//
//  Plays the pre-recorded "No.6 Obstacle Crossing" action
//  group on repeat for RUN_TIME_MS milliseconds, at a
//  controllable speed.
//
//  The sketch is SELF-CONTAINED — it uploads AG6 to SPIFFS
//  automatically on startup.  No need to run the uploader
//  sketch first.
//
//  AG6 lifts every leg to maximum height and places it down
//  deliberately — exactly the motion needed to claw up the
//  slippery crater wall.
//
//  SPEED_FACTOR — the only variable you need to tune:
//    1.0  = original Hiwonder speed
//    2.0  = twice as slow  (good start for crater)
//    3.0  = three times as slow  (maximum grip)
//    0.5  = twice as fast
//  Each frame's servo move-time is scaled by this value.
//  Slowing down gives each foot more time to dig in before
//  the next leg lifts.  Min clamped to 50 ms per frame.
//
//  LED colours:
//    YELLOW = uploading AG6 to SPIFFS
//    BLUE   = running
//    WHITE  = done
//
//  *** NEVER connect USB while running — fire hazard! ***
// ============================================================

#include "hiwonder_robot.h"
#include "SPIFFS.h"
#include "action_group_data.h"

Robot minihexa;

// ==========================================
// TUNE THESE
// ==========================================
const float    SPEED_FACTOR = 1.0f;   // 1.0=normal  2.0=half speed  3.0=very slow
const uint32_t RUN_TIME_MS  = 30000;  // total run time in ms

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
  delay(10000);  // safety — disconnect USB, place robot

  minihexa.begin();  // also calls SPIFFS.begin() internally
  delay(2000);       // servos stand up

  // Yellow LED = uploading to SPIFFS
  uint8_t yellow[3] = {200, 150, 0};
  minihexa.sensor.set_ultrasound_rgb(0, yellow, yellow);

  // Write AG6 binary to SPIFFS so action_group_run_scaled() can find it
  upload_ag(6, ag_6_data, ag_6_size);
  delay(500);

  // Blue LED = running
  uint8_t blue[3] = {0, 0, 200};
  minihexa.sensor.set_ultrasound_rgb(0, blue, blue);

  // Repeat AG6 at SPEED_FACTOR until time is up
  uint32_t start = millis();
  while (millis() - start < RUN_TIME_MS) {
    minihexa.action_group_run_scaled(6, SPEED_FACTOR);
  }

  // White LED = done
  uint8_t white[3] = {200, 200, 200};
  minihexa.sensor.set_ultrasound_rgb(0, white, white);
}

void loop() {}
