// ============================================================
//  ACTION GROUP UPLOADER & TESTER  (Autonomous — no laptop needed)
//
//  1. Upload this sketch while robot is OFF battery
//  2. Disconnect USB, power on battery, set robot on the ground
//  3. 10-second delay → uploads action groups to SPIFFS
//  4. Runs every action group one by one with 3s pauses
//  5. After this, any future sketch can call:
//       minihexa.action_group_run(6);  // Obstacle Crossing
//       minihexa.action_group_run(13); // Door Push
//       etc.  (they persist in SPIFFS across reboots)
// ============================================================

#include "hiwonder_robot.h"
#include "SPIFFS.h"
#include "action_group_data.h"

Robot minihexa;

bool upload_action_group(uint8_t id, const uint8_t* progmem_data, size_t total_size) {
    String path = "/ActionGroup" + String(id) + ".rob";

    File file = SPIFFS.open(path, FILE_WRITE);
    if (!file) {
        return false;
    }

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
    return true;
}

void upload_all_action_groups() {
    for (int i = 0; i < NUM_ACTION_GROUPS; i++) {
        upload_action_group(
            action_groups[i].id,
            action_groups[i].data,
            action_groups[i].size
        );
    }
}

void setup() {
    delay(10000);  // 10s safety delay — disconnect USB, put on ground

    Serial.begin(115200);
    minihexa.begin();

    // Write all action groups to SPIFFS (persists across reboots)
    upload_all_action_groups();

    delay(3000);  // Let robot stand up and stabilize

    // Run every action group so you can see what each one does
    for (int i = 0; i < NUM_ACTION_GROUPS; i++) {
        Serial.printf(">>> AG%d: %s\n", action_groups[i].id, action_groups[i].name);
        minihexa.action_group_run(action_groups[i].id);
        delay(3000);  // 3s pause between each so you can tell them apart
    }

    Serial.println("Done! All action groups uploaded and demoed.");
}

void loop() {
    // Nothing — one-shot demo
}
