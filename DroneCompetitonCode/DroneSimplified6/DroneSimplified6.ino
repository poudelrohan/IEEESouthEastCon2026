// ══════════════════════════════════════════════════════════════
//  TELLO SIMPLE SEQUENCE v6 — MISSION PAD "GO" COMMAND
//
//  KEY DIFFERENCE from v4/v5:
//    v4/v5: Manual RC centering loop (±15 bang-bang nudges)
//           → oscillation, stale data, overshooting
//    v6:    Uses Tello's built-in "go x y z speed mid" command
//           to fly to EXACT coordinates relative to mission pad.
//           Tello's internal PID handles all positioning —
//           NO RC commands needed for centering!
//
//  How "go x y z speed mid" works:
//    - Flies drone to (x, y, z) relative to mission pad center
//    - Rocket icon on pad = positive X direction
//    - z = height above pad (detection range: 30-120cm)
//    - speed = 10-100 cm/s
//    - mid = "m-2" (nearest) or "m1"-"m8" (specific pad)
//    - Blocks until arrival, returns "ok" or "error"
//    - If pad not detected → returns "error", drone hovers
//
//  Commands:
//    S = Full sequence (Phase 1 movement + go land on pad)
//    G = Quick GO test (takeoff + go to pad + land — no movement)
//    B = Battery check
//    E = EMERGENCY (motors off!)
//    L = Normal land
// ══════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <WiFiUdp.h>

// ══════════════════════════════════════════════════════════════
//                    EASY TO CHANGE VARIABLES
// ══════════════════════════════════════════════════════════════

// ─────────────── WIFI CREDENTIALS ───────────────
const char* WIFI_NAME = "TELLO-BCU-DRONE";
const char* WIFI_PASSWORD = "southeast2026";

// ─────────────── PHASE 1 MOVEMENT (cm) ───────────────
int SAFETY_FORWARD = 20;      // Step 2: Away from back net
int MOVE_RIGHT_1 = 40;        // Step 3: Right toward center
int ROTATE_DEGREES = 270;     // Step 4: Rotate (cw)
int MOVE_FORWARD_2 = 40;      // Step 5: Forward
int MOVE_LEFT_1 = 30;         // Step 6: Small adjustment

// ─────────────── GO COMMAND SETTINGS (NEW in v6!) ───────────────
// "go x y z speed mid" flies drone to exact position above pad.
// X = forward (rocket icon direction on pad)
// Y = left/right on pad
// Z = height above pad
// Detection range: 30cm to 120cm height
int GO_X = 0;              // X offset from pad center (cm). 0 = centered
int GO_Y = 0;              // Y offset from pad center (cm). 0 = centered
int GO_HEIGHT_HIGH = 60;   // First approach height (cm)
int GO_HEIGHT_MID  = 50;   // Second approach height (cm)
int GO_HEIGHT_LOW  = 40;   // Final approach height (cm) — low before land
int GO_SPEED = 20;         // Go speed (10-100 cm/s). 20 = slow and stable
int GO_TIMEOUT_MS = 30000; // Max wait for go command response

// ─────────────── SEARCH SETTINGS ───────────────
int SEARCH_TIMEOUT = 15000;   // Max time to look for pad (ms)

// ─────────────── TIMING (ms) ───────────────
int STABILIZE_AFTER_TAKEOFF = 3000;   // Wait after takeoff
int PAUSE_AFTER_MOVE = 1500;          // Pause after each move
int COMMAND_RETRY_DELAY = 500;        // Delay before retry

// ─────────────── LANDING STYLE ───────────────
bool USE_EMERGENCY_LAND = false;  // true = drop, false = gentle

// ══════════════════════════════════════════════════════════════
//                    NETWORK SETTINGS (don't change)
// ══════════════════════════════════════════════════════════════

const char* TELLO_IP = "192.168.10.1";
const int CMD_PORT = 8889;
const int STATE_PORT = 8890;

WiFiUDP udpCmd;
WiFiUDP udpState;

// ══════════════════════════════════════════════════════════════
//                    STATE VARIABLES
// ══════════════════════════════════════════════════════════════

bool flightActive = false;
int padId = -1;
int padX = 0, padY = 0, padZ = 0;
int batteryLevel = 0;
unsigned long lastStateRead = 0;

// ══════════════════════════════════════════════════════════════
//                    TELLO COMMUNICATION
// ══════════════════════════════════════════════════════════════

void sendCommand(const char* cmd) {
    Serial.print(">> ");
    Serial.println(cmd);
    udpCmd.beginPacket(TELLO_IP, CMD_PORT);
    udpCmd.print(cmd);
    udpCmd.endPacket();
}

String waitResponse(int timeoutMs = 10000) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)timeoutMs) {
        int size = udpCmd.parsePacket();
        if (size) {
            char buf[256];
            int len = udpCmd.read(buf, 255);
            buf[len] = 0;
            Serial.print("<< ");
            Serial.println(buf);
            return String(buf);
        }
        delay(10);
    }
    Serial.println("<< TIMEOUT");
    return "TIMEOUT";
}

bool sendWithRetry(const char* cmd, int delayAfter = 0) {
    sendCommand(cmd);
    String resp = waitResponse();

    if (resp != "ok") {
        Serial.println("   Retrying...");
        delay(COMMAND_RETRY_DELAY);
        sendCommand(cmd);
        resp = waitResponse();
    }

    if (delayAfter > 0) delay(delayAfter);
    return (resp == "ok");
}

// ══════════════════════════════════════════════════════════════
//                    STATE READING
// ══════════════════════════════════════════════════════════════

bool readState() {
    String latest = "";
    while (udpState.parsePacket()) {
        char buf[512];
        int len = udpState.read(buf, 511);
        if (len > 0) {
            buf[len] = 0;
            latest = String(buf);
        }
    }

    if (latest.length() == 0) return false;

    lastStateRead = millis();

    int idx;

    idx = latest.indexOf("mid:");
    if (idx != -1) {
        padId = latest.substring(idx + 4, latest.indexOf(";", idx)).toInt();
    }

    idx = latest.indexOf(";x:");
    if (idx != -1) {
        padX = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();
    }

    idx = latest.indexOf(";y:");
    if (idx != -1) {
        padY = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();
    }

    idx = latest.indexOf(";z:");
    if (idx != -1) {
        padZ = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();
    }

    idx = latest.indexOf("bat:");
    if (idx != -1) {
        batteryLevel = latest.substring(idx + 4, latest.indexOf(";", idx)).toInt();
    }

    return true;
}

// ══════════════════════════════════════════════════════════════
//                    LANDING FUNCTIONS
// ══════════════════════════════════════════════════════════════

void performLand() {
    delay(200);

    if (USE_EMERGENCY_LAND) {
        Serial.println("\n*** EMERGENCY LAND (drop) ***");
        sendCommand("emergency");
    } else {
        Serial.println("\n*** NORMAL LAND ***");
        sendCommand("land");
    }
    waitResponse(10000);
    flightActive = false;
}

// ══════════════════════════════════════════════════════════════
//                    HOVER AND SEARCH FOR PAD
//
//  Still needed: the "go" command requires the pad to be
//  currently detected. So we hover and poll state until
//  mid > 0 (pad found), THEN send the go command.
// ══════════════════════════════════════════════════════════════

bool hoverAndSearchForPad() {
    Serial.println("\n----------- SEARCHING FOR PAD (hovering) -----------");
    Serial.printf("Timeout: %d seconds\n", SEARCH_TIMEOUT / 1000);

    unsigned long searchStart = millis();
    int lastSecondPrinted = -1;

    while (millis() - searchStart < (unsigned long)SEARCH_TIMEOUT) {

        // Emergency check
        if (Serial.available()) {
            char c = toupper(Serial.read());
            if (c == 'E') {
                Serial.println("\n!!! EMERGENCY !!!");
                sendCommand("emergency");
                flightActive = false;
                return false;
            }
            if (c == 'L') {
                Serial.println("\nManual land...");
                sendCommand("land");
                flightActive = false;
                return false;
            }
        }

        readState();

        // Print status every second
        int elapsedSec = (millis() - searchStart) / 1000;
        int remainingSec = (SEARCH_TIMEOUT / 1000) - elapsedSec;

        if (elapsedSec != lastSecondPrinted) {
            lastSecondPrinted = elapsedSec;

            if (padId > 0) {
                Serial.printf("PAD FOUND! ID=%d | X:%d Y:%d Z:%d\n",
                              padId, padX, padY, padZ);
                return true;
            } else {
                Serial.printf("Searching... %d sec remaining (no pad)\n",
                              remainingSec);
            }
        }

        delay(100);
    }

    Serial.println("SEARCH TIMEOUT - Pad not found!");
    return false;
}

// ══════════════════════════════════════════════════════════════
//  GO TO PAD POSITION (NEW in v6!)
//
//  Sends "go x y z speed mid" — the Tello flies to exact
//  coordinates relative to the detected mission pad.
//  Returns true if "ok", false if "error" or timeout.
//
//  REPLACES the entire centerOnPad() + descendAndLandOnPad()
//  from v4/v5 with a single command!
// ══════════════════════════════════════════════════════════════

bool goToPadPosition(int x, int y, int z, int speed) {
    // Use m-2 = nearest pad detected by downward camera (more reliable than specific ID)
    char cmd[60];
    sprintf(cmd, "go %d %d %d %d m-2", x, y, z, speed);

    Serial.printf("\n>> GO: fly to (%d, %d, %d) at %d cm/s [pad=m-2]\n",
                  x, y, z, speed);

    sendCommand(cmd);
    String resp = waitResponse(GO_TIMEOUT_MS);

    if (resp == "ok") {
        Serial.println("   GO succeeded — drone at target position!");
        return true;
    } else {
        Serial.printf("   GO failed: %s\n", resp.c_str());
        return false;
    }
}

// ══════════════════════════════════════════════════════════════
//  LAND ON PAD WITH GO COMMAND
//
//  v4/v5 approach: RC centering loop (100+ lines, oscillation)
//  v6 approach:    2 go commands + land (that's it!)
//
//  Sequence:
//    1. Enable pad detection (mon + mdirection 0)
//    2. Search for pad (hover + poll state until mid > 0)
//    3. go 0 0 60 20 m-2  (fly to 60cm directly above pad)
//    4. go 0 0 50 20 m-2  (descend to 50cm above pad)
//    5. go 0 0 40 20 m-2  (descend to 40cm above pad)
//    6. land               (gentle landing from 40cm)
// ══════════════════════════════════════════════════════════════

bool landOnPadWithGo() {
    Serial.println("\n============================================");
    Serial.println("  PAD LANDING WITH GO COMMAND (v6)");
    Serial.println("============================================\n");

    // Step 1: Enable mission pad detection
    Serial.println("[1] Enabling mission pad detection...");
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);  // downward only = 20Hz
    Serial.println("    Pad detection ON (downward camera, 20Hz)\n");

    // Step 2: Search for pad (hover and poll state)
    Serial.println("[2] Searching for pad...");
    bool found = hoverAndSearchForPad();

    if (!found) {
        Serial.println("    PAD NOT FOUND — landing wherever.");
        performLand();
        return false;
    }
    Serial.printf("    Found pad ID=%d at X:%d Y:%d Z:%dcm\n\n",
                  padId, padX, padY, padZ);

    // Step 3: Fly to HIGH position directly above pad center
    Serial.printf("[3] GO to (%d, %d, %d) — positioning above pad...\n",
                  GO_X, GO_Y, GO_HEIGHT_HIGH);
    bool ok = goToPadPosition(GO_X, GO_Y, GO_HEIGHT_HIGH, GO_SPEED);

    if (!ok) {
        Serial.println("    HIGH approach failed — landing from here.");
        performLand();
        return false;
    }

    // Brief hover to confirm position
    delay(1000);
    readState();
    Serial.printf("    Position check: pad=%d X:%d Y:%d Z:%d\n\n",
                  padId, padX, padY, padZ);

    // Step 4: Descend to MID position above pad
    Serial.printf("[4] GO to (%d, %d, %d) — descending to mid height...\n",
                  GO_X, GO_Y, GO_HEIGHT_MID);
    ok = goToPadPosition(GO_X, GO_Y, GO_HEIGHT_MID, GO_SPEED);

    if (!ok) {
        Serial.println("    MID approach failed — landing from current height.");
        performLand();
        return false;
    }
    delay(500);
    readState();
    Serial.printf("    Position check: pad=%d X:%d Y:%d Z:%d\n",
                  padId, padX, padY, padZ);

    // Step 5: Descend to LOW position above pad
    Serial.printf("\n[5] GO to (%d, %d, %d) — descending to low height...\n",
                  GO_X, GO_Y, GO_HEIGHT_LOW);
    ok = goToPadPosition(GO_X, GO_Y, GO_HEIGHT_LOW, GO_SPEED);

    if (!ok) {
        Serial.println("    LOW approach failed — landing from current height.");
    } else {
        delay(500);
        readState();
        Serial.printf("    Position check: pad=%d X:%d Y:%d Z:%d\n",
                      padId, padX, padY, padZ);
    }

    // Step 6: Land!
    Serial.println("\n[6] Landing from low position...");
    performLand();
    Serial.println("    LANDED!\n");
    return true;
}

// ══════════════════════════════════════════════════════════════
//  FULL SEQUENCE (Phase 1 movement + GO pad landing)
// ══════════════════════════════════════════════════════════════

void runFullSequence() {
    Serial.println("\n==========================================================");
    Serial.println("     FULL SEQUENCE (movement + GO pad landing)");
    Serial.println("==========================================================\n");

    // Print plan
    Serial.println("PHASE 1 MOVEMENT:");
    Serial.printf("  [2] Forward:  %dcm (safety)\n", SAFETY_FORWARD);
    Serial.printf("  [3] Right:    %dcm\n", MOVE_RIGHT_1);
    Serial.printf("  [4] Rotate:   %d deg (cw)\n", ROTATE_DEGREES);
    Serial.printf("  [5] Forward:  %dcm\n", MOVE_FORWARD_2);
    Serial.printf("  [6] Left:     %dcm\n", MOVE_LEFT_1);
    Serial.println();
    Serial.println("PHASE 2 GO LANDING:");
    Serial.printf("  High: (%d, %d, %d) at %d cm/s\n",
                  GO_X, GO_Y, GO_HEIGHT_HIGH, GO_SPEED);
    Serial.printf("  Mid:  (%d, %d, %d) at %d cm/s\n",
                  GO_X, GO_Y, GO_HEIGHT_MID, GO_SPEED);
    Serial.printf("  Low:  (%d, %d, %d) at %d cm/s\n",
                  GO_X, GO_Y, GO_HEIGHT_LOW, GO_SPEED);
    Serial.println("  Pad:  m-2 (nearest downward)");
    Serial.println();

    flightActive = true;
    char cmd[30];

    // ═══════════════════════════════════════════════════════
    // PHASE 1: Movement
    // ═══════════════════════════════════════════════════════

    Serial.println("======== PHASE 1: MOVEMENT ========\n");

    // Step 1: Takeoff
    Serial.println("[1] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    Serial.println("    Takeoff complete\n");

    // Step 2: Forward (safety)
    Serial.printf("[2] FORWARD %dcm...\n", SAFETY_FORWARD);
    sprintf(cmd, "forward %d", SAFETY_FORWARD);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // Step 3: Right
    Serial.printf("[3] RIGHT %dcm...\n", MOVE_RIGHT_1);
    sprintf(cmd, "right %d", MOVE_RIGHT_1);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // Step 4: Rotate
    Serial.printf("[4] ROTATE %d deg (cw)...\n", ROTATE_DEGREES);
    sprintf(cmd, "cw %d", ROTATE_DEGREES);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // Step 5: Forward
    Serial.printf("[5] FORWARD %dcm...\n", MOVE_FORWARD_2);
    sprintf(cmd, "forward %d", MOVE_FORWARD_2);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // Step 6: Left
    Serial.printf("[6] LEFT %dcm...\n", MOVE_LEFT_1);
    sprintf(cmd, "left %d", MOVE_LEFT_1);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // ═══════════════════════════════════════════════════════
    // PHASE 2: GO pad landing
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 2: GO PAD LANDING ========\n");
    landOnPadWithGo();

    Serial.println("\n==========================================================");
    Serial.println("              SEQUENCE COMPLETE!");
    Serial.println("==========================================================\n");
}

// ══════════════════════════════════════════════════════════════
//  QUICK GO TEST (no Phase 1 movement)
//
//  Just: takeoff → enable pad → search → go above pad → land
//  Perfect for testing the go command without flying around.
//  Place drone near/on the pad before starting.
// ══════════════════════════════════════════════════════════════

void runQuickGoTest() {
    Serial.println("\n==========================================================");
    Serial.println("     QUICK GO TEST (takeoff + go to pad + land)");
    Serial.println("==========================================================\n");

    Serial.printf("GO settings: (%d, %d, %d->%d) at %d cm/s, pad=detected ID\n",
                  GO_X, GO_Y, GO_HEIGHT_HIGH, GO_HEIGHT_LOW, GO_SPEED);
    Serial.println();

    flightActive = true;

    // Step 1: Takeoff
    Serial.println("[1] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    Serial.println("    Takeoff complete — drone hovering\n");

    // Step 2-5: Go to pad and land
    landOnPadWithGo();

    Serial.println("\n==========================================================");
    Serial.println("              QUICK GO TEST COMPLETE!");
    Serial.println("==========================================================\n");
}

// ══════════════════════════════════════════════════════════════
//                    SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n==========================================================");
    Serial.println("   TELLO SIMPLE SEQUENCE v6 — GO COMMAND PAD LANDING");
    Serial.println("==========================================================\n");

    Serial.println("v6 uses the Tello 'go x y z speed mid' command to fly");
    Serial.println("directly to coordinates above the mission pad.");
    Serial.println("No more RC centering loops! Tello handles positioning.\n");

    // Print settings
    Serial.println("WIFI:");
    Serial.printf("  SSID:     %s\n", WIFI_NAME);
    Serial.printf("  Password: %s\n", WIFI_PASSWORD);
    Serial.println();

    Serial.println("GO COMMAND SETTINGS:");
    Serial.printf("  High position: (%d, %d, %d)\n",
                  GO_X, GO_Y, GO_HEIGHT_HIGH);
    Serial.printf("  Mid position:  (%d, %d, %d)\n",
                  GO_X, GO_Y, GO_HEIGHT_MID);
    Serial.printf("  Low position:  (%d, %d, %d)\n",
                  GO_X, GO_Y, GO_HEIGHT_LOW);
    Serial.printf("  Speed:         %d cm/s\n", GO_SPEED);
    Serial.printf("  Pad:           m-2 (nearest downward)\n");
    Serial.printf("  Timeout:       %d ms\n", GO_TIMEOUT_MS);
    Serial.println();

    Serial.println("PHASE 1 MOVEMENT:");
    Serial.printf("  Safety forward:  %dcm\n", SAFETY_FORWARD);
    Serial.printf("  Right:           %dcm\n", MOVE_RIGHT_1);
    Serial.printf("  Rotate:          %d deg (cw)\n", ROTATE_DEGREES);
    Serial.printf("  Forward:         %dcm\n", MOVE_FORWARD_2);
    Serial.printf("  Left:            %dcm\n", MOVE_LEFT_1);
    Serial.println();

    // Connect WiFi
    Serial.print("Connecting to Tello");
    if (strlen(WIFI_PASSWORD) > 0) {
        WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
    } else {
        WiFi.begin(WIFI_NAME, "");
    }

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n\nWiFi FAILED!");
        Serial.println("Check Tello is ON and credentials are correct.");
        while (1) delay(1000);
    }
    Serial.println(" Connected!\n");

    // Setup UDP
    udpCmd.begin(CMD_PORT);
    udpState.begin(STATE_PORT);
    delay(1000);

    // Enter SDK mode
    sendWithRetry("command", 500);

    // Check battery
    sendCommand("battery?");
    String bat = waitResponse(3000);
    Serial.printf("Battery: %s%%\n\n", bat.c_str());

    // Print commands
    Serial.println("==========================================================");
    Serial.println("  COMMANDS:");
    Serial.println("  ---------");
    Serial.println("  s = Full sequence (movement + go land)");
    Serial.println("  g = Quick GO test (takeoff + go to pad + land)");
    Serial.println("  b = Battery check");
    Serial.println("  e = EMERGENCY (motors off!)");
    Serial.println("  l = Normal land");
    Serial.println("==========================================================\n");
}

// ══════════════════════════════════════════════════════════════
//                    LOOP
// ══════════════════════════════════════════════════════════════

void loop() {
    if (Serial.available()) {
        char c = toupper(Serial.read());

        switch (c) {
            case 'S':
                runFullSequence();
                break;

            case 'G':
                runQuickGoTest();
                break;

            case 'B':
                sendCommand("battery?");
                waitResponse();
                break;

            case 'E':
                Serial.println("\n!!! EMERGENCY !!!");
                sendCommand("emergency");
                flightActive = false;
                break;

            case 'L':
                Serial.println("\nLanding...");
                sendCommand("land");
                flightActive = false;
                break;
        }
    }
    delay(10);
}
