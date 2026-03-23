// ══════════════════════════════════════════════════════════════
//  TELLO v10 — LAND ON PAD USING SDK `go` COMMAND
//
//  NEW APPROACH: Instead of manual RC centering (which oscillates,
//  needs INVERT flags, and is hard to tune), we use the Tello's
//  built-in `go x y z speed mid` command. This tells the Tello
//  to fly to coordinates relative to the mission pad center using
//  its own internal PID controller.
//
//  `go 0 0 50 10 m1` = fly to pad center at 50cm, speed 10 cm/s
//
//  Flight path:
//    1. Enable pad detection (mon + mdirection 0)
//    2. Takeoff (~88cm)
//    3. Forward 40cm — move away 15"+
//    4. CW 180° — turn around
//    5. Forward 40cm — walk back to start
//    6. go 0 0 50 — snap to pad center at 50cm (coarse)
//    7. go 0 0 30 — descend + re-center at 30cm (fine)
//    8. land — from ~30cm, minimal drift
//
//  If `go` fails (pad not detected), falls back to v4-style
//  simple RC centering as backup.
//
//  Commands:
//    S = Full sequence
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

// ─────────────── MOVEMENT (cm / degrees) ───────────────
int MOVE_AWAY_DISTANCE = 40;    // cm forward (15" = 38cm, 40 for margin)
int TURN_DEGREES = 180;         // CW turn to face back toward pad
int RETURN_DISTANCE = 40;       // cm forward after turn (back to start)

// ─────────────── GO COMMAND SETTINGS ───────────────
int GO_HEIGHT_1 = 50;           // cm, first go — coarse center
int GO_HEIGHT_2 = 30;           // cm, second go — fine center + descend (30 keeps pad detectable)
int GO_SPEED = 15;              // cm/s (15 = fast enough to not timeout, still accurate)

// ─────────────── TIMING (ms) ───────────────
int STABILIZE_AFTER_TAKEOFF = 3000;
int PAUSE_AFTER_MOVE = 1500;
int COMMAND_RETRY_DELAY = 500;

// ─────────────── FALLBACK RC SETTINGS (from v4) ───────────────
// Only used if `go` command fails (pad not detected)
int PAD_X_OFFSET = -4;
int PAD_Y_OFFSET = 4;
int TARGET_ZONE = 15;
int CENTERED_COUNT_NEEDED = 25;
int LAND_HEIGHT = 45;
int NUDGE_SPEED = 15;
bool INVERT_PITCH = false;
bool INVERT_ROLL = true;

// ─────────────── LANDING STYLE ───────────────
bool USE_EMERGENCY_LAND = false;

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
int centeredCount = 0;
unsigned long lastStateRead = 0;
unsigned long lastPrintTime = 0;
int batteryLevel = 0;

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
//                    RC CONTROL (for fallback only)
// ══════════════════════════════════════════════════════════════

void sendRC(int roll, int pitch, int throttle, int yaw) {
    char cmd[50];
    sprintf(cmd, "rc %d %d %d %d", roll, pitch, throttle, yaw);
    sendCommand(cmd);
}

void stopMovement() {
    sendRC(0, 0, 0, 0);
    delay(100);
}

bool checkEmergency() {
    if (Serial.available()) {
        char c = toupper(Serial.read());
        if (c == 'E') {
            Serial.println("\n!!! EMERGENCY !!!");
            sendCommand("emergency");
            flightActive = false;
            return true;
        }
        if (c == 'L') {
            Serial.println("\nManual land...");
            sendCommand("land");
            flightActive = false;
            return true;
        }
    }
    return false;
}

// ══════════════════════════════════════════════════════════════
//                    LANDING FUNCTIONS
// ══════════════════════════════════════════════════════════════

void performLand() {
    stopMovement();
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
//  GO TO PAD CENTER — uses SDK `go` command
//
//  Sends: go 0 0 <height> <speed> m<padId>
//  The Tello flies to (0,0,height) relative to the detected pad.
//  Returns true if "ok", false if error/timeout.
// ══════════════════════════════════════════════════════════════

bool goToCenter(int height, int speed) {
    // Flush and read fresh state to get current pad ID
    for (int i = 0; i < 10; i++) {
        readState();
        delay(100);
    }

    if (padId <= 0) {
        Serial.println("   ERROR: No pad detected! Cannot use go command.");
        return false;
    }

    Serial.printf("   Pad detected: ID=%d | Current pos: X:%d Y:%d Z:%d\n",
                  padId, padX, padY, padZ);

    // Ensure drone is fully stopped before go command
    stopMovement();
    delay(500);

    // Build the go command: go 0 0 <height> <speed> m<padId>
    char goCmd[50];
    sprintf(goCmd, "go 0 0 %d %d m%d", height, speed, padId);

    // Try up to 2 times (UDP can drop packets)
    for (int attempt = 1; attempt <= 2; attempt++) {
        Serial.printf("   Sending (attempt %d): %s\n", attempt, goCmd);
        sendCommand(goCmd);

        // Wait for completion — go can take 10-20s for long moves
        String resp = waitResponse(25000);

        if (resp == "ok") {
            // Read position after go completes
            delay(1000);
            for (int i = 0; i < 5; i++) { readState(); delay(100); }
            Serial.printf("   After go: X:%d Y:%d Z:%d (pad %d)\n",
                          padX, padY, padZ, padId);
            return true;
        }

        Serial.printf("   Attempt %d failed! Response: %s\n", attempt, resp.c_str());

        if (attempt < 2) {
            // Re-enter SDK mode and retry
            Serial.println("   Re-entering SDK mode and retrying...");
            sendWithRetry("command", 500);
            sendWithRetry("mon", 500);
            sendWithRetry("mdirection 0", 500);
            delay(1000);

            // Re-read pad state
            for (int i = 0; i < 5; i++) { readState(); delay(100); }
            if (padId <= 0) {
                Serial.println("   Pad lost after retry setup!");
                return false;
            }
        }
    }

    Serial.println("   go command failed after 2 attempts!");
    return false;
}

// ══════════════════════════════════════════════════════════════
//  WAIT FOR PAD DETECTION
//
//  Reads state repeatedly until pad is detected or timeout.
//  Returns true if pad found.
// ══════════════════════════════════════════════════════════════

bool waitForPad(int timeoutMs) {
    Serial.printf("   Waiting for pad detection (%ds timeout)...\n", timeoutMs / 1000);
    unsigned long start = millis();

    while (millis() - start < (unsigned long)timeoutMs) {
        if (checkEmergency()) return false;
        readState();
        if (padId > 0) {
            Serial.printf("   PAD FOUND! ID=%d | X:%d Y:%d Z:%d\n",
                          padId, padX, padY, padZ);
            return true;
        }
        delay(100);
    }

    Serial.println("   Pad detection timeout!");
    return false;
}

// ══════════════════════════════════════════════════════════════
//  FALLBACK: V4-STYLE RC CENTERING
//
//  Simple continuous RC at fixed speed. Only used if `go` fails.
// ══════════════════════════════════════════════════════════════

bool centerOnPadRC() {
    Serial.println("\n--- FALLBACK: RC centering (v4 style) ---");
    Serial.printf("   Target zone: +/-%dcm | Speed: %d | Need %d confirms\n",
                  TARGET_ZONE, NUDGE_SPEED, CENTERED_COUNT_NEEDED);

    centeredCount = 0;
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 20000;

    while (millis() - startTime < TIMEOUT && flightActive) {
        if (checkEmergency()) return false;

        readState();

        if (padId <= 0) {
            Serial.println("   Lost pad! Hovering...");
            stopMovement();
            centeredCount = 0;
            delay(500);
            continue;
        }

        int errorX = padX - PAD_X_OFFSET;
        int errorY = padY - PAD_Y_OFFSET;

        bool centered = abs(errorX) <= TARGET_ZONE && abs(errorY) <= TARGET_ZONE;

        if (centered) {
            centeredCount++;
        } else {
            centeredCount = 0;
        }

        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            Serial.printf("   PAD:%d | X:%d Y:%d Z:%d | Err X:%d Y:%d | %s",
                          padId, padX, padY, padZ, errorX, errorY,
                          centered ? "OK" : "Adjusting");
            if (centered) {
                Serial.printf(" [%d/%d]", centeredCount, CENTERED_COUNT_NEEDED);
            }
            Serial.println();
        }

        if (centeredCount >= CENTERED_COUNT_NEEDED) {
            Serial.println("   RC centering complete!");
            stopMovement();
            return true;
        }

        int roll = 0, pitch = 0;
        if (errorX > TARGET_ZONE) pitch = -NUDGE_SPEED;
        else if (errorX < -TARGET_ZONE) pitch = NUDGE_SPEED;
        if (errorY > TARGET_ZONE) roll = -NUDGE_SPEED;
        else if (errorY < -TARGET_ZONE) roll = NUDGE_SPEED;

        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL) roll = -roll;

        sendRC(roll, pitch, 0, 0);
        delay(100);
    }

    Serial.println("   RC centering timeout!");
    stopMovement();
    return false;
}

bool descendAndLandRC() {
    Serial.println("\n--- FALLBACK: SDK descent (RC can't go below ~47cm!) ---");

    // Read current height
    delay(1000);
    for (int i = 0; i < 10; i++) { readState(); delay(100); }
    int currentHeight = abs(padZ);
    if (currentHeight == 0) currentHeight = 65;  // Fallback estimate
    Serial.printf("   Current height: ~%dcm\n", currentHeight);

    // Use SDK down commands to descend (RC throttle gets stuck at ~47cm)
    // Step 1: down to ~40cm if high enough
    int dropAmount = currentHeight - 40;
    if (dropAmount >= 20) {
        char cmd[30];
        sprintf(cmd, "down %d", dropAmount);
        Serial.printf("   SDK: %s\n", cmd);
        sendWithRetry(cmd, 0);
        delay(2000);
        for (int i = 0; i < 5; i++) { readState(); delay(100); }
        Serial.printf("   After descent: Z=%d | X:%d Y:%d\n", padZ, padX, padY);
    } else if (dropAmount > 0) {
        // Less than 20cm — do a short RC burst (best we can do)
        Serial.printf("   Only %dcm to drop, short RC burst...\n", dropAmount);
        for (int i = 0; i < 15; i++) {
            if (checkEmergency()) return false;
            sendRC(0, 0, -20, 0);
            delay(100);
        }
        stopMovement();
        delay(1000);
    }

    if (checkEmergency()) return false;

    // Land from ~40cm
    Serial.println("   Landing from current position...");
    performLand();
    return true;
}

// ══════════════════════════════════════════════════════════════
//                    MAIN SEQUENCE
// ══════════════════════════════════════════════════════════════

void runSimpleSequence() {
    Serial.println("\n==========================================================");
    Serial.println("     v10 — PAD LANDING USING SDK `go` COMMAND");
    Serial.println("==========================================================\n");

    Serial.println("FLIGHT PLAN:");
    Serial.printf("  [1] Enable pad detection\n");
    Serial.printf("  [2] Takeoff\n");
    Serial.printf("  [3] Forward %dcm (move away)\n", MOVE_AWAY_DISTANCE);
    Serial.printf("  [4] CW %d deg (turn around)\n", TURN_DEGREES);
    Serial.printf("  [5] Forward %dcm (return)\n", RETURN_DISTANCE);
    Serial.printf("  [6] go 0 0 %d %d m<id> (center at %dcm)\n", GO_HEIGHT_1, GO_SPEED, GO_HEIGHT_1);
    Serial.printf("  [7] go 0 0 %d %d m<id> (descend to %dcm)\n", GO_HEIGHT_2, GO_SPEED, GO_HEIGHT_2);
    Serial.printf("  [8] land\n");
    Serial.println();

    flightActive = true;
    char cmd[30];

    // ═══════════════════════════════════════════════════════
    // PHASE 1: TAKEOFF + MOVE AWAY
    // ═══════════════════════════════════════════════════════

    Serial.println("======== PHASE 1: TAKEOFF + MOVE AWAY ========\n");

    // [1] Enable everything before takeoff
    Serial.println("[1] Enabling SDK mode + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);
    Serial.println("    Pad detection ON (downward, 20Hz)\n");

    // [2] Takeoff
    Serial.println("[2] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("    TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    Serial.println("    Takeoff complete (~88cm)\n");

    if (checkEmergency()) return;

    // [3] Forward — move away
    Serial.printf("[3] FORWARD %dcm (moving away)...\n", MOVE_AWAY_DISTANCE);
    sprintf(cmd, "forward %d", MOVE_AWAY_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency()) return;

    // ═══════════════════════════════════════════════════════
    // PHASE 2: RETURN TO PAD
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 2: RETURN TO PAD ========\n");

    // [4] Turn around
    Serial.printf("[4] CW %d deg (turning around)...\n", TURN_DEGREES);
    sprintf(cmd, "cw %d", TURN_DEGREES);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency()) return;

    // [5] Forward — back to start
    Serial.printf("[5] FORWARD %dcm (returning)...\n", RETURN_DISTANCE);
    sprintf(cmd, "forward %d", RETURN_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency()) return;

    // ═══════════════════════════════════════════════════════
    // PHASE 3: PRECISION LANDING
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 3: PRECISION LANDING ========\n");

    // Re-enable SDK + pad detection (in case it got confused)
    Serial.println("[5.5] Re-enabling SDK mode + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // Wait for pad to be detected
    Serial.printf("\n[6] CENTERING — go to pad center at %dcm...\n", GO_HEIGHT_1);

    bool padFound = waitForPad(10000);

    if (padFound) {
        // ── PRIMARY: Use `go` command ──
        Serial.printf("\n   Using SDK go command (center at %dcm)...\n", GO_HEIGHT_1);
        bool centered1 = goToCenter(GO_HEIGHT_1, GO_SPEED);

        if (checkEmergency()) return;

        if (centered1) {
            // Second go — descend + re-center at lower height
            Serial.printf("\n[7] DESCENDING — go to pad center at %dcm...\n", GO_HEIGHT_2);
            bool centered2 = goToCenter(GO_HEIGHT_2, GO_SPEED);

            if (checkEmergency()) return;

            if (centered2) {
                // Land from low altitude
                Serial.printf("\n[8] LANDING from ~%dcm...\n", GO_HEIGHT_2);
                performLand();
            } else {
                // Second go failed — try landing anyway (we're at ~50cm centered)
                Serial.println("\n   Second go failed — landing from current position...");
                performLand();
            }
        } else {
            // First go failed — try fallback RC centering, then go for descent
            Serial.println("\n   go command failed! Trying RC fallback...");
            if (centerOnPadRC()) {
                // RC centered! Now try go for descent (drone is centered, pad visible)
                Serial.println("\n   RC centered. Trying go command for descent...");
                bool goDown = goToCenter(GO_HEIGHT_2, GO_SPEED);
                if (goDown) {
                    Serial.printf("\n   go descent worked! Landing from ~%dcm...\n", GO_HEIGHT_2);
                    performLand();
                } else {
                    // go still failing — use SDK down commands
                    Serial.println("   go descent also failed. Using SDK down...");
                    descendAndLandRC();
                }
            } else {
                Serial.println("   RC fallback also failed — landing wherever...");
                performLand();
            }
        }
    } else {
        // No pad detected at all — try RC fallback
        Serial.println("\n   No pad detected! Trying RC fallback...");
        if (centerOnPadRC()) {
            Serial.println("\n   RC centered. Trying go command for descent...");
            bool goDown = goToCenter(GO_HEIGHT_2, GO_SPEED);
            if (goDown) {
                Serial.printf("\n   go descent worked! Landing from ~%dcm...\n", GO_HEIGHT_2);
                performLand();
            } else {
                Serial.println("   go descent also failed. Using SDK down...");
                descendAndLandRC();
            }
        } else {
            Serial.println("   RC fallback also failed — landing wherever...");
            performLand();
        }
    }

    Serial.println("\n==========================================================");
    Serial.println("              SEQUENCE COMPLETE!");
    Serial.println("==========================================================\n");
}

// ══════════════════════════════════════════════════════════════
//                    SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n==========================================================");
    Serial.println("   TELLO v10 — PAD LANDING WITH SDK `go` COMMAND");
    Serial.println("==========================================================\n");

    Serial.println("NEW: Uses `go 0 0 z speed mid` to fly to pad center.");
    Serial.println("Tello's internal PID handles centering — no RC needed.");
    Serial.println("Fallback: v4-style RC centering if go command fails.");
    Serial.println();

    // Print settings
    Serial.println("WIFI:");
    Serial.printf("  SSID:     %s\n", WIFI_NAME);
    Serial.printf("  Password: %s\n", WIFI_PASSWORD);
    Serial.println();

    Serial.println("MOVEMENT:");
    Serial.printf("  Move away:       %dcm forward\n", MOVE_AWAY_DISTANCE);
    Serial.printf("  Turn:            %d deg CW\n", TURN_DEGREES);
    Serial.printf("  Return:          %dcm forward\n", RETURN_DISTANCE);
    Serial.println();

    Serial.println("GO COMMAND LANDING:");
    Serial.printf("  Step 1:  go 0 0 %d %d m<id> (coarse center)\n", GO_HEIGHT_1, GO_SPEED);
    Serial.printf("  Step 2:  go 0 0 %d %d m<id> (fine + descend)\n", GO_HEIGHT_2, GO_SPEED);
    Serial.printf("  Step 3:  land (from ~%dcm)\n", GO_HEIGHT_2);
    Serial.println();

    Serial.println("FALLBACK RC (if go fails):");
    Serial.printf("  Target zone:     +/-%dcm\n", TARGET_ZONE);
    Serial.printf("  Nudge speed:     %d\n", NUDGE_SPEED);
    Serial.printf("  Land height:     %dcm\n", LAND_HEIGHT);
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

    // Enter SDK mode + enable pad detection
    sendWithRetry("command", 500);
    Serial.println("Enabling mission pad detection...");
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);
    Serial.println("Pad detection ON\n");

    // Check battery
    sendCommand("battery?");
    String bat = waitResponse(3000);
    Serial.printf("Battery: %s%%\n\n", bat.c_str());

    // Print commands
    Serial.println("==========================================================");
    Serial.println("  COMMANDS:");
    Serial.println("  ---------");
    Serial.println("  s = START sequence (takeoff -> move -> go center -> land)");
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
                runSimpleSequence();
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
