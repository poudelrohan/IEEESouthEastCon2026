// ══════════════════════════════════════════════════════════════
//  TELLO v13 — GRADUAL GO DESCENT + RC FINE-TUNE
//
//  Strategy: 5-step gradual `go` descent for centering,
//  then RC fine-tune for last few cm, then land.
//
//  5-step go: 60 → 50 → 40 → 30 → 25 at speed 15
//  GO_X_OFFSET = -5 compensates for consistent +X drift
//  RC fine-tune closes the last gap before landing
//
//  CW 180 before centering so drone faces original direction.
//
//  Commands: S=Start  B=Battery  E=Emergency  L=Land
// ══════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <WiFiUdp.h>

// ══════════════════════════════════════════════════════════════
//                    EASY TO CHANGE VARIABLES
// ══════════════════════════════════════════════════════════════

// ─────────────── WIFI ───────────────
const char* WIFI_NAME = "TELLO-BCU-DRONE";
const char* WIFI_PASSWORD = "southeast2026";

// ─────────────── FLIGHT PATH ───────────────
int MOVE_AWAY_DISTANCE = 40;    // cm forward (15" = 38cm, 40 for margin)
int TURN_DEGREES = 180;         // CW turn
int RETURN_DISTANCE = 40;       // cm forward after turn

// ─────────────── GO COMMAND (gradual descent) ───────────────
int GO_HEIGHTS[] = {60, 50, 40, 30, 25};
int NUM_GO_STEPS = 5;
int GO_SPEED = 15;              // cm/s — slow for accuracy
int GO_X_OFFSET = -5;           // compensate +X drift
int GO_Y_OFFSET = 0;

// ─────────────── RC FINE-TUNING ───────────────
int FINE_SPEED = 10;
int FINE_ZONE = 8;              // cm — tolerance for "centered"
int FINE_COUNT = 5;             // consecutive readings needed
int PAD_X_OFFSET = 0;
int PAD_Y_OFFSET = 0;
bool INVERT_PITCH = false;
bool INVERT_ROLL = true;

// ─────────────── FALLBACK RC CENTERING ───────────────
int NUDGE_SPEED = 15;
int VERIFY_ZONE = 15;
int VERIFY_COUNT = 5;

// ─────────────── TIMING ───────────────
int STABILIZE_AFTER_TAKEOFF = 3000;
int PAUSE_AFTER_MOVE = 1500;
int COMMAND_RETRY_DELAY = 500;

// ─────────────── LANDING ───────────────
bool USE_EMERGENCY_LAND = false;

// ══════════════════════════════════════════════════════════════
//                    NETWORK (don't change)
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
    if (idx != -1) padId = latest.substring(idx + 4, latest.indexOf(";", idx)).toInt();

    idx = latest.indexOf(";x:");
    if (idx != -1) padX = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();

    idx = latest.indexOf(";y:");
    if (idx != -1) padY = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();

    idx = latest.indexOf(";z:");
    if (idx != -1) padZ = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();

    idx = latest.indexOf("bat:");
    if (idx != -1) batteryLevel = latest.substring(idx + 4, latest.indexOf(";", idx)).toInt();

    return true;
}

// ══════════════════════════════════════════════════════════════
//                    RC CONTROL
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
//                    LANDING
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
// ══════════════════════════════════════════════════════════════

bool goToCenter(int height, int speed) {
    for (int i = 0; i < 3; i++) { readState(); delay(50); }

    if (padId <= 0) {
        Serial.println("   ERROR: No pad detected!");
        return false;
    }

    Serial.printf("   Pad: ID=%d | Pos: X:%d Y:%d Z:%d\n", padId, padX, padY, padZ);
    stopMovement();

    char goCmd[50];
    sprintf(goCmd, "go %d %d %d %d m%d", GO_X_OFFSET, GO_Y_OFFSET, height, speed, padId);

    for (int attempt = 1; attempt <= 2; attempt++) {
        Serial.printf("   Sending (attempt %d): %s\n", attempt, goCmd);
        sendCommand(goCmd);

        String resp = waitResponse(25000);

        if (resp == "ok") {
            delay(200);
            for (int i = 0; i < 3; i++) { readState(); delay(50); }
            Serial.printf("   After go: X:%d Y:%d Z:%d (pad %d)\n",
                          padX, padY, padZ, padId);
            return true;
        }

        Serial.printf("   Attempt %d failed: %s\n", attempt, resp.c_str());

        if (attempt < 2) {
            Serial.println("   Re-entering SDK mode...");
            sendWithRetry("command", 500);
            sendWithRetry("mon", 500);
            sendWithRetry("mdirection 0", 500);
            delay(1000);
            for (int i = 0; i < 5; i++) { readState(); delay(100); }
            if (padId <= 0) {
                Serial.println("   Pad lost after retry!");
                return false;
            }
        }
    }

    Serial.println("   go command failed after 2 attempts!");
    return false;
}

// ══════════════════════════════════════════════════════════════
//  WAIT FOR PAD DETECTION
// ══════════════════════════════════════════════════════════════

bool waitForPad(int timeoutMs) {
    Serial.printf("   Waiting for pad (%ds timeout)...\n", timeoutMs / 1000);
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
//  RC FINE-TUNING — nudge into tight zone before landing
// ══════════════════════════════════════════════════════════════

bool fineCenterRC() {
    Serial.printf("\n   RC fine-tuning (zone +/-%dcm, speed %d, need %d)...\n",
                  FINE_ZONE, FINE_SPEED, FINE_COUNT);

    int centeredCount = 0;
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 10000;

    while (millis() - startTime < TIMEOUT && flightActive) {
        if (checkEmergency()) return false;
        readState();

        if (padId <= 0) { centeredCount = 0; delay(100); continue; }

        int errorX = padX - PAD_X_OFFSET;
        int errorY = padY - PAD_Y_OFFSET;
        bool centered = abs(errorX) <= FINE_ZONE && abs(errorY) <= FINE_ZONE;

        if (centered) {
            centeredCount++;
            stopMovement();
        } else {
            centeredCount = 0;
            int roll = 0, pitch = 0;
            if (errorX > FINE_ZONE) pitch = -FINE_SPEED;
            else if (errorX < -FINE_ZONE) pitch = FINE_SPEED;
            if (errorY > FINE_ZONE) roll = -FINE_SPEED;
            else if (errorY < -FINE_ZONE) roll = FINE_SPEED;
            if (INVERT_PITCH) pitch = -pitch;
            if (INVERT_ROLL) roll = -roll;
            sendRC(roll, pitch, 0, 0);
        }

        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            Serial.printf("   X:%d Y:%d Z:%d | err X:%d Y:%d | %s",
                          padX, padY, padZ, errorX, errorY,
                          centered ? "LOCKED" : "nudging");
            if (centered) Serial.printf(" [%d/%d]", centeredCount, FINE_COUNT);
            Serial.println();
        }

        if (centeredCount >= FINE_COUNT) {
            Serial.printf("   FINE-TUNED! X:%d Y:%d Z:%d\n", padX, padY, padZ);
            stopMovement();
            return true;
        }
        delay(50);
    }
    Serial.println("   Fine-tune timeout -- landing anyway");
    stopMovement();
    return false;
}

// ══════════════════════════════════════════════════════════════
//  FALLBACK: RC CENTERING (if go command fails)
// ══════════════════════════════════════════════════════════════

bool centerOnPadRC() {
    Serial.println("\n--- FALLBACK: RC centering ---");
    Serial.printf("   Zone: +/-%dcm | Speed: %d | Need: %d confirms\n",
                  VERIFY_ZONE, NUDGE_SPEED, VERIFY_COUNT);

    int centeredCount = 0;
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
        bool centered = abs(errorX) <= VERIFY_ZONE && abs(errorY) <= VERIFY_ZONE;

        if (centered) {
            centeredCount++;
        } else {
            centeredCount = 0;
        }

        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            Serial.printf("   X:%d Y:%d Z:%d | err X:%d Y:%d | %s",
                          padX, padY, padZ, errorX, errorY,
                          centered ? "OK" : "Adjusting");
            if (centered) Serial.printf(" [%d/%d]", centeredCount, VERIFY_COUNT);
            Serial.println();
        }

        if (centeredCount >= VERIFY_COUNT) {
            Serial.println("   RC centering complete!");
            stopMovement();
            return true;
        }

        int roll = 0, pitch = 0;
        if (errorX > VERIFY_ZONE) pitch = -NUDGE_SPEED;
        else if (errorX < -VERIFY_ZONE) pitch = NUDGE_SPEED;
        if (errorY > VERIFY_ZONE) roll = -NUDGE_SPEED;
        else if (errorY < -VERIFY_ZONE) roll = NUDGE_SPEED;

        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL) roll = -roll;

        sendRC(roll, pitch, 0, 0);
        delay(100);
    }

    Serial.println("   RC centering timeout!");
    stopMovement();
    return false;
}

// ══════════════════════════════════════════════════════════════
//                    MAIN SEQUENCE
// ══════════════════════════════════════════════════════════════

void runSequence() {
    Serial.println("\n==========================================================");
    Serial.println("     v13 -- GRADUAL GO DESCENT + RC FINE-TUNE");
    Serial.println("==========================================================\n");

    Serial.println("FLIGHT PLAN:");
    Serial.printf("  [1]  Enable pad detection\n");
    Serial.printf("  [2]  Takeoff\n");
    Serial.printf("  [3]  Forward %dcm\n", MOVE_AWAY_DISTANCE);
    Serial.printf("  [4]  CW %d deg\n", TURN_DEGREES);
    Serial.printf("  [5]  Forward %dcm (return)\n", RETURN_DISTANCE);
    Serial.printf("  [6]  CW %d deg (face original direction)\n", TURN_DEGREES);
    Serial.printf("  [7-11] Go descent: ");
    for (int i = 0; i < NUM_GO_STEPS; i++) Serial.printf("%d ", GO_HEIGHTS[i]);
    Serial.printf("(offset X:%d Y:%d)\n", GO_X_OFFSET, GO_Y_OFFSET);
    Serial.printf("  [12] RC fine-tune (zone +/-%d, speed %d)\n", FINE_ZONE, FINE_SPEED);
    Serial.printf("  [13] Land\n\n");

    flightActive = true;
    char cmd[30];

    // ═══════════════════════════════════════════════════════
    // PHASE 1: TAKEOFF + MOVE AWAY
    // ═══════════════════════════════════════════════════════

    Serial.println("======== PHASE 1: TAKEOFF + MOVE AWAY ========\n");

    Serial.println("[1] Enabling SDK + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);
    Serial.println("    Pad detection ON (downward, 20Hz)\n");

    Serial.println("[2] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("    TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    Serial.println("    Takeoff complete\n");

    if (checkEmergency() || !flightActive) return;

    Serial.printf("[3] FORWARD %dcm...\n", MOVE_AWAY_DISTANCE);
    sprintf(cmd, "forward %d", MOVE_AWAY_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency() || !flightActive) return;

    // ═══════════════════════════════════════════════════════
    // PHASE 2: RETURN TO PAD
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 2: RETURN TO PAD ========\n");

    Serial.printf("[4] CW %d deg...\n", TURN_DEGREES);
    sprintf(cmd, "cw %d", TURN_DEGREES);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency() || !flightActive) return;

    Serial.printf("[5] FORWARD %dcm (returning)...\n", RETURN_DISTANCE);
    sprintf(cmd, "forward %d", RETURN_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency() || !flightActive) return;

    // ═══════════════════════════════════════════════════════
    // PHASE 3: FACE ORIGINAL + GO DESCENT + RC FINE + LAND
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 3: PRECISION LANDING ========\n");

    // CW 180 to face original direction
    Serial.printf("[6] CW %d (face original direction)...\n", TURN_DEGREES);
    sprintf(cmd, "cw %d", TURN_DEGREES);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency() || !flightActive) return;

    // Re-enable SDK + pad detection
    Serial.println("[6.5] Re-enabling SDK + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // Wait for pad
    Serial.println("\n[7] Looking for pad...");
    bool padFound = waitForPad(10000);

    if (!padFound) {
        Serial.println("\n   No pad found! Trying RC centering first...");
        if (!centerOnPadRC()) {
            Serial.println("   RC centering failed -- landing wherever.");
            performLand();
            goto sequenceDone;
        }
    }
    if (!flightActive) return;

    // Go steps: gradual descent
    {
        Serial.printf("\n   GO descent: ");
        for (int i = 0; i < NUM_GO_STEPS; i++) Serial.printf("%d ", GO_HEIGHTS[i]);
        Serial.println();

        for (int i = 0; i < NUM_GO_STEPS; i++) {
            Serial.printf("\n[%d] GO to %dcm...\n", 7 + i, GO_HEIGHTS[i]);

            bool goOk = goToCenter(GO_HEIGHTS[i], GO_SPEED);
            if (checkEmergency() || !flightActive) return;

            if (!goOk && i == 0) {
                Serial.println("   First go failed! Trying RC centering...");
                if (!centerOnPadRC()) {
                    Serial.println("   Failed -- landing wherever.");
                    performLand();
                    goto sequenceDone;
                }
            } else if (!goOk) {
                Serial.printf("   go step %d failed -- proceeding.\n", i + 1);
            }

            if (checkEmergency() || !flightActive) return;
        }
    }

    // RC fine-tune
    Serial.println("\n[12] RC fine-tuning...");
    fineCenterRC();
    if (checkEmergency() || !flightActive) return;

    // Final position
    {
        delay(500);
        for (int i = 0; i < 5; i++) { readState(); delay(100); }
        Serial.printf("   Final position: X:%d Y:%d Z:%d (pad %d)\n",
                      padX, padY, padZ, padId);
    }

    // Land
    {
        Serial.println("\n[13] LANDING...");
        performLand();
    }

sequenceDone:
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
    Serial.println("   TELLO v13 -- GRADUAL GO DESCENT + RC FINE-TUNE");
    Serial.println("==========================================================\n");

    Serial.println("5-step go descent with X offset compensation + RC fine-tune.");
    Serial.println("CW 180 before centering so drone faces original direction.\n");

    // Print settings
    Serial.println("SETTINGS:");
    Serial.printf("  Go heights:    ");
    for (int i = 0; i < NUM_GO_STEPS; i++) Serial.printf("%d ", GO_HEIGHTS[i]);
    Serial.printf("at speed %d\n", GO_SPEED);
    Serial.printf("  Go offset:     X=%d Y=%d\n", GO_X_OFFSET, GO_Y_OFFSET);
    Serial.printf("  Fine-tune:     zone +/-%d, speed %d, need %d\n",
                  FINE_ZONE, FINE_SPEED, FINE_COUNT);
    Serial.printf("  Pad offsets:   X=%d Y=%d\n", PAD_X_OFFSET, PAD_Y_OFFSET);
    Serial.printf("  Land mode:     %s\n", USE_EMERGENCY_LAND ? "EMERGENCY" : "NORMAL");
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

    // Enter SDK mode + pad detection
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);
    Serial.println("Pad detection ON\n");

    // Battery check
    sendCommand("battery?");
    String bat = waitResponse(3000);
    Serial.printf("Battery: %s%%\n\n", bat.c_str());

    // Commands
    Serial.println("==========================================================");
    Serial.println("  COMMANDS:");
    Serial.println("  S = START sequence");
    Serial.println("  B = Battery check");
    Serial.println("  E = EMERGENCY (motors off!)");
    Serial.println("  L = Normal land");
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
                runSequence();
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
