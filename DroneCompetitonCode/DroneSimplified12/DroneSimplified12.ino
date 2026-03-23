// ══════════════════════════════════════════════════════════════
//  TELLO v12 — GO-ONLY LANDING (no RC)
//
//  Strategy: Use ONLY SDK `go` commands to center + descend.
//  RC is useless for descent (stalls at ~48cm every time).
//
//  2-step go (go 80 was unreliable — barely moved, lost pad):
//    1. go 0 0 50 — center + descend (proven reliable in v11)
//    2. go 0 0 25 — final low position (worked in v10)
//    3. land — from ~25-35cm, minimal drift
//
//  Verify centered after each go step.
//  No RC at all — just go commands + land.
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

// ─────────────── GO COMMAND (2-step descent) ───────────────
int GO_HEIGHT_1 = 50;           // cm — center + descend (reliable in v11)
int GO_HEIGHT_2 = 25;           // cm — final low position (worked in v10)
int GO_SPEED = 15;              // cm/s

// ─────────────── CENTERING VERIFICATION ───────────────
int VERIFY_COUNT = 10;          // consecutive centered readings needed
int VERIFY_ZONE = 15;           // cm — tolerance for "centered"
int PAD_X_OFFSET = -4;          // camera X offset
int PAD_Y_OFFSET = 4;           // camera Y offset

// ─────────────── FALLBACK RC CENTERING ───────────────
int NUDGE_SPEED = 15;           // speed for fallback centering
bool INVERT_PITCH = false;
bool INVERT_ROLL = true;

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
    for (int i = 0; i < 10; i++) { readState(); delay(100); }

    if (padId <= 0) {
        Serial.println("   ERROR: No pad detected!");
        return false;
    }

    Serial.printf("   Pad: ID=%d | Pos: X:%d Y:%d Z:%d\n", padId, padX, padY, padZ);

    stopMovement();
    delay(500);

    char goCmd[50];
    sprintf(goCmd, "go 0 0 %d %d m%d", height, speed, padId);

    for (int attempt = 1; attempt <= 2; attempt++) {
        Serial.printf("   Sending (attempt %d): %s\n", attempt, goCmd);
        sendCommand(goCmd);

        String resp = waitResponse(25000);

        if (resp == "ok") {
            delay(1000);
            for (int i = 0; i < 5; i++) { readState(); delay(100); }
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
//  VERIFY CENTERED — confirm position is stable near center
// ══════════════════════════════════════════════════════════════

bool verifyCentered() {
    Serial.printf("\n   Verifying centered (%d readings, zone +/-%dcm)...\n",
                  VERIFY_COUNT, VERIFY_ZONE);
    int centeredReadings = 0;
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 10000;

    while (centeredReadings < VERIFY_COUNT && millis() - startTime < TIMEOUT) {
        if (checkEmergency()) return false;
        readState();

        if (padId > 0) {
            int errX = abs(padX - PAD_X_OFFSET);
            int errY = abs(padY - PAD_Y_OFFSET);

            if (errX <= VERIFY_ZONE && errY <= VERIFY_ZONE) {
                centeredReadings++;
            } else {
                if (centeredReadings > 0) {
                    Serial.printf("   Reset at %d/%d -- X:%d Y:%d too far\n",
                                  centeredReadings, VERIFY_COUNT, padX, padY);
                }
                centeredReadings = 0;
            }
        } else {
            centeredReadings = 0;
        }

        if (centeredReadings > 0 &&
            (centeredReadings % 5 == 0 || centeredReadings == VERIFY_COUNT)) {
            Serial.printf("   Centered [%d/%d] | X:%d Y:%d Z:%d\n",
                          centeredReadings, VERIFY_COUNT, padX, padY, padZ);
        }

        delay(100);
    }

    if (centeredReadings >= VERIFY_COUNT) {
        Serial.println("   CENTERED CONFIRMED!");
        return true;
    }

    Serial.println("   Verify timeout -- proceeding anyway");
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
    Serial.println("     v12 -- GO-ONLY LANDING");
    Serial.println("==========================================================\n");

    Serial.println("FLIGHT PLAN:");
    Serial.printf("  [1] Enable pad detection\n");
    Serial.printf("  [2] Takeoff\n");
    Serial.printf("  [3] Forward %dcm\n", MOVE_AWAY_DISTANCE);
    Serial.printf("  [4] CW %d deg\n", TURN_DEGREES);
    Serial.printf("  [5] Forward %dcm (return)\n", RETURN_DISTANCE);
    Serial.printf("  [6] go 0 0 %d (center + descend)\n", GO_HEIGHT_1);
    Serial.printf("  [7] go 0 0 %d (final low position)\n", GO_HEIGHT_2);
    Serial.printf("  [8] Land from ~%dcm\n\n", GO_HEIGHT_2);

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

    if (checkEmergency()) return;

    Serial.printf("[3] FORWARD %dcm...\n", MOVE_AWAY_DISTANCE);
    sprintf(cmd, "forward %d", MOVE_AWAY_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency()) return;

    // ═══════════════════════════════════════════════════════
    // PHASE 2: RETURN TO PAD
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 2: RETURN TO PAD ========\n");

    Serial.printf("[4] CW %d deg...\n", TURN_DEGREES);
    sprintf(cmd, "cw %d", TURN_DEGREES);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency()) return;

    Serial.printf("[5] FORWARD %dcm (returning)...\n", RETURN_DISTANCE);
    sprintf(cmd, "forward %d", RETURN_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    if (checkEmergency()) return;

    // ═══════════════════════════════════════════════════════
    // PHASE 3: CENTER + TRACK DETECTION
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 3: GO-ONLY LANDING ========\n");

    // Re-enable SDK + pad detection
    Serial.println("[5.5] Re-enabling SDK + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // ── Step 6: Wait for pad then go to 80cm ──
    Serial.printf("\n[6] GO TO CENTER at %dcm...\n", GO_HEIGHT_1);

    bool padFound = waitForPad(10000);

    if (!padFound) {
        Serial.println("\n   No pad found! Trying RC centering first...");
        if (!centerOnPadRC()) {
            Serial.println("   RC centering failed -- landing wherever.");
            performLand();
            goto sequenceDone;
        }
    }

    {
        bool go1 = goToCenter(GO_HEIGHT_1, GO_SPEED);
        if (checkEmergency()) return;
        if (!go1) {
            Serial.println("   go step 1 failed! Trying RC centering...");
            if (!centerOnPadRC()) {
                Serial.println("   RC fallback failed -- landing wherever.");
                performLand();
                goto sequenceDone;
            }
        }
    }

    verifyCentered();
    if (checkEmergency()) return;

    // ── Step 7: Go to 25cm (final low position) ──
    {
        Serial.printf("\n[7] GO TO FINAL LOW at %dcm...\n", GO_HEIGHT_2);
        bool go2 = goToCenter(GO_HEIGHT_2, GO_SPEED);
        if (checkEmergency()) return;
        if (!go2) {
            Serial.println("   go step 2 failed -- landing from current height.");
        }

        // Read final position
        delay(500);
        for (int i = 0; i < 5; i++) { readState(); delay(100); }
        Serial.printf("   Final position: X:%d Y:%d Z:%d (pad %d)\n",
                      padX, padY, padZ, padId);
    }

    // ── Step 8: Land ──
    {
        Serial.printf("\n[8] LANDING from ~%dcm...\n", GO_HEIGHT_2);
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
    Serial.println("   TELLO v12 -- GO-ONLY LANDING (no RC descent)");
    Serial.println("==========================================================\n");

    Serial.println("2-step go descent: center+descend, then final low, then land.");
    Serial.println("No RC — it stalls at ~48cm and is useless for descent.\n");

    // Print settings
    Serial.println("SETTINGS:");
    Serial.printf("  Go step 1:     go 0 0 %d %d m<id>\n", GO_HEIGHT_1, GO_SPEED);
    Serial.printf("  Go step 2:     go 0 0 %d %d m<id>\n", GO_HEIGHT_2, GO_SPEED);
    Serial.printf("  Verify:        %d readings, zone +/-%dcm\n", VERIFY_COUNT, VERIFY_ZONE);
    Serial.printf("  Offsets:       X=%d Y=%d\n", PAD_X_OFFSET, PAD_Y_OFFSET);
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
    Serial.println("  s = START sequence");
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
