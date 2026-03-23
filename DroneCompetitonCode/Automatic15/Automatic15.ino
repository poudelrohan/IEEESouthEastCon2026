// ══════════════════════════════════════════════════════════════
//  TELLO AUTOMATIC 15 — CENTER FIRST, THEN DESCEND
//
//  Same as v14 but fixes landing: centers at current height
//  BEFORE descending. v14 tried to center+descend simultaneously
//  with a 48cm offset which failed badly.
//
//  LANDING SEQUENCE:
//    1. Find pad → read current Z
//    2. go to pad center at CURRENT Z (center only, no descent)
//    3. go descent: 60 → 50 → 40 → 30 → 25 (already centered)
//    4. RC fine-tune
//    5. Land
//
//  TIMELINE (from power-on):
//    T=0s:   ESP boots, connects WiFi + SDK mode
//    T=27s:  Takeoff (robot at middle-left)
//    T=27s:  Forward 40 + CW 180 + forward 40 + CW 180 + land
//    T=~55s: On ground, facing original direction. Robot doing crater.
//    T=89s:  Takeoff again (robot finishing crater)
//    T=89s:  Right 20 + hover (already facing original dir)
//    T=118s: Robot back at start. Center + descend + land
//
//  Emergency: E or L via serial if connected
// ══════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <WiFiUdp.h>

// ══════════════════════════════════════════════════════════════
//                    EASY TO CHANGE VARIABLES
// ══════════════════════════════════════════════════════════════

// ─────────────── WIFI ───────────────
const char* WIFI_NAME = "TELLO-BCU-DRONE";
const char* WIFI_PASSWORD = "southeast2026";

// ─────────────── TIMING (milliseconds from power-on) ───────────────
unsigned long TAKEOFF_1_TIME = 27000;   // T=27s: first takeoff
unsigned long TAKEOFF_2_TIME = 89000;   // T=89s: second takeoff
unsigned long LAND_PHASE_TIME = 118000; // T=118s: start precision landing

// ─────────────── FLIGHT 1 PATH ───────────────
int MOVE_AWAY_DISTANCE = 40;    // cm forward
int TURN_DEGREES = 180;         // CW turn
int RETURN_DISTANCE = 40;       // cm forward after turn

// ─────────────── FLIGHT 2 REPOSITION ───────────────
int REPOSITION_RIGHT = 20;      // cm right after CW 180

// ─────────────── GO COMMAND (gradual descent) ───────────────
int GO_HEIGHTS[] = {60, 50, 40, 30, 25};
int NUM_GO_STEPS = 5;
int GO_SPEED = 15;
int GO_X_OFFSET = -5;
int GO_Y_OFFSET = 0;

// ─────────────── RC FINE-TUNING ───────────────
int FINE_SPEED = 10;
int FINE_ZONE = 8;
int FINE_COUNT = 5;
int PAD_X_OFFSET = 0;
int PAD_Y_OFFSET = 0;
bool INVERT_PITCH = false;
bool INVERT_ROLL = true;

// ─────────────── FALLBACK RC CENTERING ───────────────
int NUDGE_SPEED = 15;
int VERIFY_ZONE = 15;
int VERIFY_COUNT = 5;

// ─────────────── TIMING ───────────────
int STABILIZE_AFTER_TAKEOFF = 2000;
int PAUSE_AFTER_MOVE = 1000;
int COMMAND_RETRY_DELAY = 500;

// ─────────────── LANDING ───────────────
bool USE_EMERGENCY_LAND = false;

// ══════════════════════════════════════════════════════════════
//                    NETWORK
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
unsigned long bootTime = 0;

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
        if (len > 0) { buf[len] = 0; latest = String(buf); }
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

void flushState() {
    Serial.println("   Flushing stale state data...");
    padId = -1; padX = 0; padY = 0; padZ = 0;
    for (int i = 0; i < 20; i++) {
        while (udpState.parsePacket()) {
            char buf[512];
            udpState.read(buf, 511);
        }
        delay(50);
    }
    Serial.println("   State flushed.");
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
//                    TIMING HELPERS
// ══════════════════════════════════════════════════════════════

unsigned long elapsedSeconds() {
    return (millis() - bootTime) / 1000;
}

bool waitUntil(unsigned long targetMs, const char* label) {
    unsigned long now = millis() - bootTime;
    if (now >= targetMs) {
        Serial.printf("   %s: already past T=%lus (now T=%lus)\n",
                      label, targetMs / 1000, now / 1000);
        return true;
    }

    unsigned long remaining = (targetMs - now) / 1000;
    Serial.printf("\n>>> WAITING for %s at T=%lus (%lus from now) <<<\n",
                  label, targetMs / 1000, remaining);

    unsigned long lastPrint = 0;
    while ((millis() - bootTime) < targetMs) {
        if (checkEmergency()) return false;

        unsigned long secLeft = (targetMs - (millis() - bootTime)) / 1000;
        if (secLeft != lastPrint && secLeft > 0 && secLeft % 10 == 0) {
            Serial.printf("   T=%lus | %s in %lus\n",
                          elapsedSeconds(), label, secLeft);
            lastPrint = secLeft;
        }
        delay(200);
    }

    Serial.printf("   T=%lus | >>> %s NOW <<<\n", elapsedSeconds(), label);
    return true;
}

// ══════════════════════════════════════════════════════════════
//                    LANDING
// ══════════════════════════════════════════════════════════════

void performLand() {
    stopMovement();
    delay(200);
    if (USE_EMERGENCY_LAND) {
        Serial.println("*** EMERGENCY LAND (drop) ***");
        sendCommand("emergency");
    } else {
        Serial.println("*** NORMAL LAND ***");
        sendCommand("land");
    }
    waitResponse(10000);
    flightActive = false;
}

// ══════════════════════════════════════════════════════════════
//  GO TO PAD CENTER
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
            Serial.printf("   After go: X:%d Y:%d Z:%d (pad %d)\n", padX, padY, padZ, padId);
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
            if (padId <= 0) { Serial.println("   Pad lost!"); return false; }
        }
    }
    Serial.println("   go failed after 2 attempts!");
    return false;
}

// ══════════════════════════════════════════════════════════════
//  WAIT FOR PAD
// ══════════════════════════════════════════════════════════════

bool waitForPad(int timeoutMs) {
    Serial.printf("   Waiting for pad (%ds timeout)...\n", timeoutMs / 1000);
    unsigned long start = millis();
    while (millis() - start < (unsigned long)timeoutMs) {
        if (checkEmergency()) return false;
        readState();
        if (padId > 0) {
            Serial.printf("   PAD FOUND! ID=%d | X:%d Y:%d Z:%d\n", padId, padX, padY, padZ);
            return true;
        }
        delay(100);
    }
    Serial.println("   Pad detection timeout!");
    return false;
}

// ══════════════════════════════════════════════════════════════
//  RC FINE-TUNING
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
//  FALLBACK RC CENTERING
// ══════════════════════════════════════════════════════════════

bool centerOnPadRC() {
    Serial.println("\n--- FALLBACK: RC centering ---");
    int centeredCount = 0;
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 20000;

    while (millis() - startTime < TIMEOUT && flightActive) {
        if (checkEmergency()) return false;
        readState();

        if (padId <= 0) { stopMovement(); centeredCount = 0; delay(500); continue; }

        int errorX = padX - PAD_X_OFFSET;
        int errorY = padY - PAD_Y_OFFSET;
        bool centered = abs(errorX) <= VERIFY_ZONE && abs(errorY) <= VERIFY_ZONE;

        if (centered) centeredCount++; else centeredCount = 0;

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
//                    SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
    bootTime = millis();
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n==========================================================");
    Serial.println("   TELLO AUTOMATIC 15 -- CENTER FIRST, THEN DESCEND");
    Serial.println("==========================================================\n");

    Serial.println("TIMELINE:");
    Serial.printf("  T=%lus: Takeoff 1 → fwd 40 → CW 180 → fwd 40 → CW 180 → land\n", TAKEOFF_1_TIME / 1000);
    Serial.printf("  T=%lus: Takeoff 2 → right %d → hover in place\n", TAKEOFF_2_TIME / 1000, REPOSITION_RIGHT);
    Serial.printf("  T=%lus: Center at height → go descent → RC fine → land\n\n", LAND_PHASE_TIME / 1000);

    // ════════════════════════════════════════════════════════════
    // PHASE 0: CONNECT TO DRONE
    // ════════════════════════════════════════════════════════════

    Serial.printf("T=%lus | PHASE 0: Connecting to drone...\n", elapsedSeconds());

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
        while (1) delay(1000);
    }
    Serial.printf(" Connected! (T=%lus)\n\n", elapsedSeconds());

    // Setup UDP
    udpCmd.begin(CMD_PORT);
    udpState.begin(STATE_PORT);
    delay(1000);

    // Enter SDK mode + pad detection
    Serial.printf("T=%lus | Entering SDK mode...\n", elapsedSeconds());
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // Battery check
    sendCommand("battery?");
    String bat = waitResponse(3000);
    bat.trim();
    Serial.printf("Battery: %s%%\n", bat.c_str());

    Serial.printf("\nT=%lus | DRONE READY. Waiting for takeoff time...\n", elapsedSeconds());

    // ════════════════════════════════════════════════════════════
    // PHASE 1: TAKEOFF + OUT + BACK + FACE ORIGINAL + LAND
    // ════════════════════════════════════════════════════════════

    if (!waitUntil(TAKEOFF_1_TIME, "TAKEOFF 1")) goto done;

    Serial.println("\n==========================================================");
    Serial.printf("T=%lus | FLIGHT 1: OUT + BACK + FACE ORIGINAL + LAND\n", elapsedSeconds());
    Serial.println("==========================================================\n");

    flightActive = true;

    {
        char cmd[30];

        // Re-enter SDK mode
        sendWithRetry("command", 500);
        sendWithRetry("mon", 500);
        sendWithRetry("mdirection 0", 500);

        // Takeoff
        Serial.printf("T=%lus | [1] TAKEOFF...\n", elapsedSeconds());
        if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
            Serial.println("    TAKEOFF FAILED!");
            flightActive = false;
            goto done;
        }
        if (checkEmergency() || !flightActive) goto done;

        // Forward 40
        Serial.printf("T=%lus | [2] FORWARD %dcm...\n", elapsedSeconds(), MOVE_AWAY_DISTANCE);
        sprintf(cmd, "forward %d", MOVE_AWAY_DISTANCE);
        sendWithRetry(cmd, PAUSE_AFTER_MOVE);
        if (checkEmergency() || !flightActive) goto done;

        // CW 180
        Serial.printf("T=%lus | [3] CW %d...\n", elapsedSeconds(), TURN_DEGREES);
        sprintf(cmd, "cw %d", TURN_DEGREES);
        sendWithRetry(cmd, PAUSE_AFTER_MOVE);
        if (checkEmergency() || !flightActive) goto done;

        // Forward 40 (return)
        Serial.printf("T=%lus | [4] FORWARD %dcm (return)...\n", elapsedSeconds(), RETURN_DISTANCE);
        sprintf(cmd, "forward %d", RETURN_DISTANCE);
        sendWithRetry(cmd, PAUSE_AFTER_MOVE);
        if (checkEmergency() || !flightActive) goto done;

        // CW 180 (face original direction)
        Serial.printf("T=%lus | [5] CW %d (face original)...\n", elapsedSeconds(), TURN_DEGREES);
        sprintf(cmd, "cw %d", TURN_DEGREES);
        sendWithRetry(cmd, PAUSE_AFTER_MOVE);
        if (checkEmergency() || !flightActive) goto done;

        // Land
        Serial.printf("T=%lus | [6] LANDING...\n", elapsedSeconds());
        performLand();

        Serial.printf("\nT=%lus | FLIGHT 1 COMPLETE. On ground.\n", elapsedSeconds());
    }

    // ════════════════════════════════════════════════════════════
    // PHASE 2: WAIT ON GROUND, THEN TAKEOFF + REPOSITION + HOVER
    // ════════════════════════════════════════════════════════════

    if (!waitUntil(TAKEOFF_2_TIME, "TAKEOFF 2")) goto done;

    Serial.println("\n==========================================================");
    Serial.printf("T=%lus | FLIGHT 2: REPOSITION + HOVER + PRECISION LAND\n", elapsedSeconds());
    Serial.println("==========================================================\n");

    flightActive = true;

    {
        char cmd[30];

        // Re-enter SDK + pad detection
        Serial.printf("T=%lus | [1] Re-entering SDK + pad detection...\n", elapsedSeconds());
        sendWithRetry("command", 500);
        sendWithRetry("mon", 500);
        sendWithRetry("mdirection 0", 500);

        // Flush stale state
        flushState();

        // Takeoff
        Serial.printf("T=%lus | [2] TAKEOFF...\n", elapsedSeconds());
        if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
            Serial.println("    TAKEOFF FAILED!");
            flightActive = false;
            goto done;
        }
        if (checkEmergency() || !flightActive) goto done;

        // Re-enable SDK + pad detection after takeoff
        sendWithRetry("command", 500);
        sendWithRetry("mon", 500);
        sendWithRetry("mdirection 0", 500);

        // Right 20cm (already facing original direction from flight 1)
        Serial.printf("T=%lus | [3] RIGHT %dcm...\n", elapsedSeconds(), REPOSITION_RIGHT);
        sprintf(cmd, "right %d", REPOSITION_RIGHT);
        sendWithRetry(cmd, PAUSE_AFTER_MOVE);
        if (checkEmergency() || !flightActive) goto done;

        Serial.printf("\nT=%lus | HOVERING — waiting for robot to return...\n", elapsedSeconds());
    }

    // ════════════════════════════════════════════════════════════
    // PHASE 3: CENTER FIRST AT CURRENT HEIGHT, THEN DESCEND
    // ════════════════════════════════════════════════════════════

    if (!waitUntil(LAND_PHASE_TIME, "PRECISION LAND")) goto done;

    Serial.println("\n==========================================================");
    Serial.printf("T=%lus | PHASE 3: CENTER FIRST, THEN DESCEND\n", elapsedSeconds());
    Serial.println("==========================================================\n");

    {
        // Look for pad
        Serial.printf("T=%lus | [4] Looking for pad...\n", elapsedSeconds());
        bool padFound = waitForPad(10000);

        if (!padFound) {
            Serial.println("   No pad found! Trying RC centering...");
            if (!centerOnPadRC()) {
                Serial.println("   Failed -- landing wherever.");
                performLand();
                goto done;
            }
        }
        if (!flightActive) goto done;

        // ── STEP A: CENTER AT CURRENT HEIGHT (no descent!) ──
        {
            // Read fresh state to get current Z
            for (int i = 0; i < 5; i++) { readState(); delay(100); }
            int currentZ = padZ;

            // Clamp to safe range (go command needs z > 20)
            if (currentZ < 30) currentZ = 30;

            Serial.printf("\nT=%lus | [5] CENTER at current height (%dcm)...\n",
                          elapsedSeconds(), currentZ);
            Serial.println("   (horizontal move only — no descent yet)");

            bool centerOk = goToCenter(currentZ, GO_SPEED);
            if (checkEmergency() || !flightActive) goto done;

            if (!centerOk) {
                Serial.println("   Centering failed! Trying RC centering...");
                if (!centerOnPadRC()) {
                    Serial.println("   Failed -- landing wherever.");
                    performLand();
                    goto done;
                }
            }

            // Verify we're actually centered now
            for (int i = 0; i < 3; i++) { readState(); delay(50); }
            Serial.printf("   After centering: X:%d Y:%d Z:%d\n", padX, padY, padZ);
        }

        if (checkEmergency() || !flightActive) goto done;

        // ── STEP B: GRADUAL DESCENT (already centered) ──
        Serial.printf("\nT=%lus | [6] GO descent: ", elapsedSeconds());
        for (int i = 0; i < NUM_GO_STEPS; i++) Serial.printf("%d ", GO_HEIGHTS[i]);
        Serial.println();

        for (int i = 0; i < NUM_GO_STEPS; i++) {
            Serial.printf("\n   GO to %dcm...\n", GO_HEIGHTS[i]);

            bool goOk = goToCenter(GO_HEIGHTS[i], GO_SPEED);
            if (checkEmergency() || !flightActive) goto done;

            if (!goOk && i == 0) {
                Serial.println("   First descent step failed! Trying RC centering...");
                if (!centerOnPadRC()) {
                    Serial.println("   Failed -- landing wherever.");
                    performLand();
                    goto done;
                }
            } else if (!goOk) {
                Serial.printf("   go step %d failed -- proceeding.\n", i + 1);
            }

            if (checkEmergency() || !flightActive) goto done;
        }

        // ── STEP C: RC FINE-TUNE ──
        Serial.printf("\nT=%lus | [7] RC fine-tuning...\n", elapsedSeconds());
        fineCenterRC();
        if (checkEmergency() || !flightActive) goto done;

        // Final position
        delay(500);
        for (int i = 0; i < 5; i++) { readState(); delay(100); }
        Serial.printf("   Final position: X:%d Y:%d Z:%d (pad %d)\n",
                      padX, padY, padZ, padId);

        // ── STEP D: LAND ──
        Serial.printf("\nT=%lus | [8] PRECISION LANDING...\n", elapsedSeconds());
        performLand();
    }

done:
    // Final battery check
    sendCommand("battery?");
    String batEnd = waitResponse(3000);
    batEnd.trim();
    Serial.printf("\nT=%lus | Battery: %s%%\n", elapsedSeconds(), batEnd.c_str());
    Serial.println("\n==========================================================");
    Serial.println("          ALL DONE!");
    Serial.println("==========================================================\n");
}

// ══════════════════════════════════════════════════════════════
//                    LOOP (emergency only)
// ══════════════════════════════════════════════════════════════

void loop() {
    if (Serial.available()) {
        char c = toupper(Serial.read());
        if (c == 'E') {
            Serial.println("\n!!! EMERGENCY !!!");
            sendCommand("emergency");
            flightActive = false;
        } else if (c == 'L') {
            Serial.println("\nLanding...");
            sendCommand("land");
            flightActive = false;
        }
    }
    delay(10);
}
