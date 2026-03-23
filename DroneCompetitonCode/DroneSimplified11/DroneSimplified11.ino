// ══════════════════════════════════════════════════════════════
//  TELLO v11 — DETECTION LIMIT FINDER
//
//  Strategy:
//    1. go 0 0 50 15 m<id> — center at 50cm using Tello PID
//    2. Verify centered (10 consecutive readings)
//    3. RC descent from 50cm — slow throttle + small corrections
//       → Track pad detection heights
//       → When detection lost: stop and print last height
//    4. Land
//
//  Purpose: Find the lowest altitude where the Tello can still
//  detect the mission pad on the black surface. v12 will use
//  this height for final centering before landing.
//
//  NOTE: RC throttle may stall at ~47cm (Tello's internal
//  height controller). Stuck detection (2s same height) will
//  catch this and stop early.
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

// ─────────────── GO COMMAND ───────────────
int GO_CENTER_HEIGHT = 50;      // cm — go to center at this height
int GO_SPEED = 15;              // cm/s

// ─────────────── CENTERING VERIFICATION ───────────────
int VERIFY_COUNT = 10;          // consecutive centered readings needed
int VERIFY_ZONE = 15;           // cm — tolerance for "centered"

// ─────────────── RC DESCENT (below 50cm) ───────────────
int RC_DESCENT_THROTTLE = 15;   // throttle magnitude (sent as negative)
int RC_CORRECT_SPEED = 8;       // lateral correction speed (small)
int DEAD_ZONE = 5;              // cm — don't correct within this
int PAD_X_OFFSET = -4;          // camera X offset
int PAD_Y_OFFSET = 4;           // camera Y offset
bool INVERT_PITCH = false;
bool INVERT_ROLL = true;

// ─────────────── FALLBACK RC CENTERING ───────────────
int NUDGE_SPEED = 15;           // speed for fallback centering

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
//
//  After `go` command, checks that the drone is actually
//  centered over the pad for VERIFY_COUNT consecutive readings.
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

        // Print progress every 5 readings or when done
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
//  RC DESCENT WITH DETECTION TRACKING
//
//  Slowly descends from ~50cm using RC throttle.
//  Applies small lateral corrections (with offset).
//  Tracks when pad detection is lost and records last height.
//
//  Has stuck detection: if height doesn't change for 2s,
//  stops early (RC throttle can't push below ~47cm).
//
//  Returns: last height (cm) where pad was detected.
// ══════════════════════════════════════════════════════════════

int rcDescentWithTracking() {
    Serial.println("\n--- RC DESCENT -- tracking detection limit ---");
    Serial.printf("   Throttle: -%d | Correct speed: %d | Dead zone: %dcm\n",
                  RC_DESCENT_THROTTLE, RC_CORRECT_SPEED, DEAD_ZONE);
    Serial.printf("   Offsets: X=%d Y=%d | Invert P:%d R:%d\n",
                  PAD_X_OFFSET, PAD_Y_OFFSET, INVERT_PITCH, INVERT_ROLL);

    int lastHeight = 0;
    int lastGoodX = 0, lastGoodY = 0;
    int lostCount = 0;

    // Stuck detection
    int prevHeight = 999;
    int stuckCount = 0;

    unsigned long startTime = millis();
    const unsigned long MAX_TIME = 20000;  // 20s max

    while (millis() - startTime < MAX_TIME && flightActive) {
        if (checkEmergency()) return lastHeight;

        readState();

        if (padId > 0) {
            // -- Pad detected --
            lastHeight = abs(padZ);
            lastGoodX = padX;
            lastGoodY = padY;
            lostCount = 0;

            // Stuck detection: same height for 2 seconds
            if (abs(lastHeight - prevHeight) < 2) {
                stuckCount++;
            } else {
                stuckCount = 0;
                prevHeight = lastHeight;
            }

            if (stuckCount >= 20) {  // 20 x 100ms = 2s
                stopMovement();
                Serial.println();
                Serial.println("   ========================================");
                Serial.printf( "   STUCK at Z = %dcm for 2 seconds!\n", lastHeight);
                Serial.println("   RC throttle cannot descend further.");
                Serial.println("   Pad STILL DETECTED at this height.");
                Serial.println("   ========================================");
                Serial.println();
                return lastHeight;
            }

            // Calculate corrections with offset
            int errorX = padX - PAD_X_OFFSET;
            int errorY = padY - PAD_Y_OFFSET;

            int roll = 0, pitch = 0;

            // Only correct if outside dead zone
            if (abs(errorX) > DEAD_ZONE) {
                pitch = (errorX > 0) ? -RC_CORRECT_SPEED : RC_CORRECT_SPEED;
            }
            if (abs(errorY) > DEAD_ZONE) {
                roll = (errorY > 0) ? -RC_CORRECT_SPEED : RC_CORRECT_SPEED;
            }

            if (INVERT_PITCH) pitch = -pitch;
            if (INVERT_ROLL) roll = -roll;

            // Descend + correct
            sendRC(roll, pitch, -RC_DESCENT_THROTTLE, 0);

            // Print every 300ms
            if (millis() - lastPrintTime > 300) {
                lastPrintTime = millis();
                Serial.printf("   Z:%dcm | X:%d Y:%d | err X:%d Y:%d | rc(%d,%d,-%d)\n",
                              lastHeight, padX, padY, errorX, errorY,
                              roll, pitch, RC_DESCENT_THROTTLE);
            }

        } else {
            // -- Pad NOT detected --
            lostCount++;

            if (lostCount >= 3) {
                // 3 consecutive misses = pad is truly lost
                stopMovement();
                Serial.println();
                Serial.println("   ========================================");
                Serial.printf( "   PAD DETECTION LOST!\n");
                Serial.printf( "   Last detected: Z = %dcm\n", lastHeight);
                Serial.printf( "   Last position: X:%d  Y:%d\n", lastGoodX, lastGoodY);
                Serial.println("   ========================================");
                Serial.println();
                return lastHeight;
            }

            // Might be a glitch -- keep descending
            sendRC(0, 0, -RC_DESCENT_THROTTLE, 0);
        }

        delay(100);
    }

    // Timeout
    stopMovement();
    Serial.println();
    Serial.println("   ========================================");
    Serial.printf( "   DESCENT TIMEOUT (20s)\n");
    if (lastHeight > 0) {
        Serial.printf( "   Pad still detected at Z = %dcm\n", lastHeight);
    } else {
        Serial.println("   Pad never detected during descent!");
    }
    Serial.println("   ========================================");
    Serial.println();

    return lastHeight;
}

// ══════════════════════════════════════════════════════════════
//  FALLBACK: RC CENTERING (if go command fails)
//  Simple v4-style fixed-speed RC corrections
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
    Serial.println("     v11 -- DETECTION LIMIT FINDER");
    Serial.println("==========================================================\n");

    Serial.println("FLIGHT PLAN:");
    Serial.printf("  [1] Enable pad detection\n");
    Serial.printf("  [2] Takeoff\n");
    Serial.printf("  [3] Forward %dcm\n", MOVE_AWAY_DISTANCE);
    Serial.printf("  [4] CW %d deg\n", TURN_DEGREES);
    Serial.printf("  [5] Forward %dcm (return)\n", RETURN_DISTANCE);
    Serial.printf("  [6] go 0 0 %d %d m<id> (center)\n", GO_CENTER_HEIGHT, GO_SPEED);
    Serial.printf("  [7] Verify centered (%d readings)\n", VERIFY_COUNT);
    Serial.printf("  [8] RC descent -- track when pad detection is lost\n");
    Serial.printf("  [9] Land\n\n");

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

    Serial.println("\n======== PHASE 3: CENTER + TRACK DETECTION ========\n");

    // Re-enable SDK + pad detection
    Serial.println("[5.5] Re-enabling SDK + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // ── Step 6: Wait for pad then go to center ──
    Serial.printf("\n[6] GO TO CENTER at %dcm...\n", GO_CENTER_HEIGHT);

    bool padFound = waitForPad(10000);

    if (!padFound) {
        // No pad detected -- try RC centering first
        Serial.println("\n   No pad found! Trying RC centering first...");
        if (!centerOnPadRC()) {
            Serial.println("   RC centering failed -- landing wherever.");
            performLand();
            goto sequenceDone;
        }
        // RC centered, pad should be visible now
    }

    {
        // Use go command to center at 50cm
        Serial.printf("\n   Using go command (center at %dcm)...\n", GO_CENTER_HEIGHT);
        bool goSuccess = goToCenter(GO_CENTER_HEIGHT, GO_SPEED);

        if (checkEmergency()) return;

        if (!goSuccess) {
            Serial.println("\n   go command failed!");
            // Try RC centering as fallback
            Serial.println("   Trying RC centering fallback...");
            if (!centerOnPadRC()) {
                Serial.println("   RC fallback also failed -- landing wherever.");
                performLand();
                goto sequenceDone;
            }
            // RC centered -- proceed to verify + descent
        }
    }

    // ── Step 7: Verify centered ──
    Serial.printf("\n[7] VERIFY CENTERED...\n");
    verifyCentered();  // Even if verify fails, we proceed

    if (checkEmergency()) return;

    {
        // ── Step 8: RC descent with detection tracking ──
        Serial.printf("\n[8] RC DESCENT -- tracking detection limit...\n");
        int lastDetectedHeight = rcDescentWithTracking();

        if (checkEmergency()) return;

        // ── Step 9: Land ──
        Serial.println("\n[9] LANDING...");
        Serial.println("   ========================================");
        Serial.printf( "   RESULT: Last pad detection at Z = %dcm\n", lastDetectedHeight);
        Serial.println("   (Use this height in v12 for final centering)");
        Serial.println("   ========================================");
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
    Serial.println("   TELLO v11 -- DETECTION LIMIT FINDER");
    Serial.println("==========================================================\n");

    Serial.println("PURPOSE: Find lowest altitude where pad is still detected.");
    Serial.println("Uses go command to center at 50cm, then RC descent to");
    Serial.println("track when detection is lost. v12 will use this height");
    Serial.println("for final centering before landing.\n");

    // Print settings
    Serial.println("SETTINGS:");
    Serial.printf("  Go center:     go 0 0 %d %d m<id>\n", GO_CENTER_HEIGHT, GO_SPEED);
    Serial.printf("  Verify:        %d readings, zone +/-%dcm\n", VERIFY_COUNT, VERIFY_ZONE);
    Serial.printf("  RC descent:    throttle -%d, correct %d, dead zone %dcm\n",
                  RC_DESCENT_THROTTLE, RC_CORRECT_SPEED, DEAD_ZONE);
    Serial.printf("  Offsets:       X=%d Y=%d\n", PAD_X_OFFSET, PAD_Y_OFFSET);
    Serial.printf("  Invert:        pitch=%d roll=%d\n", INVERT_PITCH, INVERT_ROLL);
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
