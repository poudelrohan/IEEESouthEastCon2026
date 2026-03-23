// ══════════════════════════════════════════════════════════════
//  TELLO v9 — PRECISION PAD LANDING ON 10" ROBOT CIRCLE
//
//  v7's full precision landing system + simple flight path.
//  No hover/search — goes straight into centering after return.
//
//  Flight path:
//    1. Takeoff (~100cm)
//    2. Enable mission pad detection (mon + mdirection 0)
//    3. Forward 40cm — move away 15"+
//    4. CW 180° — turn around
//    5. Forward 40cm — walk back to start
//    6. Center → Stabilize → Staged descent → Land
//
//  From v7 (precision landing):
//    - Proportional RC control (calcSpeed — far=faster, close=slower)
//    - 5cm target zone (pad is ~20cm, circle is 25.4cm)
//    - 1.5s stabilization hold after centering
//    - Staged SDK "down" descent (RC can't push below ~40cm)
//    - Re-centering at each descent checkpoint
//    - Quick re-center with wider 8cm zone
//
//  From v8 (fixes):
//    - "command" re-entry after takeoff (fixes "Not joystick" error)
//    - mon + mdirection 0 re-sent before landing phase
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
int MOVE_AWAY_DISTANCE = 40;    // cm forward (15" = 38cm, using 40 for margin)
int TURN_DEGREES = 180;         // CW turn to face back toward pad
int RETURN_DISTANCE = 40;       // cm forward after turn (back to start)

// ─────────────── TIMING (ms) ───────────────
int STABILIZE_AFTER_TAKEOFF = 3000;   // Wait after takeoff
int PAUSE_AFTER_MOVE = 1500;          // Pause after each move
int COMMAND_RETRY_DELAY = 500;        // Delay before retry
int STABILIZE_AFTER_CENTER = 1500;    // Hold still after centering confirmed

// ─────────────── PAD LANDING (v7 values) ───────────────
int PAD_X_OFFSET = -4;        // cm forward from rocket icon (-ve = back)
int PAD_Y_OFFSET = 4;         // cm left/right adjustment (-ve = right)
int TARGET_ZONE = 5;          // Must be within 5cm to count as centered
int QUICK_TARGET_ZONE = 8;    // Wider zone for quick re-centers during descent
int CENTERED_COUNT_NEEDED = 15; // Consecutive readings needed to confirm

// ─────────────── DESCENT CHECKPOINTS (cm) ───────────────
int DESCENT_CHECKPOINT_1 = 60;  // First pause height
int DESCENT_CHECKPOINT_2 = 40;  // Second pause height
int RECHECK_COUNT = 3;          // Quick re-centering confirmations at each checkpoint

// ─────────────── PROPORTIONAL CONTROL (v7 values) ───────────────
int MIN_NUDGE_SPEED = 8;     // Minimum RC speed
int MAX_NUDGE_SPEED = 15;     // Maximum RC speed
float PROPORTIONAL_GAIN = 0.6; // Speed = error * gain

// ─────────────── LANDING STYLE ───────────────
bool USE_EMERGENCY_LAND = false;  // true = drop, false = gentle

// ─────────────── ORIENTATION FIX ───────────────
// v7 after cw 270: INVERT_PITCH=false, INVERT_ROLL=true (PROVEN)
// v9 after cw 180: heading is different — may need testing/flipping
bool INVERT_PITCH = false;
bool INVERT_ROLL = true;

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
//                    RC CONTROL — PROPORTIONAL (v7)
// ══════════════════════════════════════════════════════════════

int calcSpeed(int error) {
    int absError = abs(error);
    int speed = (int)(absError * PROPORTIONAL_GAIN);
    return constrain(speed, MIN_NUDGE_SPEED, MAX_NUDGE_SPEED);
}

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
//  CENTER ON PAD — PROPORTIONAL CONTROL (v7)
//
//  Speed scales with distance (far=faster, close=slower)
//  Tight 5cm target zone for true pad center
// ══════════════════════════════════════════════════════════════

bool centerOnPad() {
    Serial.println("\n----------- CENTERING ON PAD (proportional v7) -----------");
    Serial.printf("Target: X=%d, Y=%d | Zone: +/-%dcm | Need %d confirmations\n",
                  PAD_X_OFFSET, PAD_Y_OFFSET, TARGET_ZONE, CENTERED_COUNT_NEEDED);
    Serial.printf("Speed range: %d-%d (gain=%.1f) | Invert P=%s R=%s\n",
                  MIN_NUDGE_SPEED, MAX_NUDGE_SPEED, PROPORTIONAL_GAIN,
                  INVERT_PITCH ? "true" : "false", INVERT_ROLL ? "true" : "false");

    centeredCount = 0;
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 25000;

    while (millis() - startTime < TIMEOUT && flightActive) {

        if (checkEmergency()) return false;

        readState();

        if (padId <= 0) {
            Serial.println("Lost pad! Hovering...");
            stopMovement();
            centeredCount = 0;
            delay(500);
            continue;
        }

        int errorX = padX - PAD_X_OFFSET;
        int errorY = padY - PAD_Y_OFFSET;

        bool xOk = abs(errorX) <= TARGET_ZONE;
        bool yOk = abs(errorY) <= TARGET_ZONE;
        bool centered = xOk && yOk;

        if (centered) {
            centeredCount++;
        } else {
            centeredCount = 0;
        }

        // Print status every 500ms
        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            Serial.printf("PAD:%d | X:%4d Y:%4d Z:%3d | Err X:%4d Y:%4d | ",
                          padId, padX, padY, padZ, errorX, errorY);
            if (centered) {
                Serial.printf("CENTERED [%d/%d]\n", centeredCount, CENTERED_COUNT_NEEDED);
            } else {
                Serial.printf("Adjusting (spd X:%d Y:%d)\n",
                              abs(errorX) > TARGET_ZONE ? calcSpeed(errorX) : 0,
                              abs(errorY) > TARGET_ZONE ? calcSpeed(errorY) : 0);
            }
        }

        if (centeredCount >= CENTERED_COUNT_NEEDED) {
            Serial.println("CENTERING COMPLETE!");
            stopMovement();
            return true;
        }

        // Proportional control: speed scales with distance
        int roll = 0, pitch = 0;

        if (errorX > TARGET_ZONE) {
            pitch = -calcSpeed(errorX);
        } else if (errorX < -TARGET_ZONE) {
            pitch = calcSpeed(errorX);
        }

        if (errorY > TARGET_ZONE) {
            roll = -calcSpeed(errorY);
        } else if (errorY < -TARGET_ZONE) {
            roll = calcSpeed(errorY);
        }

        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL) roll = -roll;

        sendRC(roll, pitch, 0, 0);
        delay(100);
    }

    Serial.println("CENTERING TIMEOUT!");
    stopMovement();
    return false;
}

// ══════════════════════════════════════════════════════════════
//  QUICK RE-CENTER CHECK (v7)
//
//  Used during staged descent. Wider 8cm zone, shorter confirms.
// ══════════════════════════════════════════════════════════════

bool quickRecenter(int confirmationsNeeded) {
    Serial.printf("   Quick re-center (zone=%dcm, need %d confirms, 15s timeout)...\n",
                  QUICK_TARGET_ZONE, confirmationsNeeded);

    int count = 0;
    unsigned long start = millis();
    unsigned long lastPrint = 0;
    const unsigned long TIMEOUT = 15000;
    bool gotFreshData = false;

    while (millis() - start < TIMEOUT && flightActive) {

        if (checkEmergency()) return false;

        bool fresh = readState();
        if (!fresh) {
            delay(50);
            continue;
        }
        gotFreshData = true;

        if (padId <= 0) {
            stopMovement();
            count = 0;
            delay(200);
            continue;
        }

        int errorX = padX - PAD_X_OFFSET;
        int errorY = padY - PAD_Y_OFFSET;

        bool centered = abs(errorX) <= QUICK_TARGET_ZONE && abs(errorY) <= QUICK_TARGET_ZONE;

        if (millis() - lastPrint > 1000) {
            lastPrint = millis();
            if (centered) {
                Serial.printf("   RC: X:%d Y:%d Z:%d | err X:%d Y:%d | OK [%d/%d]\n",
                              padX, padY, padZ, errorX, errorY, count, confirmationsNeeded);
            } else {
                Serial.printf("   RC: X:%d Y:%d Z:%d | err X:%d Y:%d | adjusting\n",
                              padX, padY, padZ, errorX, errorY);
            }
        }

        if (centered) {
            count++;
            stopMovement();
            if (count >= confirmationsNeeded) {
                Serial.printf("   Re-center OK! (X:%d Y:%d Z:%d err X:%d Y:%d)\n",
                              padX, padY, padZ, errorX, errorY);
                return true;
            }
        } else {
            count = 0;

            int roll = 0, pitch = 0;

            if (errorX > QUICK_TARGET_ZONE) pitch = -calcSpeed(errorX);
            else if (errorX < -QUICK_TARGET_ZONE) pitch = calcSpeed(errorX);

            if (errorY > QUICK_TARGET_ZONE) roll = -calcSpeed(errorY);
            else if (errorY < -QUICK_TARGET_ZONE) roll = calcSpeed(errorY);

            if (INVERT_PITCH) pitch = -pitch;
            if (INVERT_ROLL) roll = -roll;

            sendRC(roll, pitch, 0, 0);
        }

        delay(100);
    }

    Serial.printf("   Re-center timeout (got data: %s)\n", gotFreshData ? "yes" : "NO!");
    stopMovement();
    return false;
}

// ══════════════════════════════════════════════════════════════
//  STAGED DESCENT AND LAND ON PAD (v7)
//
//  Uses SDK "down X" commands (not RC throttle — Tello fights
//  RC below ~40cm). Re-centers at each checkpoint.
//
//  Sequence:
//    1. SDK "down" to ~60cm, re-center
//    2. SDK "down" to ~40cm, re-center
//    3. Final re-center, then "land"
// ══════════════════════════════════════════════════════════════

bool descendAndLandOnPad() {
    Serial.println("\n----------- STAGED DESCENT (SDK down commands) -----------");

    // Read current height
    delay(2000);
    for (int i = 0; i < 10; i++) { readState(); delay(100); }
    int currentHeight = abs(padZ);
    if (currentHeight == 0) currentHeight = 88;
    Serial.printf("Current height: ~%dcm\n", currentHeight);

    // ── Stage 1: Descend to checkpoint 1 (~60cm) ──
    int drop1 = currentHeight - DESCENT_CHECKPOINT_1;
    if (drop1 >= 20) {
        Serial.printf("\n[Stage 1] Descending %dcm (SDK down command)...\n", drop1);
        char cmd[30];
        sprintf(cmd, "down %d", drop1);
        sendWithRetry(cmd, 0);

        if (checkEmergency()) return false;

        delay(2000);
        for (int i = 0; i < 10; i++) { readState(); delay(100); }
        Serial.printf("   After descent: Z=%dcm | X:%d Y:%d\n", padZ, padX, padY);

        Serial.println("   Re-centering at checkpoint 1...");
        quickRecenter(RECHECK_COUNT);
    } else {
        Serial.printf("[Stage 1] Already near checkpoint 1 (Z:%d), skipping\n", currentHeight);
    }

    if (checkEmergency()) return false;

    // ── Stage 2: Descend to checkpoint 2 (~40cm) ──
    delay(1000);
    for (int i = 0; i < 10; i++) { readState(); delay(100); }
    currentHeight = abs(padZ);
    if (currentHeight == 0) currentHeight = 58;

    int drop2 = currentHeight - DESCENT_CHECKPOINT_2;
    if (drop2 >= 20) {
        Serial.printf("\n[Stage 2] Descending %dcm (SDK down command)...\n", drop2);
        char cmd[30];
        sprintf(cmd, "down %d", drop2);
        sendWithRetry(cmd, 0);

        if (checkEmergency()) return false;

        delay(2000);
        for (int i = 0; i < 10; i++) { readState(); delay(100); }
        Serial.printf("   After descent: Z=%dcm | X:%d Y:%d\n", padZ, padX, padY);

        Serial.println("   Re-centering at checkpoint 2...");
        quickRecenter(RECHECK_COUNT);
    } else if (drop2 > 0) {
        Serial.printf("[Stage 2] Only %dcm to drop (< 20 minimum). Using short RC burst...\n", drop2);
        for (int i = 0; i < 15; i++) {
            if (checkEmergency()) return false;
            sendRC(0, 0, -20, 0);
            delay(100);
        }
        stopMovement();
        delay(1000);
        quickRecenter(RECHECK_COUNT);
    } else {
        Serial.printf("[Stage 2] Already near checkpoint 2 (Z:%d), skipping\n", currentHeight);
    }

    if (checkEmergency()) return false;

    // ── Final: Re-center one last time, then land ──
    Serial.println("\n[FINAL CENTER] Last centering before land...");
    quickRecenter(RECHECK_COUNT);

    readState();
    if (padId > 0) {
        Serial.printf("[LAND] Final position: X:%d Y:%d Z:%d (err X:%d Y:%d)\n",
                      padX, padY, padZ,
                      abs(padX - PAD_X_OFFSET), abs(padY - PAD_Y_OFFSET));
    } else {
        Serial.println("[LAND] Pad not visible — landing at current position");
    }

    performLand();
    return true;
}

// ══════════════════════════════════════════════════════════════
//                    MAIN SEQUENCE
// ══════════════════════════════════════════════════════════════

void runSimpleSequence() {
    Serial.println("\n==========================================================");
    Serial.println("     v9 — PRECISION PAD LANDING (v7 system, simple path)");
    Serial.println("==========================================================\n");

    Serial.println("FLIGHT PLAN:");
    Serial.printf("  [1] Takeoff\n");
    Serial.printf("  [2] Enable pad detection (mon + mdirection 0)\n");
    Serial.printf("  [3] Forward %dcm (move away 15\"+)\n", MOVE_AWAY_DISTANCE);
    Serial.printf("  [4] CW %d deg (turn around)\n", TURN_DEGREES);
    Serial.printf("  [5] Forward %dcm (return to pad)\n", RETURN_DISTANCE);
    Serial.printf("  [6] Center (proportional, +/-%dcm zone)\n", TARGET_ZONE);
    Serial.printf("  [7] Stabilize (%dms hold)\n", STABILIZE_AFTER_CENTER);
    Serial.printf("  [8] Staged descent: %dcm -> %dcm -> land\n",
                  DESCENT_CHECKPOINT_1, DESCENT_CHECKPOINT_2);
    Serial.println();

    flightActive = true;
    char cmd[30];

    // ═══════════════════════════════════════════════════════
    // PHASE 1: TAKEOFF + MOVE AWAY
    // ═══════════════════════════════════════════════════════

    Serial.println("======== PHASE 1: TAKEOFF + MOVE AWAY ========\n");

    // [1] Takeoff
    Serial.println("[1] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    Serial.println("    Takeoff complete (~100cm)\n");

    // [2] Enable mission pad detection + ensure SDK mode
    Serial.println("[2] Enabling mission pad detection...");
    sendWithRetry("command", 500);       // Re-enter SDK mode (fixes "Not joystick" error)
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);  // Downward only = 20Hz
    Serial.println("    Pad detection ON (downward, 20Hz)\n");

    // [3] Forward — move away from robot
    Serial.printf("[3] FORWARD %dcm (moving away)...\n", MOVE_AWAY_DISTANCE);
    sprintf(cmd, "forward %d", MOVE_AWAY_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // ═══════════════════════════════════════════════════════
    // PHASE 2: RETURN TO PAD
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 2: RETURN TO PAD ========\n");

    // [4] Turn around 180°
    Serial.printf("[4] CW %d deg (turning around)...\n", TURN_DEGREES);
    sprintf(cmd, "cw %d", TURN_DEGREES);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // [5] Forward — walk back to starting position
    Serial.printf("[5] FORWARD %dcm (returning to pad)...\n", RETURN_DISTANCE);
    sprintf(cmd, "forward %d", RETURN_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // ═══════════════════════════════════════════════════════
    // PHASE 3: PRECISION PAD LANDING (v7 system)
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 3: PRECISION PAD LANDING ========\n");

    // [6a] Re-enter SDK mode + re-enable pad detection
    Serial.println("[6a] Re-enabling SDK mode + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // [6] Center on pad — proportional control (goes straight into centering)
    Serial.println("\n[6] CENTERING (proportional control)...");
    bool centered = centerOnPad();

    if (centered) {
        // [7] Stabilization hold — kill momentum
        Serial.printf("\n[7] STABILIZING (%dms hold)...\n", STABILIZE_AFTER_CENTER);
        stopMovement();
        delay(STABILIZE_AFTER_CENTER);

        // Verify still centered after hold
        readState();
        int errX = abs(padX - PAD_X_OFFSET);
        int errY = abs(padY - PAD_Y_OFFSET);
        Serial.printf("    Post-hold: X:%d Y:%d Z:%d (err X:%d Y:%d)\n",
                      padX, padY, padZ, errX, errY);

        if (errX > TARGET_ZONE * 2 || errY > TARGET_ZONE * 2) {
            Serial.println("    Drifted during hold! Re-centering...");
            centered = centerOnPad();
            if (centered) {
                stopMovement();
                delay(STABILIZE_AFTER_CENTER);
            }
        }

        // [8] Staged descent
        Serial.println("\n[8] STAGED DESCENT...");
        descendAndLandOnPad();
    } else {
        Serial.println("Centering failed - landing wherever...");
        performLand();
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
    Serial.println("   TELLO v9 — PRECISION PAD LANDING (v7 + simple path)");
    Serial.println("==========================================================\n");

    Serial.println("Mission: Take off, move 15\"+ away, return, land on 10\" circle");
    Serial.println("Landing: v7 precision (proportional RC, staged SDK descent)");
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

    Serial.println("PRECISION LANDING (v7):");
    Serial.printf("  Target zone:     +/-%dcm\n", TARGET_ZONE);
    Serial.printf("  Quick zone:      +/-%dcm\n", QUICK_TARGET_ZONE);
    Serial.printf("  Confirmations:   %d\n", CENTERED_COUNT_NEEDED);
    Serial.printf("  RC speed:        %d-%d (proportional, gain=%.1f)\n",
                  MIN_NUDGE_SPEED, MAX_NUDGE_SPEED, PROPORTIONAL_GAIN);
    Serial.printf("  Descent stages:  SDK down to %dcm -> %dcm -> land\n",
                  DESCENT_CHECKPOINT_1, DESCENT_CHECKPOINT_2);
    Serial.printf("  Pad offsets:     X=%d Y=%d\n", PAD_X_OFFSET, PAD_Y_OFFSET);
    Serial.printf("  Stab hold:       %dms\n", STABILIZE_AFTER_CENTER);
    Serial.printf("  Invert pitch:    %s\n", INVERT_PITCH ? "true" : "false");
    Serial.printf("  Invert roll:     %s\n", INVERT_ROLL ? "true" : "false");
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

    // Enable mission pad detection early
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
    Serial.println("  s = START sequence (takeoff -> move -> land on pad)");
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
