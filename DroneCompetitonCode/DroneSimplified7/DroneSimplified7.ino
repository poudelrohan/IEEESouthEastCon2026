// ══════════════════════════════════════════════════════════════
//  TELLO SIMPLE SEQUENCE v7 — PRECISION PAD LANDING
//
//  Based on v4 (the best working version). Key improvements:
//
//  1. ENABLES mission pad detection (mon + mdirection 0)
//     v4 never sent these commands — pad detection was accidental.
//
//  2. PROPORTIONAL RC control (replaces bang-bang)
//     v4: always nudge at speed 15, regardless of distance → oscillation
//     v7: speed scales with distance (far=faster, close=slower)
//         This prevents overshoot near the center of the pad.
//
//  3. TIGHTER TARGET_ZONE (5cm instead of 15cm)
//     The pad is only 20cm wide. v4's 15cm zone meant "centered"
//     could be at the very edge. 5cm zone = true center.
//
//  4. LAND FROM ~40cm (instead of 45cm)
//     The "land" command takes ~2 seconds of uncontrolled descent.
//     From 40cm that's ~1.5 seconds of drift. From 45cm it's ~2-3 seconds.
//
//  5. STABILIZATION HOLD after centering confirmed
//     1.5 second hover at rc 0,0,0,0 to kill residual momentum
//     before starting descent.
//
//  6. STAGED DESCENT with re-centering checkpoints
//     Uses SDK "down X" commands (not RC throttle — Tello fights RC below 40cm!)
//       - SDK "down 30" to ~60cm, pause & re-verify centered
//       - SDK "down 20" to ~40cm, pause & re-verify centered
//       - Then issue "land" from close range
//     This catches drift introduced during descent.
//
//  Commands:
//    S = Full sequence (Phase 1 movement + precision pad landing)
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
int MOVE_RIGHT_1 = 50;        // Step 3: Right toward center
int ROTATE_DEGREES = 270;     // Step 4: Rotate right (cw) — same as 90 left
int MOVE_FORWARD_2 = 40;      // Step 5: Forward (now going "left" in arena)
int MOVE_LEFT_1 = 30;         // Step 6: Small adjustment

// ─────────────── SEARCH SETTINGS ───────────────
int SEARCH_TIMEOUT = 15000;   // Max time to look for pad (ms)

// ─────────────── TIMING (ms) ───────────────
int STABILIZE_AFTER_TAKEOFF = 3000;   // Wait after takeoff
int PAUSE_AFTER_MOVE = 1500;          // Pause after each move
int COMMAND_RETRY_DELAY = 500;        // Delay before retry
int STABILIZE_AFTER_CENTER = 1500;    // NEW: Hold still after centering confirmed

// ─────────────── PAD LANDING ───────────────
int PAD_X_OFFSET = -4;        // cm forward from rocket icon (-ve = back)
int PAD_Y_OFFSET = 4;         // cm left/right adjustment (-ve = right)
int TARGET_ZONE = 8;          // v7: TIGHTER — must be within 5cm (was 15)
int QUICK_TARGET_ZONE = 8;    // v7.2: Wider zone for quick re-centers during descent
int CENTERED_COUNT_NEEDED = 15; // v7: 15 confirmations (was 25, but zone is tighter now)
// ─────────────── DESCENT CHECKPOINTS (cm) ───────────────
int DESCENT_CHECKPOINT_1 = 60;  // First pause height — use SDK "down" to reach
int DESCENT_CHECKPOINT_2 = 40;  // Second pause height — v7.1: raised from 35 (Tello can't RC below ~40)
int RECHECK_COUNT = 3;          // Quick re-centering confirmations at each checkpoint

// ─────────────── PROPORTIONAL CONTROL ───────────────
int MIN_NUDGE_SPEED = 10;     // v7.2: raised from 8 (8 too weak at lower altitudes)
int MAX_NUDGE_SPEED = 20;     // v7.2: raised from 18
float PROPORTIONAL_GAIN = 0.6; // v7.2: raised from 0.5 (more responsive)

// ─────────────── LANDING STYLE ───────────────
bool USE_EMERGENCY_LAND = false;  // true = drop, false = gentle

// ─────────────── ORIENTATION FIX ───────────────
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

// Calculate proportional speed: bigger error = faster correction
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

// Check serial for emergency commands. Returns true if emergency triggered.
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
//                    HOVER AND SEARCH FOR PAD
// ══════════════════════════════════════════════════════════════

bool hoverAndSearchForPad() {
    Serial.println("\n----------- SEARCHING FOR PAD (hovering) -----------");
    Serial.printf("Timeout: %d seconds\n", SEARCH_TIMEOUT / 1000);

    unsigned long searchStart = millis();
    int lastSecondPrinted = -1;

    while (millis() - searchStart < (unsigned long)SEARCH_TIMEOUT) {

        if (checkEmergency()) return false;

        readState();

        int elapsedSec = (millis() - searchStart) / 1000;
        int remainingSec = (SEARCH_TIMEOUT / 1000) - elapsedSec;

        if (elapsedSec != lastSecondPrinted) {
            lastSecondPrinted = elapsedSec;

            if (padId > 0) {
                Serial.printf("PAD FOUND! ID=%d | X:%d Y:%d Z:%d\n", padId, padX, padY, padZ);
                return true;
            } else {
                Serial.printf("Searching... %d seconds remaining (no pad)\n", remainingSec);
            }
        }

        delay(100);
    }

    Serial.println("SEARCH TIMEOUT - Pad not found!");
    return false;
}

// ══════════════════════════════════════════════════════════════
//  CENTER ON PAD — PROPORTIONAL CONTROL (v7)
//
//  Key differences from v4:
//    - Proportional speed (not fixed 15)
//    - Tighter zone (5cm not 15cm)
//    - Fewer confirmations needed (15 not 25)
// ══════════════════════════════════════════════════════════════

bool centerOnPad() {
    Serial.println("\n----------- CENTERING ON PAD (proportional v7) -----------");
    Serial.printf("Target: X=%d, Y=%d | Zone: +/-%dcm | Need %d confirmations\n",
                  PAD_X_OFFSET, PAD_Y_OFFSET, TARGET_ZONE, CENTERED_COUNT_NEEDED);

    centeredCount = 0;
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 25000;  // 25 second timeout

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
//  QUICK RE-CENTER CHECK
//
//  Used during staged descent. Does a quick centering verification
//  without the full timeout. If pad is lost or too far off, returns false.
// ══════════════════════════════════════════════════════════════

bool quickRecenter(int confirmationsNeeded) {
    // v7.2: Uses QUICK_TARGET_ZONE (wider), longer timeout, prints progress
    Serial.printf("   Quick re-center (zone=%dcm, need %d confirms, 15s timeout)...\n",
                  QUICK_TARGET_ZONE, confirmationsNeeded);

    int count = 0;
    unsigned long start = millis();
    unsigned long lastPrint = 0;
    const unsigned long TIMEOUT = 15000;  // v7.2: 15s (was 8s — too short!)
    bool gotFreshData = false;

    while (millis() - start < TIMEOUT && flightActive) {

        if (checkEmergency()) return false;

        bool fresh = readState();
        if (!fresh) {
            delay(50);
            continue;  // Don't act on stale data
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

        // Use wider zone for intermediate re-centers
        bool centered = abs(errorX) <= QUICK_TARGET_ZONE && abs(errorY) <= QUICK_TARGET_ZONE;

        // Print progress every 1 second
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

            // Proportional correction using QUICK_TARGET_ZONE
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
//  STAGED DESCENT AND LAND ON PAD (v7.1)
//
//  v7 LESSON: RC throttle (-15) CANNOT push Tello below ~40cm.
//  The Tello's internal height controller fights it, drone stalls,
//  XY drifts, and pad detection is lost.
//
//  v7.1 FIX: Use SDK "down X" commands for altitude changes.
//  These go through Tello's internal PID which actually descends.
//  RC is used ONLY for XY corrections while hovering at checkpoints.
//
//  Sequence:
//    1. SDK "down 30" → drops from ~88cm to ~58cm
//    2. PAUSE: re-center at ~58cm
//    3. SDK "down 20" → drops from ~58cm to ~38cm
//    4. PAUSE: re-center at ~38cm
//    5. "land" → from ~38cm, only ~1s of uncontrolled drift
// ══════════════════════════════════════════════════════════════

bool descendAndLandOnPad() {
    Serial.println("\n----------- STAGED DESCENT (v7.1 — SDK down commands) -----------");

    // Read current height — wait 2s then flush to get truly fresh data
    delay(2000);
    for (int i = 0; i < 10; i++) { readState(); delay(100); }
    int currentHeight = abs(padZ);
    if (currentHeight == 0) currentHeight = 88;  // Fallback estimate
    Serial.printf("Current height: ~%dcm\n", currentHeight);

    // ── Stage 1: Descend to checkpoint 1 (~60cm) ──
    int drop1 = currentHeight - DESCENT_CHECKPOINT_1;
    if (drop1 >= 20) {  // v7.2 fix: >= 20 (was > 20, skipped when exactly 20!)
        Serial.printf("\n[Stage 1] Descending %dcm (SDK down command)...\n", drop1);
        char cmd[30];
        sprintf(cmd, "down %d", drop1);
        sendWithRetry(cmd, 0);

        if (checkEmergency()) return false;

        // Wait for Tello to settle, then flush stale state
        delay(2000);
        for (int i = 0; i < 10; i++) { readState(); delay(100); }
        Serial.printf("   After descent: Z=%dcm | X:%d Y:%d\n", padZ, padX, padY);

        // Re-center at checkpoint 1
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
    if (currentHeight == 0) currentHeight = 58;  // Fallback

    int drop2 = currentHeight - DESCENT_CHECKPOINT_2;
    if (drop2 >= 20) {  // v7.2 fix: >= 20
        Serial.printf("\n[Stage 2] Descending %dcm (SDK down command)...\n", drop2);
        char cmd[30];
        sprintf(cmd, "down %d", drop2);
        sendWithRetry(cmd, 0);

        if (checkEmergency()) return false;

        // Wait for settle, flush stale state
        delay(2000);
        for (int i = 0; i < 10; i++) { readState(); delay(100); }
        Serial.printf("   After descent: Z=%dcm | X:%d Y:%d\n", padZ, padX, padY);

        // Re-center at checkpoint 2
        Serial.println("   Re-centering at checkpoint 2...");
        quickRecenter(RECHECK_COUNT);
    } else if (drop2 > 0) {
        // Can't use SDK "down" for less than 20cm, try a small RC burst
        Serial.printf("[Stage 2] Only %dcm to drop (< 20 minimum). Using short RC burst...\n", drop2);
        for (int i = 0; i < 15; i++) {  // ~1.5 sec of gentle descent
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

    // ── Final: Re-center one last time, then IMMEDIATELY land ──
    Serial.println("\n[FINAL CENTER] Last centering before land...");
    quickRecenter(RECHECK_COUNT);

    // Read position and land IMMEDIATELY — no delays between center and land
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
    Serial.println("     PRECISION COMPETITION SEQUENCE (v7)");
    Serial.println("==========================================================\n");

    Serial.println("PHASE 1 MOVEMENT:");
    Serial.printf("  [2] Forward:  %dcm (safety)\n", SAFETY_FORWARD);
    Serial.printf("  [3] Right:    %dcm\n", MOVE_RIGHT_1);
    Serial.printf("  [4] Rotate:   %d deg right (cw)\n", ROTATE_DEGREES);
    Serial.printf("  [5] Forward:  %dcm\n", MOVE_FORWARD_2);
    Serial.printf("  [6] Left:     %dcm\n", MOVE_LEFT_1);
    Serial.println();
    Serial.println("PAD LANDING (v7 precision):");
    Serial.printf("  Target zone:    +/-%dcm\n", TARGET_ZONE);
    Serial.printf("  Confirmations:  %d\n", CENTERED_COUNT_NEEDED);
    Serial.printf("  RC speed range: %d-%d (proportional)\n", MIN_NUDGE_SPEED, MAX_NUDGE_SPEED);
    Serial.printf("  Descent stages: SDK down to %dcm -> %dcm -> land\n",
                  DESCENT_CHECKPOINT_1, DESCENT_CHECKPOINT_2);
    Serial.printf("  Stabilize hold: %dms\n", STABILIZE_AFTER_CENTER);
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

    // Step 1.5: Enable mission pad detection (CRITICAL - v4 was missing this!)
    Serial.println("[1.5] Enabling mission pad detection...");
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);  // Downward only = 20Hz (faster)
    Serial.println("    Pad detection ON (downward, 20Hz)\n");

    // Step 2: Forward (safety, away from net)
    Serial.printf("[2] FORWARD %dcm (away from net)...\n", SAFETY_FORWARD);
    sprintf(cmd, "forward %d", SAFETY_FORWARD);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // Step 3: Right
    Serial.printf("[3] RIGHT %dcm...\n", MOVE_RIGHT_1);
    sprintf(cmd, "right %d", MOVE_RIGHT_1);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    // Step 4: Rotate 270 right (cw) — equivalent to 90 left
    Serial.printf("[4] ROTATE %d deg RIGHT (cw)...\n", ROTATE_DEGREES);
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
    // PHASE 2: Precision Pad Landing
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 2: PRECISION PAD LANDING ========\n");

    // Step 7: Search for pad
    Serial.println("[7] SEARCHING FOR PAD...");
    bool padFound = hoverAndSearchForPad();

    if (padFound) {
        // Step 8: Center on pad (proportional control)
        Serial.println("\n[8] CENTERING (proportional control)...");
        bool centered = centerOnPad();

        if (centered) {
            // Step 9: Stabilization hold — kill momentum
            Serial.printf("\n[9] STABILIZING (%dms hold)...\n", STABILIZE_AFTER_CENTER);
            stopMovement();
            delay(STABILIZE_AFTER_CENTER);

            // Verify we're still centered after the hold
            readState();
            int errX = abs(padX - PAD_X_OFFSET);
            int errY = abs(padY - PAD_Y_OFFSET);
            Serial.printf("    Post-hold position: X:%d Y:%d Z:%d (err X:%d Y:%d)\n",
                          padX, padY, padZ, errX, errY);

            if (errX > TARGET_ZONE * 2 || errY > TARGET_ZONE * 2) {
                Serial.println("    Drifted during hold! Re-centering...");
                centered = centerOnPad();
                if (centered) {
                    stopMovement();
                    delay(STABILIZE_AFTER_CENTER);
                }
            }

            // Step 10: Staged descent
            Serial.println("\n[10] STAGED DESCENT...");
            descendAndLandOnPad();
        } else {
            Serial.println("Centering failed - landing wherever...");
            performLand();
        }
    } else {
        Serial.println("\n[8] PAD NOT FOUND - Landing wherever...");
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
    Serial.println("   TELLO v7 — PRECISION PAD LANDING");
    Serial.println("==========================================================\n");

    Serial.println("Improvements over v4:");
    Serial.println("  - Mission pad detection explicitly enabled");
    Serial.println("  - Proportional RC control (no more oscillation)");
    Serial.println("  - Tighter 5cm centering / 8cm quick-check zones");
    Serial.println("  - Land from ~40cm via SDK 'down' (was 45cm)");
    Serial.println("  - Stabilization hold after centering");
    Serial.println("  - Staged descent + final re-center before land (v7.2)");
    Serial.println();

    // Print settings
    Serial.println("WIFI:");
    Serial.printf("  SSID:     %s\n", WIFI_NAME);
    Serial.printf("  Password: %s\n", WIFI_PASSWORD);
    Serial.println();

    Serial.println("MOVEMENT:");
    Serial.printf("  Safety forward:  %dcm\n", SAFETY_FORWARD);
    Serial.printf("  Right:           %dcm\n", MOVE_RIGHT_1);
    Serial.printf("  Rotate:          %d deg right\n", ROTATE_DEGREES);
    Serial.printf("  Forward:         %dcm\n", MOVE_FORWARD_2);
    Serial.printf("  Left:            %dcm\n", MOVE_LEFT_1);
    Serial.println();

    Serial.println("PRECISION LANDING:");
    Serial.printf("  Target zone:     +/-%dcm\n", TARGET_ZONE);
    Serial.printf("  Confirmations:   %d\n", CENTERED_COUNT_NEEDED);
    Serial.printf("  RC speed:        %d-%d (proportional, gain=%.1f)\n",
                  MIN_NUDGE_SPEED, MAX_NUDGE_SPEED, PROPORTIONAL_GAIN);
    Serial.printf("  Descent stages:  SDK down to %dcm -> %dcm -> land\n",
                  DESCENT_CHECKPOINT_1, DESCENT_CHECKPOINT_2);
    Serial.printf("  Pad offsets:     X=%d Y=%d\n", PAD_X_OFFSET, PAD_Y_OFFSET);
    Serial.printf("  Stab hold:       %dms\n", STABILIZE_AFTER_CENTER);
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

    // Enable mission pad detection early (so it's ready before takeoff)
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
    Serial.println("  s = START precision sequence");
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
