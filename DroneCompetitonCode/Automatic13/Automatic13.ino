// ══════════════════════════════════════════════════════════════
//  TELLO AUTOMATIC 13 — FULLY AUTOMATIC TWO-FLIGHT SEQUENCE
//
//  No laptop needed. Runs automatically after WiFi connects.
//
//  FLIGHT 1: Out and back
//    Takeoff → forward 40 → CW 180 → forward 40 → land
//
//  WAIT 10 seconds on ground
//
//  FLIGHT 2: Reposition + precision land
//    Re-enter SDK + pad detection
//    Takeoff → CW 180 (face original) → right 20
//    Find pad → go steps (60→50→40→30→25) → RC fine-tune → land
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
int STARTUP_DELAY = 0;          // ms — delay before starting (set later)
int STABILIZE_AFTER_TAKEOFF = 2000;
int PAUSE_AFTER_MOVE = 1000;
int GROUND_WAIT = 10000;        // ms — wait between flights
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
//  FLIGHT 1: Out and back (simple land)
// ══════════════════════════════════════════════════════════════

void flight1() {
    Serial.println("\n==========================================================");
    Serial.println("     FLIGHT 1: OUT AND BACK");
    Serial.println("==========================================================\n");

    flightActive = true;
    char cmd[30];

    // SDK + pad detection
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // Takeoff
    Serial.println("[1] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("    TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    if (checkEmergency() || !flightActive) return;

    // Forward
    Serial.printf("[2] FORWARD %dcm...\n", MOVE_AWAY_DISTANCE);
    sprintf(cmd, "forward %d", MOVE_AWAY_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return;

    // Turn
    Serial.printf("[3] CW %d...\n", TURN_DEGREES);
    sprintf(cmd, "cw %d", TURN_DEGREES);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return;

    // Return
    Serial.printf("[4] FORWARD %dcm (return)...\n", RETURN_DISTANCE);
    sprintf(cmd, "forward %d", RETURN_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return;

    // Simple land
    Serial.println("[5] LANDING (simple)...");
    performLand();

    Serial.println("\n     FLIGHT 1 COMPLETE\n");
}

// ══════════════════════════════════════════════════════════════
//  FLIGHT 2: Reposition + precision land
// ══════════════════════════════════════════════════════════════

void flight2() {
    Serial.println("\n==========================================================");
    Serial.println("     FLIGHT 2: REPOSITION + PRECISION LAND");
    Serial.println("==========================================================\n");

    flightActive = true;
    char cmd[30];

    // Re-enter SDK + pad detection
    Serial.println("[1] Re-entering SDK + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // Flush stale state from flight 1
    flushState();

    // Takeoff
    Serial.println("[2] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("    TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    if (checkEmergency() || !flightActive) return;

    // Re-enable SDK + pad detection after takeoff
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // CW 180 to face original direction
    Serial.println("[3] CW 180 (face original direction)...");
    sendWithRetry("cw 180", PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return;

    // Right 20cm to get over pad area
    Serial.printf("[4] RIGHT %dcm...\n", REPOSITION_RIGHT);
    sprintf(cmd, "right %d", REPOSITION_RIGHT);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return;

    // Wait for pad
    Serial.println("[5] Looking for pad...");
    bool padFound = waitForPad(10000);

    if (!padFound) {
        Serial.println("   No pad found! Trying RC centering...");
        if (!centerOnPadRC()) {
            Serial.println("   Failed -- landing wherever.");
            performLand();
            return;
        }
    }
    if (!flightActive) return;

    // Go steps: gradual descent
    Serial.printf("\n[6] GO descent: ");
    for (int i = 0; i < NUM_GO_STEPS; i++) Serial.printf("%d ", GO_HEIGHTS[i]);
    Serial.println();

    for (int i = 0; i < NUM_GO_STEPS; i++) {
        Serial.printf("\n   GO to %dcm...\n", GO_HEIGHTS[i]);

        bool goOk = goToCenter(GO_HEIGHTS[i], GO_SPEED);
        if (checkEmergency() || !flightActive) return;

        if (!goOk && i == 0) {
            Serial.println("   First go failed! Trying RC centering...");
            if (!centerOnPadRC()) {
                Serial.println("   Failed -- landing wherever.");
                performLand();
                return;
            }
        } else if (!goOk) {
            Serial.printf("   go step %d failed -- proceeding.\n", i + 1);
        }

        if (checkEmergency() || !flightActive) return;
    }

    // RC fine-tune
    Serial.println("\n[7] RC fine-tuning...");
    fineCenterRC();
    if (checkEmergency() || !flightActive) return;

    // Land
    Serial.println("\n[8] PRECISION LANDING...");
    performLand();

    Serial.println("\n     FLIGHT 2 COMPLETE\n");
}

// ══════════════════════════════════════════════════════════════
//                    SETUP (runs everything)
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n==========================================================");
    Serial.println("   TELLO AUTOMATIC 13 -- FULLY AUTOMATIC");
    Serial.println("==========================================================\n");

    Serial.println("PLAN:");
    Serial.println("  Flight 1: takeoff -> fwd 40 -> CW 180 -> fwd 40 -> land");
    Serial.printf("  Wait %d seconds\n", GROUND_WAIT / 1000);
    Serial.printf("  Flight 2: takeoff -> CW 180 -> right %d -> find pad -> go descent -> RC fine -> land\n\n",
                  REPOSITION_RIGHT);

    // Startup delay (set later for competition)
    if (STARTUP_DELAY > 0) {
        Serial.printf("Waiting %d seconds before starting...\n", STARTUP_DELAY / 1000);
        delay(STARTUP_DELAY);
    }

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
        while (1) delay(1000);
    }
    Serial.println(" Connected!\n");

    // Setup UDP
    udpCmd.begin(CMD_PORT);
    udpState.begin(STATE_PORT);
    delay(1000);

    // Battery check
    sendWithRetry("command", 500);
    sendCommand("battery?");
    String bat = waitResponse(3000);
    Serial.printf("Battery: %s%%\n", bat.c_str());

    // ═══════════════════════ RUN EVERYTHING ═══════════════════════

    flight1();

    // Wait on ground between flights
    Serial.printf("\n>>> GROUND WAIT: %d seconds <<<\n", GROUND_WAIT / 1000);
    for (int i = GROUND_WAIT / 1000; i > 0; i--) {
        Serial.printf("   %d...\n", i);
        delay(1000);
    }

    flight2();

    Serial.println("\n==========================================================");
    Serial.println("          ALL FLIGHTS COMPLETE!");
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
