// ══════════════════════════════════════════════════════════════
//  ESP32 PHASE CONTROL — TELLO DRONE
//
//  Boots up, connects WiFi + SDK, then WAITS for start trigger.
//  Type 'S' in Serial Monitor (or send 'S' from Mega on Serial2).
//
//  PHASES (15s delay between each):
//    Phase 1: Takeoff + forward 40cm (hover in place)
//    Phase 2: CW 180 + forward 40 + land (return to start)
//    Phase 3: Takeoff + CW 180 + right 20 + find pad + go
//             descent + RC fine-tune + precision land
//
//  STATUS OUTPUT (after each phase):
//    USB Serial: [STATUS] PHASE X: OK / FAIL
//    Serial2 → Mega: "1" = success, "0" = fail
//    After all done: Serial2 sends "DONE"
//
//  EMERGENCY: Type 'E' (emergency stop) or 'L' (land) in Serial Monitor
//
//  WIRING (ESP32 ↔ Mega):
//    ESP32 GPIO 16 (RX2) ← [voltage divider] ← Mega TX1 (pin 18)
//    ESP32 GPIO 17 (TX2) → Mega RX1 (pin 19)
//    ESP32 GND ← Mega GND
//    ESP32 VIN ← Mega 5V
// ══════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <WiFiUdp.h>

// ══════════════════════════════════════════════════════════════
//                    EASY TO CHANGE VARIABLES
// ══════════════════════════════════════════════════════════════

// ─────────────── WIFI ───────────────
const char* WIFI_NAME = "TELLO-BCU-DRONE";
const char* WIFI_PASSWORD = "southeast2026";

// ─────────────── SERIAL2 PINS (Mega communication) ───────────
#define MEGA_RX 16   // ESP32 RX2 ← Mega TX1
#define MEGA_TX 17   // ESP32 TX2 → Mega RX1
#define MEGA_BAUD 9600

// ─────────────── DELAYS BETWEEN PHASES ───────────────
int PHASE_DELAY = 15000;        // ms — wait between phases

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
bool sequenceStarted = false;    // true once 'S' received
int padId = -1;
int padX = 0, padY = 0, padZ = 0;
unsigned long lastStateRead = 0;
unsigned long lastPrintTime = 0;
int batteryLevel = 0;

// ══════════════════════════════════════════════════════════════
//                    MEGA COMMUNICATION
// ══════════════════════════════════════════════════════════════

void sendToMega(const char* msg) {
    Serial2.println(msg);
    Serial.printf("[→ MEGA] %s\n", msg);
}

void sendPhaseStatus(int phase, bool success) {
    // USB Serial — human readable
    Serial.printf("[STATUS] PHASE %d: %s\n", phase, success ? "OK" : "FAIL");
    // Serial2 — machine readable for Mega
    Serial2.println(success ? "1" : "0");
}

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
    // USB Serial — emergency from laptop
    if (Serial.available()) {
        char c = toupper(Serial.read());
        if (c == 'E') {
            Serial.println("\n!!! EMERGENCY (USB) !!!");
            sendCommand("emergency");
            flightActive = false;
            sendToMega("0");
            return true;
        }
        if (c == 'L') {
            Serial.println("\nManual land (USB)...");
            sendCommand("land");
            flightActive = false;
            sendToMega("0");
            return true;
        }
    }
    // Serial2 — emergency from Mega
    if (Serial2.available()) {
        String cmd = Serial2.readStringUntil('\n');
        cmd.trim();
        if (cmd == "E") {
            Serial.println("\n!!! EMERGENCY (from Mega) !!!");
            sendCommand("emergency");
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
//  PHASE 1: Takeoff + forward 40cm (then hover)
// ══════════════════════════════════════════════════════════════

bool executePhase1() {
    Serial.println("\n==========================================================");
    Serial.println("     PHASE 1: TAKEOFF + FORWARD");
    Serial.println("==========================================================\n");

    flightActive = true;
    char cmd[30];

    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    Serial.println("[1] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("    TAKEOFF FAILED!");
        flightActive = false;
        return false;
    }
    if (checkEmergency() || !flightActive) return false;

    Serial.printf("[2] FORWARD %dcm...\n", MOVE_AWAY_DISTANCE);
    sprintf(cmd, "forward %d", MOVE_AWAY_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return false;

    Serial.println("\n     PHASE 1 COMPLETE — hovering\n");
    return true;
}

// ══════════════════════════════════════════════════════════════
//  PHASE 2: CW 180 + forward 40 + land
// ══════════════════════════════════════════════════════════════

bool executePhase2() {
    Serial.println("\n==========================================================");
    Serial.println("     PHASE 2: RETURN + LAND");
    Serial.println("==========================================================\n");

    flightActive = true;
    char cmd[30];

    Serial.printf("[1] CW %d...\n", TURN_DEGREES);
    sprintf(cmd, "cw %d", TURN_DEGREES);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return false;

    Serial.printf("[2] FORWARD %dcm (return)...\n", RETURN_DISTANCE);
    sprintf(cmd, "forward %d", RETURN_DISTANCE);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return false;

    Serial.println("[3] LANDING...");
    performLand();

    Serial.println("\n     PHASE 2 COMPLETE — on ground\n");
    return true;
}

// ══════════════════════════════════════════════════════════════
//  PHASE 3: Takeoff + reposition + precision land
// ══════════════════════════════════════════════════════════════

bool executePhase3() {
    Serial.println("\n==========================================================");
    Serial.println("     PHASE 3: REPOSITION + PRECISION LAND");
    Serial.println("==========================================================\n");

    flightActive = true;
    char cmd[30];

    Serial.println("[1] Re-entering SDK + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    flushState();

    Serial.println("[2] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("    TAKEOFF FAILED!");
        flightActive = false;
        return false;
    }
    if (checkEmergency() || !flightActive) return false;

    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    Serial.println("[3] CW 180 (face original direction)...");
    sendWithRetry("cw 180", PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return false;

    Serial.printf("[4] RIGHT %dcm...\n", REPOSITION_RIGHT);
    sprintf(cmd, "right %d", REPOSITION_RIGHT);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    if (checkEmergency() || !flightActive) return false;

    Serial.println("[5] Looking for pad...");
    bool padFound = waitForPad(10000);

    if (!padFound) {
        Serial.println("   No pad found! Trying RC centering...");
        if (!centerOnPadRC()) {
            Serial.println("   Failed -- landing wherever.");
            performLand();
            return false;
        }
    }
    if (!flightActive) return false;

    Serial.printf("\n[6] GO descent: ");
    for (int i = 0; i < NUM_GO_STEPS; i++) Serial.printf("%d ", GO_HEIGHTS[i]);
    Serial.println();

    for (int i = 0; i < NUM_GO_STEPS; i++) {
        Serial.printf("\n   GO to %dcm...\n", GO_HEIGHTS[i]);

        bool goOk = goToCenter(GO_HEIGHTS[i], GO_SPEED);
        if (checkEmergency() || !flightActive) return false;

        if (!goOk && i == 0) {
            Serial.println("   First go failed! Trying RC centering...");
            if (!centerOnPadRC()) {
                Serial.println("   Failed -- landing wherever.");
                performLand();
                return false;
            }
        } else if (!goOk) {
            Serial.printf("   go step %d failed -- proceeding.\n", i + 1);
        }

        if (checkEmergency() || !flightActive) return false;
    }

    Serial.println("\n[7] RC fine-tuning...");
    fineCenterRC();
    if (checkEmergency() || !flightActive) return false;

    Serial.println("\n[8] PRECISION LANDING...");
    performLand();

    Serial.println("\n     PHASE 3 COMPLETE\n");
    return true;
}

// ══════════════════════════════════════════════════════════════
//  RUN ALL PHASES (called once after start trigger)
// ══════════════════════════════════════════════════════════════

void runSequence() {
    Serial.println("\n**********************************************************");
    Serial.println("          SEQUENCE STARTED");
    Serial.println("**********************************************************\n");

    // Phase 1
    bool p1 = executePhase1();
    sendPhaseStatus(1, p1);

    // 15s delay
    Serial.printf("\n>>> WAITING %d seconds before Phase 2 <<<\n", PHASE_DELAY / 1000);
    for (int i = PHASE_DELAY / 1000; i > 0; i--) {
        Serial.printf("   %d...\n", i);
        if (checkEmergency()) return;
        delay(1000);
    }

    // Phase 2
    bool p2 = executePhase2();
    sendPhaseStatus(2, p2);

    // 15s delay
    Serial.printf("\n>>> WAITING %d seconds before Phase 3 <<<\n", PHASE_DELAY / 1000);
    for (int i = PHASE_DELAY / 1000; i > 0; i--) {
        Serial.printf("   %d...\n", i);
        if (checkEmergency()) return;
        delay(1000);
    }

    // Phase 3
    bool p3 = executePhase3();
    sendPhaseStatus(3, p3);

    // All done
    sendToMega("DONE");

    Serial.println("\n**********************************************************");
    Serial.println("          ALL PHASES COMPLETE!");
    Serial.printf("          Results: P1=%s  P2=%s  P3=%s\n",
                  p1 ? "OK" : "FAIL", p2 ? "OK" : "FAIL", p3 ? "OK" : "FAIL");
    Serial.println("**********************************************************\n");
}

// ══════════════════════════════════════════════════════════════
//                    SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial2.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX, MEGA_TX);

    Serial.println("\n\n==========================================================");
    Serial.println("   ESP32 PHASE CONTROL — TELLO DRONE");
    Serial.println("==========================================================\n");

    Serial.println("PLAN:");
    Serial.println("  Phase 1: takeoff → fwd 40 (hover)");
    Serial.printf("  Wait %ds\n", PHASE_DELAY / 1000);
    Serial.println("  Phase 2: CW 180 → fwd 40 → land");
    Serial.printf("  Wait %ds\n", PHASE_DELAY / 1000);
    Serial.printf("  Phase 3: takeoff → CW 180 → right %d → find pad → go descent → land\n\n",
                  REPOSITION_RIGHT);

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
        sendToMega("0");
        while (1) delay(1000);
    }
    Serial.println(" Connected!\n");

    // Setup UDP
    udpCmd.begin(CMD_PORT);
    udpState.begin(STATE_PORT);
    delay(1000);

    // Enter SDK mode + battery check
    sendWithRetry("command", 500);
    sendCommand("battery?");
    String bat = waitResponse(3000);
    Serial.printf("Battery: %s%%\n\n", bat.c_str());

    // Signal ready
    sendToMega("READY");
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║  READY — Type 'S' to start sequence     ║");
    Serial.println("║  Emergency: 'E' = stop, 'L' = land      ║");
    Serial.println("╚══════════════════════════════════════════╝\n");
}

// ══════════════════════════════════════════════════════════════
//                    LOOP
// ══════════════════════════════════════════════════════════════

void loop() {
    // Already ran the sequence — just handle emergency
    if (sequenceStarted) {
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
        return;
    }

    // Check USB Serial for start trigger
    if (Serial.available()) {
        char c = toupper(Serial.read());
        if (c == 'S' || c == 'G') {
            sequenceStarted = true;
            runSequence();
            return;
        }
        if (c == 'E') {
            Serial.println("\n!!! EMERGENCY !!!");
            sendCommand("emergency");
        } else if (c == 'L') {
            Serial.println("\nLanding...");
            sendCommand("land");
        }
    }

    // Check Serial2 for start from Mega
    if (Serial2.available()) {
        String cmd = Serial2.readStringUntil('\n');
        cmd.trim();
        if (cmd == "S" || cmd == "G") {
            sequenceStarted = true;
            runSequence();
            return;
        }
        if (cmd == "E") {
            Serial.println("\n!!! EMERGENCY (from Mega) !!!");
            sendCommand("emergency");
        }
    }

    delay(10);
}
