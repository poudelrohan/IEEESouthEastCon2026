// ============================================================
//  DRONE-ONLY TEST SKETCH
//  No robot libraries — just WiFi + UDP to Tello
//  Tests the full drone sequence from full_mission.ino
//
//  FLIGHT 1 (simulates robot going forward):
//    Takeoff → Forward 50cm → 180 turn → Forward 50cm → Land
//
//  FLIGHT 2 (simulates pad landing):
//    Takeoff → Enable pad detection → Left 20cm →
//    Search for pad → Center on pad → Descend and land
//
//  Serial Monitor: 115200 baud
//  Send 'E' for emergency motor kill at any time
//  Send 'L' for gentle land at any time
// ============================================================

#include <WiFi.h>
#include <WiFiUdp.h>

// ══════════════════════════════════════════════════════════════
//  CONFIGURATION
// ══════════════════════════════════════════════════════════════

const char* TELLO_SSID     = "TELLO-BCU-DRONE";
const char* TELLO_PASS     = "southeast2026";
const char* TELLO_IP       = "192.168.10.1";
const int   CMD_PORT       = 8889;
const int   STATE_PORT     = 8890;

const int   WIFI_TIMEOUT_S = 30;
const int   CMD_TIMEOUT_MS = 10000;
const int   STABILIZE_MS   = 2000;     // Wait after takeoff
const int   PAUSE_AFTER_MS = 2000;     // Wait after each movement for stability

// Flight 1 parameters
const int   FWD_DISTANCE   = 50;       // cm forward
const int   RETURN_DISTANCE = 50;      // cm return after 180

// Flight 2 parameters
const int   LEFT_DISTANCE  = 20;       // cm left toward robot

// Pad landing parameters
const int   PAD_SEARCH_TIMEOUT_MS  = 8000;
const int   PAD_CENTER_TIMEOUT_MS  = 10000;
const int   PAD_DESCEND_TIMEOUT_MS = 10000;
const int   PAD_TARGET_ZONE        = 15;
const int   PAD_CENTERED_NEEDED    = 10;
const int   PAD_LAND_HEIGHT        = 45;
const int   PAD_NUDGE_SPEED        = 15;
const int   PAD_X_OFFSET           = 0;
const int   PAD_Y_OFFSET           = 0;
const bool  INVERT_PITCH           = false;
const bool  INVERT_ROLL            = true;

// What to test — set false to skip parts
const bool  DO_FLIGHT_1    = true;     // First flight (fwd + 180 + return + land)
const bool  DO_FLIGHT_2    = true;     // Second flight (pad landing)
const int   BETWEEN_FLIGHTS_SEC = 5;   // Rest between flights

// ══════════════════════════════════════════════════════════════
//  STATE
// ══════════════════════════════════════════════════════════════

WiFiUDP udpCmd;
WiFiUDP udpState;
bool flightActive = false;
int  padId = -1;
int  padX = 0, padY = 0, padZ = 0;
int  centeredCount = 0;
unsigned long lastPrintTime = 0;

// ══════════════════════════════════════════════════════════════
//  COMMUNICATION
// ══════════════════════════════════════════════════════════════

void sendCommand(const char* cmd) {
    Serial.printf(">> %s\n", cmd);
    udpCmd.beginPacket(TELLO_IP, CMD_PORT);
    udpCmd.print(cmd);
    udpCmd.endPacket();
}

String waitResponse(int timeoutMs = CMD_TIMEOUT_MS) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)timeoutMs) {
        // Check for emergency serial input
        if (Serial.available()) {
            char c = toupper(Serial.read());
            if (c == 'E') {
                Serial.println("\n!!! EMERGENCY !!!");
                sendCommand("emergency");
                flightActive = false;
                return "EMERGENCY";
            }
            if (c == 'L') {
                Serial.println("\nManual land...");
                sendCommand("land");
                flightActive = false;
                return "MANUAL_LAND";
            }
        }

        int size = udpCmd.parsePacket();
        if (size) {
            char buf[256];
            int len = udpCmd.read(buf, 255);
            buf[len] = 0;
            Serial.printf("<< %s\n", buf);
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

    if (resp == "EMERGENCY" || resp == "MANUAL_LAND") return false;

    if (resp != "ok") {
        Serial.println("   Retrying...");
        delay(500);
        sendCommand(cmd);
        resp = waitResponse();
    }

    if (resp == "EMERGENCY" || resp == "MANUAL_LAND") return false;
    if (delayAfter > 0) delay(delayAfter);
    return (resp == "ok");
}

// ══════════════════════════════════════════════════════════════
//  DRONE STATE READING
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

    int idx;

    idx = latest.indexOf("mid:");
    if (idx != -1) padId = latest.substring(idx + 4, latest.indexOf(";", idx)).toInt();

    idx = latest.indexOf(";x:");
    if (idx != -1) padX = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();

    idx = latest.indexOf(";y:");
    if (idx != -1) padY = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();

    idx = latest.indexOf(";z:");
    if (idx != -1) padZ = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();

    return true;
}

void sendRC(int roll, int pitch, int throttle, int yaw) {
    char cmd[50];
    sprintf(cmd, "rc %d %d %d %d", roll, pitch, throttle, yaw);
    sendCommand(cmd);
}

// ══════════════════════════════════════════════════════════════
//  PAD LANDING FUNCTIONS
// ══════════════════════════════════════════════════════════════

bool searchForPad() {
    Serial.println("\n[PAD] Searching for mission pad...");
    unsigned long start = millis();
    int lastSec = -1;

    while (millis() - start < (unsigned long)PAD_SEARCH_TIMEOUT_MS) {
        readState();

        int sec = (millis() - start) / 1000;
        int remain = (PAD_SEARCH_TIMEOUT_MS / 1000) - sec;

        if (sec != lastSec) {
            lastSec = sec;
            if (padId > 0) {
                Serial.printf("[PAD] FOUND! ID=%d X:%d Y:%d Z:%d\n", padId, padX, padY, padZ);
                return true;
            }
            Serial.printf("[PAD] Searching... %ds remaining\n", remain);
        }
        delay(100);
    }

    Serial.println("[PAD] Search TIMEOUT — pad not found");
    return false;
}

bool centerOnPad() {
    Serial.println("[PAD] Centering...");
    centeredCount = 0;
    unsigned long start = millis();

    while (millis() - start < (unsigned long)PAD_CENTER_TIMEOUT_MS) {
        readState();

        if (padId <= 0) {
            Serial.println("[PAD] Lost pad — hovering...");
            sendRC(0, 0, 0, 0);
            centeredCount = 0;
            delay(500);
            continue;
        }

        int errX = padX - PAD_X_OFFSET;
        int errY = padY - PAD_Y_OFFSET;
        bool centered = (abs(errX) <= PAD_TARGET_ZONE) && (abs(errY) <= PAD_TARGET_ZONE);

        if (centered) centeredCount++;
        else centeredCount = 0;

        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            Serial.printf("[PAD] X:%4d Y:%4d Z:%3d | Err X:%4d Y:%4d | ", padX, padY, padZ, errX, errY);
            if (centered) Serial.printf("CENTERED [%d/%d]\n", centeredCount, PAD_CENTERED_NEEDED);
            else Serial.println("Adjusting...");
        }

        if (centeredCount >= PAD_CENTERED_NEEDED) {
            Serial.println("[PAD] CENTERING COMPLETE!");
            sendRC(0, 0, 0, 0);
            return true;
        }

        int roll = 0, pitch = 0;
        if (errX > PAD_TARGET_ZONE)       pitch = -PAD_NUDGE_SPEED;
        else if (errX < -PAD_TARGET_ZONE) pitch =  PAD_NUDGE_SPEED;
        if (errY > PAD_TARGET_ZONE)       roll = -PAD_NUDGE_SPEED;
        else if (errY < -PAD_TARGET_ZONE) roll =  PAD_NUDGE_SPEED;

        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL)  roll  = -roll;

        sendRC(roll, pitch, 0, 0);
        delay(100);
    }

    Serial.println("[PAD] Centering TIMEOUT!");
    return false;
}

bool descendAndLand() {
    Serial.println("[PAD] Descending...");
    unsigned long start = millis();

    while (millis() - start < (unsigned long)PAD_DESCEND_TIMEOUT_MS) {
        readState();

        int errX = padX - PAD_X_OFFSET;
        int errY = padY - PAD_Y_OFFSET;

        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            if (padId > 0) Serial.printf("[PAD] DESCENDING | Z:%3d | Err X:%4d Y:%4d\n", padZ, errX, errY);
            else Serial.println("[PAD] DESCENDING | Pad lost — continuing...");
        }

        if (padId > 0 && padZ < PAD_LAND_HEIGHT) {
            Serial.printf("[PAD] Z=%dcm — landing!\n", padZ);
            break;
        }

        if (padId <= 0) {
            Serial.println("[PAD] Pad lost — landing now!");
            break;
        }

        int roll = 0, pitch = 0;
        if (errX > PAD_TARGET_ZONE)       pitch = -PAD_NUDGE_SPEED;
        else if (errX < -PAD_TARGET_ZONE) pitch =  PAD_NUDGE_SPEED;
        if (errY > PAD_TARGET_ZONE)       roll = -PAD_NUDGE_SPEED;
        else if (errY < -PAD_TARGET_ZONE) roll =  PAD_NUDGE_SPEED;

        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL)  roll  = -roll;

        sendRC(roll, pitch, -15, 0);
        delay(100);
    }

    Serial.println("[PAD] Sending land...");
    sendCommand("land");
    waitResponse();
    flightActive = false;
    return true;
}

// ══════════════════════════════════════════════════════════════
//  SETUP — auto-runs everything
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n\n══════════════════════════════════════════");
    Serial.println("  DRONE FULL-SEQUENCE TEST");
    Serial.println("══════════════════════════════════════════\n");

    Serial.printf("Flight 1 (fwd+180+return): %s\n", DO_FLIGHT_1 ? "YES" : "SKIP");
    Serial.printf("Flight 2 (pad landing):    %s\n", DO_FLIGHT_2 ? "YES" : "SKIP");
    Serial.println();

    // --- WiFi Connect ---
    Serial.print("[WIFI] Connecting");
    WiFi.begin(TELLO_SSID, TELLO_PASS);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < WIFI_TIMEOUT_S * 2) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n\n*** WiFi FAILED! ***");
        Serial.println("Check: Is Tello on? Wait 20s after power on.");
        while (1) delay(1000);
    }

    Serial.println(" Connected!");
    Serial.printf("  IP: %s  RSSI: %d dBm\n\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());

    // --- UDP Setup ---
    udpCmd.begin(CMD_PORT);
    udpState.begin(STATE_PORT);
    delay(1000);

    // --- SDK Mode ---
    Serial.println("[SDK] Entering SDK mode...");
    if (!sendWithRetry("command", 500)) {
        Serial.println("*** SDK FAILED! ***");
        while (1) delay(1000);
    }

    // --- Battery ---
    sendCommand("battery?");
    String bat = waitResponse(3000);
    Serial.printf("[BAT] Battery: %s%%\n\n", bat.c_str());

    // ══════════════════════════════════════════════════════════
    //  FLIGHT 1: Takeoff → Forward → 180 → Return → Land
    // ══════════════════════════════════════════════════════════

    if (DO_FLIGHT_1) {
        Serial.println("══════════════════════════════════════════");
        Serial.println("  FLIGHT 1: Forward + 180 + Return");
        Serial.println("══════════════════════════════════════════\n");

        // Takeoff
        Serial.println("[F1] Takeoff...");
        flightActive = true;
        if (!sendWithRetry("takeoff", STABILIZE_MS)) {
            Serial.println("*** Takeoff FAILED! ***");
            flightActive = false;
            goto done;
        }

        // Forward
        {
            char cmd[30];
            Serial.printf("[F1] Forward %dcm...\n", FWD_DISTANCE);
            sprintf(cmd, "forward %d", FWD_DISTANCE);
            sendWithRetry(cmd, PAUSE_AFTER_MS);
        }

        // 180 turn
        Serial.println("[F1] Rotating 180 degrees...");
        sendWithRetry("cw 180", PAUSE_AFTER_MS);

        // Return
        {
            char cmd[30];
            Serial.printf("[F1] Forward %dcm (returning)...\n", RETURN_DISTANCE);
            sprintf(cmd, "forward %d", RETURN_DISTANCE);
            sendWithRetry(cmd, PAUSE_AFTER_MS);
        }

        // Land
        Serial.println("[F1] Landing...");
        sendCommand("land");
        waitResponse();
        flightActive = false;
        Serial.println("[F1] Flight 1 complete!\n");

        // Rest between flights
        if (DO_FLIGHT_2) {
            Serial.printf("[REST] Waiting %d seconds before Flight 2...\n", BETWEEN_FLIGHTS_SEC);
            for (int i = BETWEEN_FLIGHTS_SEC; i > 0; i--) {
                Serial.printf("  %d...\n", i);
                delay(1000);
            }
        }
    }

    // ══════════════════════════════════════════════════════════
    //  FLIGHT 2: Takeoff → Left 20cm → Pad Search → Land
    // ══════════════════════════════════════════════════════════

    if (DO_FLIGHT_2) {
        Serial.println("══════════════════════════════════════════");
        Serial.println("  FLIGHT 2: Pad Landing");
        Serial.println("══════════════════════════════════════════\n");

        // Enable mission pad detection
        Serial.println("[F2] Enabling pad detection...");
        sendWithRetry("mon", 300);
        sendWithRetry("mdirection 0", 300);

        // Takeoff
        Serial.println("[F2] Takeoff...");
        flightActive = true;
        if (!sendWithRetry("takeoff", STABILIZE_MS)) {
            Serial.println("*** Takeoff FAILED! ***");
            flightActive = false;
            goto done;
        }

        // Left toward robot
        {
            char cmd[30];
            Serial.printf("[F2] Left %dcm toward robot...\n", LEFT_DISTANCE);
            sprintf(cmd, "left %d", LEFT_DISTANCE);
            sendWithRetry(cmd, PAUSE_AFTER_MS);
        }

        // Search for pad
        bool padFound = searchForPad();

        if (padFound) {
            if (centerOnPad()) {
                descendAndLand();
            } else {
                Serial.println("[F2] Centering failed — landing...");
                sendCommand("land");
                waitResponse();
                flightActive = false;
            }
        } else {
            Serial.println("[F2] Pad not found — landing...");
            sendCommand("land");
            waitResponse();
            flightActive = false;
        }

        Serial.println("[F2] Flight 2 complete!\n");
    }

done:
    Serial.println("══════════════════════════════════════════");
    Serial.println("  === DRONE TEST COMPLETE ===");
    Serial.println("══════════════════════════════════════════\n");
    Serial.println("Commands: T=takeoff, E=emergency, L=land, B=battery");
}

// ══════════════════════════════════════════════════════════════
//  LOOP — manual commands
// ══════════════════════════════════════════════════════════════

void loop() {
    if (Serial.available()) {
        char c = toupper(Serial.read());
        switch (c) {
            case 'T':
                Serial.println("\nManual takeoff...");
                sendWithRetry("takeoff", STABILIZE_MS);
                flightActive = true;
                break;
            case 'E':
                Serial.println("\n!!! EMERGENCY !!!");
                sendCommand("emergency");
                flightActive = false;
                break;
            case 'L':
                Serial.println("\nLanding...");
                sendCommand("land");
                waitResponse();
                flightActive = false;
                break;
            case 'B':
                sendCommand("battery?");
                waitResponse();
                break;
        }
    }
    delay(10);
}
