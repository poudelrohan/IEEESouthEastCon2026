// ══════════════════════════════════════════════════════════════
//  SHORTCUT — Go to 25cm slowly, land
//
//  No forward/turn/return. Just:
//    1. Takeoff
//    2. go -5 0 25 10 m<id>  (very slow, straight down to 25cm)
//    3. Land
//
//  For testing landing accuracy without the full flight path.
//
//  Commands: S=Start  B=Battery  E=Emergency  L=Land
// ══════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <WiFiUdp.h>

// ─────────────── WIFI ───────────────
const char* WIFI_NAME = "TELLO-OLLET";
const char* WIFI_PASSWORD = "southeast2026";

// ─────────────── GO COMMAND ───────────────
int GO_HEIGHT = 40;       // cm — target height
int GO_SPEED = 10;        // cm/s — slowest possible
int GO_X_OFFSET = -9;     // cm — compensate go drift (-5) + land drift (-4)
int GO_Y_OFFSET = 0;
bool USE_EMERGENCY_LAND = false;  // true = motors cut (no drift), false = normal land

// ─────────────── NETWORK ───────────────
const char* TELLO_IP = "192.168.10.1";
const int CMD_PORT = 8889;
const int STATE_PORT = 8890;

WiFiUDP udpCmd;
WiFiUDP udpState;

// ─────────────── STATE ───────────────
bool flightActive = false;
int padId = -1;
int padX = 0, padY = 0, padZ = 0;

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
        delay(500);
        sendCommand(cmd);
        resp = waitResponse();
    }
    if (delayAfter > 0) delay(delayAfter);
    return (resp == "ok");
}

bool readState() {
    String latest = "";
    while (udpState.parsePacket()) {
        char buf[512];
        int len = udpState.read(buf, 511);
        if (len > 0) { buf[len] = 0; latest = String(buf); }
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

void runSequence() {
    Serial.println("\n==========================================================");
    Serial.println("     SHORTCUT — Slow go to 25, land");
    Serial.println("==========================================================\n");

    flightActive = true;

    // 1. Enable pad detection
    Serial.println("[1] Enabling SDK + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // 2. Takeoff
    Serial.println("\n[2] TAKEOFF...");
    if (!sendWithRetry("takeoff", 3000)) {
        Serial.println("    TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    Serial.println("    Takeoff complete\n");

    if (checkEmergency()) return;

    // 3. Wait for pad
    Serial.println("[3] Waiting for pad...");
    unsigned long start = millis();
    bool found = false;
    while (millis() - start < 10000) {
        if (checkEmergency()) return;
        readState();
        if (padId > 0) {
            Serial.printf("    PAD FOUND! ID=%d | X:%d Y:%d Z:%d\n", padId, padX, padY, padZ);
            found = true;
            break;
        }
        delay(100);
    }

    if (!found) {
        Serial.println("    No pad found — landing wherever.");
        sendCommand("land");
        waitResponse(10000);
        flightActive = false;
        return;
    }

    // 4. Go to 25cm slowly
    delay(500);
    for (int i = 0; i < 5; i++) { readState(); delay(100); }
    Serial.printf("\n[4] GO to %dcm at speed %d...\n", GO_HEIGHT, GO_SPEED);
    Serial.printf("    Before: X:%d Y:%d Z:%d\n", padX, padY, padZ);

    char goCmd[50];
    sprintf(goCmd, "go %d %d %d %d m%d", GO_X_OFFSET, GO_Y_OFFSET, GO_HEIGHT, GO_SPEED, padId);
    Serial.printf("    Sending: %s\n", goCmd);
    sendCommand(goCmd);
    String resp = waitResponse(30000);

    if (resp == "ok") {
        delay(500);
        for (int i = 0; i < 5; i++) { readState(); delay(100); }
        Serial.printf("    After go: X:%d Y:%d Z:%d (pad %d)\n", padX, padY, padZ, padId);
    } else {
        Serial.printf("    go failed: %s\n", resp.c_str());
    }

    if (checkEmergency() || !flightActive) return;

    // 5. Land
    if (USE_EMERGENCY_LAND) {
        Serial.println("\n[5] EMERGENCY LAND (motors cut — straight drop)...");
        sendCommand("emergency");
    } else {
        Serial.println("\n[5] NORMAL LAND...");
        sendCommand("land");
    }
    waitResponse(10000);
    flightActive = false;

    // Final position
    delay(500);
    for (int i = 0; i < 5; i++) { readState(); delay(100); }
    Serial.printf("    Landed at: X:%d Y:%d Z:%d\n", padX, padY, padZ);

    Serial.println("\n==========================================================");
    Serial.println("              DONE!");
    Serial.println("==========================================================\n");
}

// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n==========================================================");
    Serial.println("   SHORTCUT — Go to 25cm slowly, land");
    Serial.println("==========================================================\n");

    Serial.printf("Target: go %d %d %d %d m<id>\n", GO_X_OFFSET, GO_Y_OFFSET, GO_HEIGHT, GO_SPEED);
    Serial.printf("Land mode: %s\n\n", USE_EMERGENCY_LAND ? "EMERGENCY (motor cut)" : "NORMAL");

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

    udpCmd.begin(CMD_PORT);
    udpState.begin(STATE_PORT);
    delay(1000);

    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    sendCommand("battery?");
    String bat = waitResponse(3000);
    Serial.printf("Battery: %s%%\n\n", bat.c_str());

    Serial.println("Press S to start, E=emergency, L=land\n");
}

void loop() {
    if (Serial.available()) {
        char c = toupper(Serial.read());
        switch (c) {
            case 'S': runSequence(); break;
            case 'B': sendCommand("battery?"); waitResponse(); break;
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
