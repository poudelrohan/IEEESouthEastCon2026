// ══════════════════════════════════════════════════════════════
//  MISSION PAD CHECK — Test different pads for detection
//
//  Takes off, hovers, and continuously prints mission pad
//  detection status. Place different pads underneath to see
//  which ones the Tello can detect.
//
//  Commands:
//    S = Start (takeoff + hover + continuous pad reading)
//    B = Battery check
//    E = EMERGENCY (motors off!)
//    L = Normal land
// ══════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <WiFiUdp.h>

// ─────────────── WIFI CREDENTIALS ───────────────
const char* WIFI_NAME = "TELLO-BCU-DRONE";
const char* WIFI_PASSWORD = "southeast2026";

// ─────────────── SETTINGS ───────────────
int HOVER_HEIGHT = 80;  // cm — stay in good detection range (0.3-1.2m)

// ─────────────── NETWORK (don't change) ───────────────
const char* TELLO_IP = "192.168.10.1";
const int CMD_PORT = 8889;
const int STATE_PORT = 8890;

WiFiUDP udpCmd;
WiFiUDP udpState;

// ─────────────── STATE ───────────────
bool flightActive = false;
int padId = -1;
int padX = 0, padY = 0, padZ = 0;
int batteryLevel = 0;
unsigned long lastPrintTime = 0;

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
        delay(500);
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
//                    PAD CHECK LOOP
// ══════════════════════════════════════════════════════════════

void runPadCheck() {
    Serial.println("\n==========================================================");
    Serial.println("     MISSION PAD CHECK — Place pads underneath");
    Serial.println("==========================================================\n");

    flightActive = true;

    // Takeoff
    Serial.println("[1] TAKEOFF...");
    if (!sendWithRetry("takeoff", 3000)) {
        Serial.println("TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    Serial.println("    Takeoff complete\n");

    // Enable mission pad detection
    Serial.println("[2] Enabling mission pad detection...");
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);  // Downward only = 20Hz
    Serial.println("    Pad detection ON (downward, 20Hz)\n");

    Serial.println("==========================================================");
    Serial.println("  HOVERING — Place different pads underneath now!");
    Serial.println("  Reading pad status every 500ms...");
    Serial.println("  Press L to land, E for emergency");
    Serial.println("==========================================================\n");

    // Continuous pad reading loop
    while (flightActive) {

        // Check for serial commands
        if (Serial.available()) {
            char c = toupper(Serial.read());
            if (c == 'E') {
                Serial.println("\n!!! EMERGENCY !!!");
                sendCommand("emergency");
                flightActive = false;
                return;
            }
            if (c == 'L') {
                Serial.println("\nLanding...");
                sendCommand("land");
                waitResponse(10000);
                flightActive = false;
                return;
            }
        }

        readState();

        // Print every 500ms
        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();

            if (padId > 0) {
                Serial.printf("DETECTED  | Pad ID: %d | X:%4d Y:%4d Z:%3d | Bat: %d%%\n",
                              padId, padX, padY, padZ, batteryLevel);
            } else {
                Serial.printf("NO PAD    | mid: %d | Bat: %d%%\n",
                              padId, batteryLevel);
            }
        }

        delay(50);
    }
}

// ══════════════════════════════════════════════════════════════
//                    SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n==========================================================");
    Serial.println("   MISSION PAD CHECK");
    Serial.println("   Test different pads for Tello detection");
    Serial.println("==========================================================\n");

    // Connect WiFi
    Serial.printf("Connecting to %s", WIFI_NAME);
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

    // Enter SDK mode
    sendWithRetry("command", 500);

    // Check battery
    sendCommand("battery?");
    String bat = waitResponse(3000);
    Serial.printf("Battery: %s%%\n\n", bat.c_str());

    Serial.println("==========================================================");
    Serial.println("  COMMANDS:");
    Serial.println("  ---------");
    Serial.println("  s = START (takeoff + hover + pad reading)");
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
                runPadCheck();
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
