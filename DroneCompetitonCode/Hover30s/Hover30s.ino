#include <WiFi.h>
#include <WiFiUdp.h>

// ══════════════════════════════════════════════════════════════
//                    SETTINGS
// ══════════════════════════════════════════════════════════════

const char* WIFI_NAME = "TELLO-BCU-DRONE";
const char* WIFI_PASSWORD = "southeast2026";

int HOVER_TIME = 10000;  // 30 seconds

// ══════════════════════════════════════════════════════════════
//                    NETWORK
// ══════════════════════════════════════════════════════════════

const char* TELLO_IP = "192.168.10.1";
const int CMD_PORT = 8889;

WiFiUDP udp;

// ══════════════════════════════════════════════════════════════
//                    FUNCTIONS
// ══════════════════════════════════════════════════════════════

void sendCommand(const char* cmd) {
    Serial.print(">> ");
    Serial.println(cmd);
    udp.beginPacket(TELLO_IP, CMD_PORT);
    udp.print(cmd);
    udp.endPacket();
}

String waitResponse(int timeoutMs = 10000) {
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        int size = udp.parsePacket();
        if (size) {
            char buf[256];
            int len = udp.read(buf, 255);
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
//                    HOVER TEST
// ══════════════════════════════════════════════════════════════

void runHoverTest() {
    Serial.println("\n══════════════════════════════════════════════════════════");
    Serial.println("              HOVER TEST");
    Serial.println("══════════════════════════════════════════════════════════\n");
    
    Serial.printf("Will hover for %d seconds\n\n", HOVER_TIME / 1000);
    
    // Takeoff
    Serial.println("[1] TAKEOFF...");
    if (!sendWithRetry("takeoff", 3000)) {
        Serial.println("TAKEOFF FAILED!");
        return;
    }
    Serial.println("    Takeoff complete!\n");
    
    // Hover with countdown
    Serial.println("[2] HOVERING...");
    int totalSec = HOVER_TIME / 1000;
    
    for (int i = totalSec; i > 0; i--) {
        
        // Check for emergency
        if (Serial.available()) {
            char c = toupper(Serial.read());
            if (c == 'E') {
                Serial.println("\n!!! EMERGENCY !!!");
                sendCommand("emergency");
                return;
            }
            if (c == 'L') {
                Serial.println("\nManual land...");
                sendCommand("land");
                return;
            }
        }
        
        Serial.printf("    Hovering... %d seconds remaining\n", i);
        delay(1000);
    }
    
    Serial.println("    Hover complete!\n");
    
    // Land
    Serial.println("[3] LANDING...");
    sendCommand("land");
    
    Serial.println("\n══════════════════════════════════════════════════════════");
    Serial.println("              HOVER TEST COMPLETE!");
    Serial.println("══════════════════════════════════════════════════════════\n");
}

// ══════════════════════════════════════════════════════════════
//                    SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n══════════════════════════════════════════════════════════");
    Serial.println("              TELLO HOVER TEST");
    Serial.println("══════════════════════════════════════════════════════════\n");
    
    Serial.printf("Hover time: %d seconds\n\n", HOVER_TIME / 1000);
    
    // Connect WiFi
    Serial.print("Connecting to Tello");
    WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi FAILED!");
        while (1) delay(1000);
    }
    Serial.println(" Connected!\n");
    
    // Setup UDP
    udp.begin(CMD_PORT);
    delay(1000);
    
    // Enter SDK mode
    sendWithRetry("command", 500);
    
    // Battery check
    sendCommand("battery?");
    waitResponse();
    
    Serial.println("\nCOMMANDS:");
    Serial.println("  h = Start hover test");
    Serial.println("  e = EMERGENCY");
    Serial.println("  l = Land");
    runHoverTest();
    Serial.println();
}

// ══════════════════════════════════════════════════════════════
//                    LOOP
// ══════════════════════════════════════════════════════════════

void loop() {
    if (Serial.available()) {
        char c = toupper(Serial.read());
        
        switch (c) {
            case 'H':
                runHoverTest();
                break;
                
            case 'E':
                Serial.println("\n!!! EMERGENCY !!!");
                sendCommand("emergency");
                break;
                
            case 'L':
                Serial.println("\nLanding...");
                sendCommand("land");
                break;
        }
    }
    delay(10);
}
