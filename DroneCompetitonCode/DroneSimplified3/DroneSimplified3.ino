#include <WiFi.h>
#include <WiFiUdp.h>

// ══════════════════════════════════════════════════════════════
//                    EASY TO CHANGE VARIABLES
// ══════════════════════════════════════════════════════════════

// ─────────────── WIFI CREDENTIALS ───────────────
const char* WIFI_NAME = "TELLO-BCU-DRONE"; // CHANGE THIS
const char* WIFI_PASSWORD = "southeast2026";

// ─────────────── FLIGHT PATH DISTANCES (cm) ───────────────
int DIST_FORWARD_1 = 20;      // First move forward
int DIST_RIGHT     = 20;      // Then move right
int TURN_ANGLE     = 180;     // Turn around (180 degrees)
int DIST_FORWARD_2 = 20;      // Move forward after turning

// ─────────────── TIMING (ms) ───────────────
int SLEEP_TIME = 10000;               // 10 seconds sleep
int STABILIZE_AFTER_TAKEOFF = 4000;   // Wait to stabilize
int PAUSE_AFTER_MOVE = 2000;          // Wait between moves

// ─────────────── PAD LANDING SETTINGS ───────────────
int SEARCH_TIMEOUT = 15000;   // How long to look for pad (ms)
int LAND_HEIGHT = 45;         // Height to cut motors (cm)
int TARGET_ZONE = 15;         // Accuracy required (cm)
int NUDGE_SPEED = 15;         // Speed for corrections

// ─────────────── PAD OFFSETS (cm) ───────────────
// Adjust these if it lands slightly off-center
int PAD_X_OFFSET = 0;        
int PAD_Y_OFFSET = 0;

// ══════════════════════════════════════════════════════════════
//                    NETWORK & STATE
// ══════════════════════════════════════════════════════════════

const char* TELLO_IP = "192.168.10.1";
const int CMD_PORT = 8889;
const int STATE_PORT = 8890;

WiFiUDP udpCmd;
WiFiUDP udpState;

bool flightActive = false;
int padId = -1;
int padX = 0, padY = 0, padZ = 0;
int centeredCount = 0;

// ══════════════════════════════════════════════════════════════
//                    COMMUNICATION FUNCTIONS
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
    while (millis() - start < timeoutMs) {
        int size = udpCmd.parsePacket();
        if (size) {
            char buf[256];
            int len = udpCmd.read(buf, 255);
            buf[len] = 0;
            return String(buf);
        }
        delay(10);
    }
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

void sendRC(int roll, int pitch, int throttle, int yaw) {
    char cmd[50];
    sprintf(cmd, "rc %d %d %d %d", roll, pitch, throttle, yaw);
    sendCommand(cmd);
}

void rotateDrone(int degrees) {
    char cmd[30];
    if (degrees > 0) {
        Serial.printf("   ROTATING CW %d...\n", degrees);
        sprintf(cmd, "cw %d", degrees);
    } else {
        Serial.printf("   ROTATING CCW %d...\n", abs(degrees));
        sprintf(cmd, "ccw %d", abs(degrees));
    }
    sendWithRetry(cmd, 2500); // Give it time to spin
}

// ══════════════════════════════════════════════════════════════
//                    STATE READING
// ══════════════════════════════════════════════════════════════

void readState() {
    String latest = "";
    while (udpState.parsePacket()) {
        char buf[512];
        int len = udpState.read(buf, 511);
        if (len > 0) { buf[len] = 0; latest = String(buf); }
    }
    
    if (latest.length() > 0) {
        int idx = latest.indexOf("mid:");
        if (idx != -1) padId = latest.substring(idx + 4, latest.indexOf(";", idx)).toInt();
        
        idx = latest.indexOf(";x:");
        if (idx != -1) padX = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();
        
        idx = latest.indexOf(";y:");
        if (idx != -1) padY = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();
        
        idx = latest.indexOf(";z:");
        if (idx != -1) padZ = latest.substring(idx + 3, latest.indexOf(";", idx + 3)).toInt();
    }
}

// ══════════════════════════════════════════════════════════════
//                    LANDING LOGIC
// ══════════════════════════════════════════════════════════════

bool hoverAndSearch() {
    Serial.println("[SEARCH] Looking for Pad...");
    unsigned long start = millis();
    while (millis() - start < SEARCH_TIMEOUT) {
        readState();
        if (padId > 0) {
            Serial.printf("   PAD FOUND! ID: %d\n", padId);
            return true;
        }
        delay(100);
    }
    Serial.println("   TIMEOUT: No pad found.");
    return false;
}

bool centerAndLand() {
    Serial.println("[LANDING] Centering...");
    unsigned long start = millis();
    
    while (millis() - start < 20000) { // 20s max to land
        readState();
        
        if (padId <= 0) {
            sendRC(0,0,0,0); // Hover if lost
            continue;
        }

        // Logic
        int errX = padX - PAD_X_OFFSET;
        int errY = padY - PAD_Y_OFFSET;
        
        int p = 0, r = 0, t = 0;
        
        // Simple P-Controller
        if (errX > TARGET_ZONE) p = -NUDGE_SPEED;
        else if (errX < -TARGET_ZONE) p = NUDGE_SPEED;
        
        if (errY > TARGET_ZONE) r = -NUDGE_SPEED;
        else if (errY < -TARGET_ZONE) r = NUDGE_SPEED;

        // Invert Roll for Tello
        r = -r;

        // Descent Logic
        if (abs(errX) < TARGET_ZONE && abs(errY) < TARGET_ZONE) {
            if (padZ > LAND_HEIGHT) {
                t = -20; // Slow descent
                Serial.printf("   Dropping... Z: %d\n", padZ);
            } else {
                Serial.println("   TOUCHDOWN!");
                sendCommand("land");
                return true;
            }
        }
        
        sendRC(r, p, t, 0);
        delay(50);
    }
    sendCommand("land");
    return false;
}

// ══════════════════════════════════════════════════════════════
//                    MAIN SEQUENCE
// ══════════════════════════════════════════════════════════════

void runSimpleSequence() {
    Serial.println("\n=== STARTING MISSION ===");
    flightActive = true;
    char cmd[30];

    // --- PHASE 1: THE FLIGHT PATH ---
    
    Serial.println("1. Takeoff");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) return;

    Serial.printf("2. Forward %d cm\n", DIST_FORWARD_1);
    sprintf(cmd, "forward %d", DIST_FORWARD_1);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    Serial.printf("3. Right %d cm\n", DIST_RIGHT);
    sprintf(cmd, "right %d", DIST_RIGHT);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    Serial.printf("4. Turning %d degrees\n", TURN_ANGLE);
    rotateDrone(TURN_ANGLE);

    Serial.printf("5. Forward %d cm (New Direction)\n", DIST_FORWARD_2);
    sprintf(cmd, "forward %d", DIST_FORWARD_2);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    Serial.printf("3. Right %d cm\n", DIST_RIGHT);
    sprintf(cmd, "right %d", DIST_RIGHT);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    Serial.println("6. Landing (Nap Time)");
    sendCommand("land");
    
    // --- PHASE 2: THE NAP ---
    
    Serial.printf("7. Sleeping for %d seconds...\n", SLEEP_TIME/1000);
    for(int i=SLEEP_TIME/1000; i>0; i--) {
        Serial.printf("   %d...\n", i);
        delay(1000);
    }

    // --- PHASE 3: THE SEARCH ---
    
    Serial.println("8. Waking Up - Enabling Cameras");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500); // Look down

    Serial.println("9. Second Takeoff");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) return;

    

    Serial.println("10. Searching for Pad");
    if (hoverAndSearch()) {
        centerAndLand();
    } else {
        Serial.println("   No Pad seen, landing anyway.");
        sendCommand("land");
    }
    
    flightActive = false;
    Serial.println("=== MISSION COMPLETE ===");
}

// ══════════════════════════════════════════════════════════════
//                    SETUP & LOOP
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\nConnecting to Tello...");
    WiFi.begin(WIFI_NAME, WIFI_PASSWORD); // Use "" for password if open
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected!");
    
    udpCmd.begin(CMD_PORT);
    udpState.begin(STATE_PORT);
    
    sendWithRetry("command"); // Enter SDK Mode
    Serial.println("\nREADY! Press 'S' to Start, 'L' to Land, 'E' for Emergency.");
}

void loop() {
    if (Serial.available()) {
        char c = toupper(Serial.read());
        if (c == 'S') runSimpleSequence();
        if (c == 'L') sendCommand("land");
        if (c == 'E') sendCommand("emergency");
    }
    delay(50);
}