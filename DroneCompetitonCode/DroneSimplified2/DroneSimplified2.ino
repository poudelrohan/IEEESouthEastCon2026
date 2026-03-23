#include <WiFi.h>
#include <WiFiUdp.h>

// ══════════════════════════════════════════════════════════════
//                    EASY TO CHANGE VARIABLES
// ══════════════════════════════════════════════════════════════

// ─────────────── WIFI CREDENTIALS ───────────────
const char* WIFI_NAME = "TELLO-BCU-DRONE";
const char* WIFI_PASSWORD = "southeast2026";

// ─────────────── SAFETY MOVE AFTER TAKEOFF (cm) ───────────────
int SAFETY_FORWARD = 20;      // Move away from back net (min 20)
int SAFETY_RIGHT = 20;        // Move away from left net (min 20)

// ─────────────── DESCENT (cm) ───────────────
int DESCEND_AMOUNT = 40;      // How much to go down after takeoff

// ─────────────── MOVEMENT PATTERN (cm) ───────────────
int MOVE_RIGHT = 40;          // First move right
int MOVE_LEFT = 30;           // Then move left (more to compensate drift)

// ─────────────── SEARCH SETTINGS ───────────────
int SEARCH_BACK = 80;         // cm to move back before searching
int SEARCH_TIMEOUT = 15000;   // Max time to look for pad (ms)

// ─────────────── TIMING (ms) ───────────────
int SLEEP_TIME = 10000;               // Sleep after first landing (10 sec)
int STABILIZE_AFTER_TAKEOFF = 3000;   // Wait after takeoff
int PAUSE_AFTER_MOVE = 1500;          // Pause after each move
int COMMAND_RETRY_DELAY = 500;        // Delay before retry

// ─────────────── PAD LANDING ───────────────
int PAD_X_OFFSET = 0;        // cm forward from rocket icon
int PAD_Y_OFFSET = 0;         // cm left/right adjustment
int TARGET_ZONE = 10;         // Centered if within this (cm)
int CENTERED_COUNT_NEEDED = 10; // Readings needed to confirm
int LAND_HEIGHT = 40;         // Below this = land

// ─────────────── SPEEDS ───────────────
int NUDGE_SPEED = 15;         // RC speed for centering

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
    while (millis() - start < timeoutMs) {
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
//                    RC CONTROL (only for centering)
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

// ══════════════════════════════════════════════════════════════
//                    LANDING FUNCTIONS
// ══════════════════════════════════════════════════════════════

void performLand() {
    delay(200);
    
    if (USE_EMERGENCY_LAND) {
        Serial.println("\n*** EMERGENCY LAND (drop) ***");
        sendCommand("emergency");
    } else {
        Serial.println("\n*** NORMAL LAND ***");
        sendCommand("land");
    }
    flightActive = false;
}

// ══════════════════════════════════════════════════════════════
//                    SLEEP WITH COUNTDOWN
// ══════════════════════════════════════════════════════════════

void sleepWithCountdown(int totalMs) {
    Serial.println("\n───────── SLEEPING ─────────");
    
    int totalSec = totalMs / 1000;
    
    for (int i = totalSec; i > 0; i--) {
        Serial.printf("Sleeping... %d seconds remaining\n", i);
        delay(1000);
    }
    
    Serial.println("Sleep complete!\n");
}

// ══════════════════════════════════════════════════════════════
//                    HOVER AND SEARCH FOR PAD
// ══════════════════════════════════════════════════════════════

bool hoverAndSearchForPad() {
    Serial.println("\n───────── SEARCHING FOR PAD (hovering) ─────────");
    Serial.printf("Timeout: %d seconds\n", SEARCH_TIMEOUT / 1000);
    
    unsigned long searchStart = millis();
    int lastSecondPrinted = -1;
    
    while (millis() - searchStart < SEARCH_TIMEOUT) {
        
        // Emergency check
        if (Serial.available()) {
            char c = toupper(Serial.read());
            if (c == 'E') {
                Serial.println("\n!!! EMERGENCY !!!");
                sendCommand("emergency");
                flightActive = false;
                return false;
            }
            if (c == 'L') {
                Serial.println("\nManual land...");
                sendCommand("land");
                flightActive = false;
                return false;
            }
        }
        
        readState();
        
        // Print countdown every second
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
//                    CENTER ON PAD
// ══════════════════════════════════════════════════════════════

bool centerOnPad() {
    Serial.println("\n───────── CENTERING ON PAD ─────────");
    Serial.printf("Target: X=%d, Y=%d (offset from rocket)\n", PAD_X_OFFSET, PAD_Y_OFFSET);
    
    centeredCount = 0;
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 20000;
    
    while (millis() - startTime < TIMEOUT && flightActive) {
        
        // Emergency check
        if (Serial.available()) {
            char c = toupper(Serial.read());
            if (c == 'E') {
                Serial.println("\n!!! EMERGENCY !!!");
                sendCommand("emergency");
                flightActive = false;
                return false;
            }
        }
        
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
        
        // Print status
        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            Serial.printf("PAD:%d | X:%4d Y:%4d Z:%3d | Error X:%4d Y:%4d | ", 
                          padId, padX, padY, padZ, errorX, errorY);
            if (centered) {
                Serial.printf("CENTERED [%d/%d]\n", centeredCount, CENTERED_COUNT_NEEDED);
            } else {
                Serial.println("Adjusting...");
            }
        }
        
        if (centeredCount >= CENTERED_COUNT_NEEDED) {
            Serial.println("CENTERING COMPLETE!");
            stopMovement();
            return true;
        }
        
        int roll = 0, pitch = 0;
        
        if (errorX > TARGET_ZONE) pitch = -NUDGE_SPEED;
        else if (errorX < -TARGET_ZONE) pitch = NUDGE_SPEED;
        
        if (errorY > TARGET_ZONE) roll = -NUDGE_SPEED;
        else if (errorY < -TARGET_ZONE) roll = NUDGE_SPEED;
        
        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL) roll = -roll;
        
        sendRC(roll, pitch, 0, 0);
        delay(100);
    }
    
    Serial.println("CENTERING TIMEOUT!");
    return false;
}

// ══════════════════════════════════════════════════════════════
//                    DESCEND AND LAND ON PAD
// ══════════════════════════════════════════════════════════════

bool descendAndLandOnPad() {
    Serial.println("\n───────── DESCENDING TO PAD ─────────");
    Serial.printf("Maintaining offset: X=%d, Y=%d\n", PAD_X_OFFSET, PAD_Y_OFFSET);
    Serial.printf("Will land when Z < %dcm\n", LAND_HEIGHT);
    
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 20000;
    
    while (millis() - startTime < TIMEOUT && flightActive) {
        
        readState();
        
        int errorX = padX - PAD_X_OFFSET;
        int errorY = padY - PAD_Y_OFFSET;
        
        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            if (padId > 0) {
                Serial.printf("DESCENDING | Z:%3d | Error X:%4d Y:%4d\n", padZ, errorX, errorY);
            } else {
                Serial.println("DESCENDING | Pad lost - continuing down...");
            }
        }
        
        if (padId > 0 && padZ < LAND_HEIGHT) {
            Serial.printf("\nZ=%dcm - LOW ENOUGH!\n", padZ);
            break;
        }
        
        if (padId <= 0) {
            Serial.println("\nPad lost during descent - landing now!");
            break;
        }
        
        int roll = 0, pitch = 0;
        
        if (errorX > TARGET_ZONE) pitch = -NUDGE_SPEED;
        else if (errorX < -TARGET_ZONE) pitch = NUDGE_SPEED;
        
        if (errorY > TARGET_ZONE) roll = -NUDGE_SPEED;
        else if (errorY < -TARGET_ZONE) roll = NUDGE_SPEED;
        
        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL) roll = -roll;
        
        sendRC(roll, pitch, -15, 0);
        delay(100);
    }
    
    performLand();
    return true;
}

// ══════════════════════════════════════════════════════════════
//                    MAIN SIMPLE SEQUENCE
// ══════════════════════════════════════════════════════════════

void runSimpleSequence() {
    Serial.println("\n══════════════════════════════════════════════════════════");
    Serial.println("              SIMPLE COMPETITION SEQUENCE");
    Serial.println("══════════════════════════════════════════════════════════\n");
    
    Serial.println("SETTINGS:");
    Serial.printf("  Safety: Forward %dcm, Right %dcm\n", SAFETY_FORWARD, SAFETY_RIGHT);
    Serial.printf("  Descend: %dcm\n", DESCEND_AMOUNT);
    Serial.printf("  Pattern: Right %dcm, Left %dcm\n", MOVE_RIGHT, MOVE_LEFT);
    Serial.printf("  Sleep: %d seconds\n", SLEEP_TIME / 1000);
    Serial.printf("  Search back: %dcm\n", SEARCH_BACK);
    Serial.printf("  Search timeout: %d seconds\n", SEARCH_TIMEOUT / 1000);
    Serial.printf("  Land type: %s\n", USE_EMERGENCY_LAND ? "EMERGENCY" : "Normal");
    Serial.println();
    
    flightActive = true;
    char cmd[30];
    
    // ═══════════════════════════════════════════════════════
    // PHASE 1: First Flight (no pad detection)
    // ═══════════════════════════════════════════════════════
    
    Serial.println("════════ PHASE 1: FIRST FLIGHT ════════\n");
    
    // Step 1: Takeoff
    Serial.println("[1] TAKEOFF...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    Serial.println("    Takeoff complete (~100cm)\n");
    
    // Step 2: Safety move (away from nets)
    Serial.printf("[2] SAFETY MOVE: Forward %dcm...\n", SAFETY_FORWARD);
    sprintf(cmd, "forward %d", SAFETY_FORWARD);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    
    Serial.printf("    SAFETY MOVE: Right %dcm...\n", SAFETY_RIGHT);
    sprintf(cmd, "right %d", SAFETY_RIGHT);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    Serial.println("    Away from nets!\n");
    
    // Step 3: Descend (trust sensors)
    //Serial.printf("[3] DESCENDING %dcm (trusting sensors)...\n", DESCEND_AMOUNT);
    //sprintf(cmd, "down %d", DESCEND_AMOUNT);
    //sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    //Serial.println("    Now at ~40cm\n");
    
    // Step 4: Right
    Serial.printf("[4] MOVE RIGHT %dcm...\n", MOVE_RIGHT);
    sprintf(cmd, "right %d", MOVE_RIGHT);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    
    // Step 5: Left
    Serial.printf("[5] MOVE LEFT %dcm...\n", MOVE_LEFT);
    sprintf(cmd, "left %d", MOVE_LEFT);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);

    Serial.printf("[5.5] MOVE BACK %dcm...\n", SEARCH_BACK);
    sprintf(cmd, "back %d", SEARCH_BACK);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    
    // Step 6: Land (no pad detection)
    Serial.println("[6] LANDING (wherever we are)...");
    performLand();
    
    // ═══════════════════════════════════════════════════════
    // PHASE 2: Sleep
    // ═══════════════════════════════════════════════════════
    
    Serial.println("\n════════ PHASE 2: SLEEPING ════════\n");
    sleepWithCountdown(SLEEP_TIME);
    
    // ═══════════════════════════════════════════════════════
    // PHASE 3: Second Flight (with pad detection)
    // ═══════════════════════════════════════════════════════
    
    Serial.println("════════ PHASE 3: SECOND FLIGHT ════════\n");
    
    flightActive = true;
    
    // Step 7: Re-enter SDK mode and enable pad detection
    Serial.println("[7] Re-entering SDK mode and enabling pad detection...");
    sendWithRetry("command", 500);  // IMPORTANT: Reset to command mode!
    sendWithRetry("mon", 300);
    sendWithRetry("mdirection 0", 300);
    
    // Step 8: Takeoff again
    Serial.println("\n[8] TAKEOFF (second time)...");
    if (!sendWithRetry("takeoff", STABILIZE_AFTER_TAKEOFF)) {
        Serial.println("SECOND TAKEOFF FAILED!");
        flightActive = false;
        return;
    }
    Serial.println("    Takeoff complete (~100cm)\n");
    
    // Step 9: Move back (normal command)
    Serial.printf("[9] MOVE BACK %dcm...\n", SEARCH_BACK);
    sprintf(cmd, "back %d", SEARCH_BACK);
    sendWithRetry(cmd, PAUSE_AFTER_MOVE);
    
    // Step 10: Hover and search for pad
    Serial.println("[10] SEARCHING FOR PAD (hovering)...");
    bool padFound = hoverAndSearchForPad();
    
    // Step 11: Land
    if (padFound) {
        Serial.println("\n[11] PAD FOUND - Centering and landing...");
        if (centerOnPad()) {
            descendAndLandOnPad();
        } else {
            Serial.println("Centering failed - landing wherever...");
            performLand();
        }
    } else {
        Serial.println("\n[11] PAD NOT FOUND - Landing wherever...");
        performLand();
    }
    
    // ═══════════════════════════════════════════════════════
    // DONE
    // ═══════════════════════════════════════════════════════
    
    Serial.println("\n══════════════════════════════════════════════════════════");
    Serial.println("              SEQUENCE COMPLETE!");
    Serial.println("══════════════════════════════════════════════════════════\n");
}

// ══════════════════════════════════════════════════════════════
//                    SETUP
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n══════════════════════════════════════════════════════════");
    Serial.println("         TELLO SIMPLE COMPETITION SEQUENCE");
    Serial.println("══════════════════════════════════════════════════════════\n");
    
    // Print settings
    Serial.println("WIFI:");
    Serial.printf("  SSID:     %s\n", WIFI_NAME);
    Serial.printf("  Password: %s\n", WIFI_PASSWORD);
    Serial.println();
    
    Serial.println("FLIGHT SETTINGS:");
    Serial.printf("  Safety forward:  %dcm\n", SAFETY_FORWARD);
    Serial.printf("  Safety right:    %dcm\n", SAFETY_RIGHT);
    Serial.printf("  Descend amount:  %dcm\n", DESCEND_AMOUNT);
    Serial.printf("  Move right:      %dcm\n", MOVE_RIGHT);
    Serial.printf("  Move left:       %dcm\n", MOVE_LEFT);
    Serial.printf("  Sleep time:      %d seconds\n", SLEEP_TIME / 1000);
    Serial.printf("  Search back:     %dcm\n", SEARCH_BACK);
    Serial.printf("  Search timeout:  %d seconds\n", SEARCH_TIMEOUT / 1000);
    Serial.printf("  Land type:       %s\n", USE_EMERGENCY_LAND ? "EMERGENCY" : "Normal");
    Serial.println();
    
    Serial.println("PAD SETTINGS:");
    Serial.printf("  X offset:        %dcm\n", PAD_X_OFFSET);
    Serial.printf("  Y offset:        %dcm\n", PAD_Y_OFFSET);
    Serial.printf("  Target zone:     +/-%dcm\n", TARGET_ZONE);
    Serial.printf("  Land height:     %dcm\n", LAND_HEIGHT);
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
    
    // Check battery
    sendCommand("battery?");
    String bat = waitResponse(3000);
    Serial.printf("Battery: %s%%\n\n", bat.c_str());
    
    // Print commands
    Serial.println("══════════════════════════════════════════════════════════");
    Serial.println("  COMMANDS:");
    Serial.println("  ─────────");
    Serial.println("  s = START simple sequence");
    Serial.println("  b = Battery check");
    Serial.println("  e = EMERGENCY (motors off!)");
    Serial.println("  l = Normal land");
    Serial.println("══════════════════════════════════════════════════════════\n");
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