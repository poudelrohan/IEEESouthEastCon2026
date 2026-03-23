// ══════════════════════════════════════════════════════════════
//  TELLO v8 — LAND ON 10" ROBOT CIRCLE
//
//  Uses v4's PROVEN centering + descent (simple continuous RC)
//  with v8's simple flight path.
//
//  Flight path:
//    1. Takeoff (~100cm)
//    2. Enable mission pad detection (mon + mdirection 0)
//    3. Forward 40cm — move away 15"+
//    4. CW 180° — turn around
//    5. Forward 40cm — walk back to start
//    6. Search for pad → Center → Descend while centering → Land
//
//  Centering (from v4):
//    - Continuous RC at fixed NUDGE_SPEED (no proportional, no pulse)
//    - TARGET_ZONE = 15cm (forgiving)
//    - 100ms loop delay
//    - Descend while correcting: rc roll, pitch, -15, 0
//    - Land when Z < LAND_HEIGHT (45cm)
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

// ─────────────── SEARCH SETTINGS ───────────────
int SEARCH_TIMEOUT = 15000;     // Max time to look for pad (ms)

// ─────────────── TIMING (ms) ───────────────
int STABILIZE_AFTER_TAKEOFF = 3000;   // Wait after takeoff
int PAUSE_AFTER_MOVE = 1500;          // Pause after each move
int COMMAND_RETRY_DELAY = 500;        // Delay before retry

// ─────────────── PAD LANDING (v4 proven values) ───────────────
int PAD_X_OFFSET = -4;        // cm forward from rocket icon (-ve = back)
int PAD_Y_OFFSET = 4;         // cm left/right adjustment (-ve = right)
int TARGET_ZONE = 15;         // Centered if within this (cm) — v4 value
int CENTERED_COUNT_NEEDED = 25; // Readings needed to confirm — v4 value
int LAND_HEIGHT = 45;         // Below this = land (cm) — v4 value

// ─────────────── SPEEDS ───────────────
int NUDGE_SPEED = 15;         // Fixed RC speed for centering — v4 value

// ─────────────── LANDING STYLE ───────────────
bool USE_EMERGENCY_LAND = false;  // true = drop, false = gentle

// ─────────────── ORIENTATION FIX ───────────────
// v4 after ccw 90: INVERT_PITCH=false, INVERT_ROLL=true (PROVEN)
// v8 after cw 180: heading is different — may need testing
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
//                    RC CONTROL (v4 style — simple fixed speed)
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

// Quick check: read state for up to waitMs, return true if pad detected
bool quickPadCheck(int waitMs) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)waitMs) {
        if (checkEmergency()) return false;
        readState();
        if (padId > 0) {
            Serial.printf("PAD FOUND! ID=%d | X:%d Y:%d Z:%d\n", padId, padX, padY, padZ);
            return true;
        }
        delay(100);
    }
    return false;
}

bool hoverAndSearchForPad() {
    Serial.println("\n----------- SEARCHING FOR PAD -----------");

    // STEP 1: Hover for 3 seconds — maybe we're already above it
    Serial.println("Step 1: Hover check (3s)...");
    if (quickPadCheck(3000)) return true;

    // STEP 2: Not found — drone likely drifted. Do a slow search pattern.
    Serial.println("Step 2: Pad not visible — starting search pattern...");

    const char* searchMoves[] = {
        "forward 20", "back 20",    // check 20cm ahead, return
        "back 20",    "forward 20", // check 20cm behind, return
        "right 20",   "left 20",    // check 20cm right, return
        "left 20",    "right 20"    // check 20cm left, return
    };
    int numMoves = 8;

    for (int i = 0; i < numMoves; i += 2) {
        if (checkEmergency()) return false;

        Serial.printf("   Search move: %s\n", searchMoves[i]);
        sendWithRetry(searchMoves[i], 500);

        // Check for pad at this position
        if (quickPadCheck(2000)) return true;

        // Move back to center
        Serial.printf("   Return: %s\n", searchMoves[i + 1]);
        sendWithRetry(searchMoves[i + 1], 500);

        // Check after returning
        if (quickPadCheck(1000)) return true;
    }

    // STEP 3: Still not found — try going up for wider detection cone
    Serial.println("Step 3: Going up 30cm for wider detection cone...");
    sendWithRetry("up 30", 1000);
    if (quickPadCheck(3000)) return true;

    Serial.println("SEARCH FAILED - Pad not found after full search!");
    return false;
}

// ══════════════════════════════════════════════════════════════
//                    CENTER ON PAD (v4 style — continuous RC)
// ══════════════════════════════════════════════════════════════

bool centerOnPad() {
    Serial.println("\n----------- CENTERING ON PAD (v4 continuous RC) -----------");
    Serial.printf("Target: X=%d, Y=%d | Zone: +/-%dcm | Need %d confirms\n",
                  PAD_X_OFFSET, PAD_Y_OFFSET, TARGET_ZONE, CENTERED_COUNT_NEEDED);
    Serial.printf("Nudge speed: %d | Invert pitch=%s roll=%s\n",
                  NUDGE_SPEED, INVERT_PITCH ? "true" : "false", INVERT_ROLL ? "true" : "false");

    centeredCount = 0;
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 20000;

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
                Serial.println("Adjusting...");
            }
        }

        if (centeredCount >= CENTERED_COUNT_NEEDED) {
            Serial.println("CENTERING COMPLETE!");
            stopMovement();
            return true;
        }

        // Calculate correction — v4 style: fixed speed, bang-bang
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
    stopMovement();
    return false;
}

// ══════════════════════════════════════════════════════════════
//                    DESCEND AND LAND ON PAD (v4 style)
// ══════════════════════════════════════════════════════════════

bool descendAndLandOnPad() {
    Serial.println("\n----------- DESCENDING TO PAD (v4 style) -----------");
    Serial.printf("Correcting XY while descending | rc roll, pitch, -15, 0\n");
    Serial.printf("Will land when Z < %dcm\n", LAND_HEIGHT);

    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 20000;

    while (millis() - startTime < TIMEOUT && flightActive) {

        if (checkEmergency()) return false;

        readState();

        int errorX = padX - PAD_X_OFFSET;
        int errorY = padY - PAD_Y_OFFSET;

        // Print status every 500ms
        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            if (padId > 0) {
                Serial.printf("DESCENDING | Z:%3d | Err X:%4d Y:%4d\n", padZ, errorX, errorY);
            } else {
                Serial.println("DESCENDING | Pad lost - continuing down...");
            }
        }

        // Check if low enough to land
        if (padId > 0 && padZ < LAND_HEIGHT) {
            Serial.printf("\nZ=%dcm - LOW ENOUGH!\n", padZ);
            break;
        }

        // If pad lost during descent, land immediately
        if (padId <= 0) {
            Serial.println("\nPad lost during descent - landing now!");
            break;
        }

        // v4 style: correct XY while descending simultaneously
        int roll = 0, pitch = 0;

        if (errorX > TARGET_ZONE) pitch = -NUDGE_SPEED;
        else if (errorX < -TARGET_ZONE) pitch = NUDGE_SPEED;

        if (errorY > TARGET_ZONE) roll = -NUDGE_SPEED;
        else if (errorY < -TARGET_ZONE) roll = NUDGE_SPEED;

        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL) roll = -roll;

        // Descend at -15 throttle while correcting XY
        sendRC(roll, pitch, -15, 0);
        delay(100);
    }

    performLand();
    return true;
}

// ══════════════════════════════════════════════════════════════
//                    MAIN SEQUENCE
// ══════════════════════════════════════════════════════════════

void runSimpleSequence() {
    Serial.println("\n==========================================================");
    Serial.println("     v8 — LAND ON 10\" ROBOT CIRCLE (v4 centering)");
    Serial.println("==========================================================\n");

    Serial.println("FLIGHT PLAN:");
    Serial.printf("  [1] Takeoff\n");
    Serial.printf("  [2] Enable pad detection (mon + mdirection 0)\n");
    Serial.printf("  [3] Forward %dcm (move away 15\"+)\n", MOVE_AWAY_DISTANCE);
    Serial.printf("  [4] CW %d deg (turn around)\n", TURN_DEGREES);
    Serial.printf("  [5] Forward %dcm (return to pad)\n", RETURN_DISTANCE);
    Serial.printf("  [6] Search for pad\n");
    Serial.printf("  [7] Center on pad (continuous RC, speed %d)\n", NUDGE_SPEED);
    Serial.printf("  [8] Descend + correct XY → land when Z < %dcm\n", LAND_HEIGHT);
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
    // PHASE 3: LAND ON PAD (v4 approach)
    // ═══════════════════════════════════════════════════════

    Serial.println("\n======== PHASE 3: LAND ON PAD ========\n");

    // [6a] Re-enter SDK mode + re-enable pad detection
    Serial.println("[6a] Re-enabling SDK mode + pad detection...");
    sendWithRetry("command", 500);
    sendWithRetry("mon", 500);
    sendWithRetry("mdirection 0", 500);

    // [6b] Search for pad
    Serial.println("[6b] SEARCHING FOR PAD...");
    bool padFound = hoverAndSearchForPad();

    if (padFound) {
        // [7] Center on pad — v4 style continuous RC
        Serial.println("\n[7] CENTERING (v4 continuous RC)...");
        if (centerOnPad()) {
            // [8] Descend while correcting XY — v4 style
            Serial.println("\n[8] DESCENDING (v4 style: rc + throttle -15)...");
            descendAndLandOnPad();
        } else {
            Serial.println("Centering failed - landing wherever...");
            performLand();
        }
    } else {
        Serial.println("\n[7] PAD NOT FOUND - Landing wherever...");
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
    Serial.println("   TELLO v8 — LAND ON 10\" CIRCLE (v4 centering)");
    Serial.println("==========================================================\n");

    Serial.println("Mission: Take off, move 15\"+ away, return, land on 10\" circle");
    Serial.println("Centering: v4 style (continuous RC, fixed speed, simple)");
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

    Serial.println("PAD LANDING (v4 values):");
    Serial.printf("  Target zone:     +/-%dcm\n", TARGET_ZONE);
    Serial.printf("  Confirmations:   %d\n", CENTERED_COUNT_NEEDED);
    Serial.printf("  Nudge speed:     %d (fixed)\n", NUDGE_SPEED);
    Serial.printf("  Land height:     %dcm\n", LAND_HEIGHT);
    Serial.printf("  Pad offsets:     X=%d Y=%d\n", PAD_X_OFFSET, PAD_Y_OFFSET);
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
