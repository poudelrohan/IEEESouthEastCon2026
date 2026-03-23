// ============================================================
//  SOUTHEASTCON 2026 - FULL MISSION: ROBOT + TELLO DRONE
//
//  Single ESP32 controls both the MiniHexa hexapod AND
//  the Tello EDU drone via WiFi UDP commands.
//
//  Phase 1: Strafe left + WiFi connect (parallel)
//  Phase 2: Drone takeoff + forward 50cm
//  Phase 3: Robot forward + drone 180/return/land (parallel)
//  Phase 4: Crater square orbit (no drone)
//  Phase 5: Turn south + drone takeoff
//  Phase 6: Robot to south wall + strafe left
//  Phase 7: Drone left 20cm + pad search + land
// ============================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include "hiwonder_robot.h"

Robot minihexa;

// ══════════════════════════════════════════════════════════════
//  SECTION A: TELLO WIFI / NETWORK
// ══════════════════════════════════════════════════════════════

const char* TELLO_SSID       = "TELLO-BCU-DRONE";
const char* TELLO_PASS       = "southeast2026";
const char* TELLO_IP         = "192.168.10.1";
const int   TELLO_CMD_PORT   = 8889;
const int   TELLO_STATE_PORT = 8890;
const int   WIFI_TIMEOUT_MS  = 30000;  // 30 seconds max to connect

// ══════════════════════════════════════════════════════════════
//  SECTION B: ROBOT MOVEMENT
// ══════════════════════════════════════════════════════════════

const int   SLOW_GAIT_MS     = 600;    // Careful walking
const int   HEAVY_GAIT_MS    = 800;    // Deliberate steps for crater
const float SLOW_SPEED       = 1.5f;   // Forward speed
const float STRAFE_SPEED     = 1.5f;   // Left/Right speed

const int   TURN_90_STEPS    = 4;      // Steps for 90 degree turn
const float TURN_SPEED       = 1.8f;   // Turn omega

const float MAX_HEIGHT       = 3.0f;   // Highest position
const float NORMAL_HEIGHT    = 0.0f;   // Default standing

// ══════════════════════════════════════════════════════════════
//  SECTION C: PHASE PARAMETERS (ROBOT)
// ══════════════════════════════════════════════════════════════

const int   PHASE1_LEFT_STEPS   = 9;   // Strafe left at start
const int   PHASE3_FWD_STEPS    = 24;  // Forward to crater (dead reckoning)
const int   CRATER_HALF_SIDE    = 5;   // Entry/exit half-side steps
const int   CRATER_FULL_SIDE    = 9;   // Full side steps
const int   STOP_DIST_MM        = 70;  // Stop distance from wall (mm)
const int   PHASE6_STRAFE_STEPS = 15;  // Final strafe steps
const bool  PHASE6_STRAFE_LEFT  = true;// Final strafe direction

// ══════════════════════════════════════════════════════════════
//  SECTION D: DRONE FLIGHT
// ══════════════════════════════════════════════════════════════

const int   DRONE_FWD_CM            = 50;    // Forward distance for first flight
const int   DRONE_RETURN_CM         = 50;    // Return distance after 180 turn
const int   DRONE_LEFT_CM           = 20;    // Drone left toward robot in Phase 7
const int   DRONE_CMD_TIMEOUT_MS    = 10000; // Timeout waiting for drone response
const int   DRONE_STABILIZE_MS      = 2000;  // Wait after takeoff
const int   DRONE_PAUSE_AFTER_MS    = 500;   // Pause after movement command

// ══════════════════════════════════════════════════════════════
//  SECTION E: PAD LANDING
// ══════════════════════════════════════════════════════════════

const int   PAD_SEARCH_TIMEOUT_MS   = 8000;  // Max time to search for pad
const int   PAD_CENTER_TIMEOUT_MS   = 10000; // Max time to center on pad
const int   PAD_DESCEND_TIMEOUT_MS  = 10000; // Max time for descent
const int   PAD_TARGET_ZONE         = 15;    // Centered if within this (cm)
const int   PAD_CENTERED_NEEDED     = 10;    // Consecutive centered readings
const int   PAD_LAND_HEIGHT         = 45;    // Land when Z below this (cm)
const int   PAD_NUDGE_SPEED         = 15;    // RC speed for centering
const int   PAD_X_OFFSET            = 0;     // X offset from pad center
const int   PAD_Y_OFFSET            = 0;     // Y offset from pad center
const bool  INVERT_PITCH            = false;
const bool  INVERT_ROLL             = true;
const bool  USE_EMERGENCY_LAND      = false; // true=drop, false=gentle

// ══════════════════════════════════════════════════════════════
//  STATE VARIABLES
// ══════════════════════════════════════════════════════════════

// Robot state
Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t   pos = {0.0f, 0.0f, 0.0f};
Euler_t    att = {0.0f, 0.0f, 0.0f};
float current_height = NORMAL_HEIGHT;

// Drone state
WiFiUDP udpCmd;
WiFiUDP udpState;
bool DRONE_CONNECTED  = false;   // Master flag — set once at startup
bool droneInFlight    = false;   // True while drone is airborne
int  padId = -1;
int  padX  = 0, padY = 0, padZ = 0;
int  centeredCount    = 0;
int  droneBattery     = 0;
unsigned long lastPrintTime = 0;

// RGB colors
uint8_t RGB_GREEN[3]  = {0, 255, 0};
uint8_t RGB_RED[3]    = {255, 0, 0};
uint8_t RGB_BLUE[3]   = {0, 0, 255};
uint8_t RGB_YELLOW[3] = {255, 255, 0};
uint8_t RGB_OFF[3]    = {0, 0, 0};

// ══════════════════════════════════════════════════════════════
//  DRONE COMMUNICATION FUNCTIONS
// ══════════════════════════════════════════════════════════════

void drone_sendCommand(const char* cmd) {
    if (!DRONE_CONNECTED) return;
    Serial.printf("[DRONE] >> %s\n", cmd);
    udpCmd.beginPacket(TELLO_IP, TELLO_CMD_PORT);
    udpCmd.print(cmd);
    udpCmd.endPacket();
}

String drone_waitResponse(int timeoutMs = DRONE_CMD_TIMEOUT_MS) {
    if (!DRONE_CONNECTED) return "DISCONNECTED";
    unsigned long start = millis();
    while (millis() - start < (unsigned long)timeoutMs) {
        int size = udpCmd.parsePacket();
        if (size) {
            char buf[256];
            int len = udpCmd.read(buf, 255);
            buf[len] = 0;
            Serial.printf("[DRONE] << %s\n", buf);
            return String(buf);
        }
        delay(10);
    }
    Serial.println("[DRONE] << TIMEOUT");
    return "TIMEOUT";
}

bool drone_sendWithRetry(const char* cmd, int delayAfter = 0) {
    if (!DRONE_CONNECTED) return false;
    drone_sendCommand(cmd);
    String resp = drone_waitResponse();

    if (resp != "ok") {
        Serial.println("[DRONE]    Retrying...");
        delay(500);
        drone_sendCommand(cmd);
        resp = drone_waitResponse();
    }

    if (delayAfter > 0) delay(delayAfter);
    return (resp == "ok");
}

// ══════════════════════════════════════════════════════════════
//  DRONE STATE READING
// ══════════════════════════════════════════════════════════════

bool drone_readState() {
    if (!DRONE_CONNECTED) return false;

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
        droneBattery = latest.substring(idx + 4, latest.indexOf(";", idx)).toInt();
    }

    return true;
}

// ══════════════════════════════════════════════════════════════
//  DRONE RC CONTROL (for pad centering)
// ══════════════════════════════════════════════════════════════

void drone_sendRC(int roll, int pitch, int throttle, int yaw) {
    if (!DRONE_CONNECTED) return;
    char cmd[50];
    sprintf(cmd, "rc %d %d %d %d", roll, pitch, throttle, yaw);
    drone_sendCommand(cmd);
}

void drone_stopMovement() {
    drone_sendRC(0, 0, 0, 0);
    delay(100);
}

// ══════════════════════════════════════════════════════════════
//  DRONE HIGH-LEVEL FUNCTIONS
// ══════════════════════════════════════════════════════════════

bool drone_takeoff() {
    if (!DRONE_CONNECTED) return false;
    Serial.println("[DRONE] Taking off...");

    if (!drone_sendWithRetry("takeoff", DRONE_STABILIZE_MS)) {
        Serial.println("[DRONE] Takeoff FAILED — disabling drone ops");
        DRONE_CONNECTED = false;
        return false;
    }

    droneInFlight = true;
    Serial.println("[DRONE] Airborne!");
    return true;
}

void drone_land() {
    if (!DRONE_CONNECTED) return;
    Serial.println("[DRONE] Landing...");

    delay(200);
    if (USE_EMERGENCY_LAND) {
        drone_sendCommand("emergency");
    } else {
        drone_sendCommand("land");
    }
    drone_waitResponse(DRONE_CMD_TIMEOUT_MS);

    droneInFlight = false;
    Serial.println("[DRONE] Landed.");
}

// ══════════════════════════════════════════════════════════════
//  DRONE PAD DETECTION FUNCTIONS
// ══════════════════════════════════════════════════════════════

bool drone_searchForPad(int timeoutMs) {
    if (!DRONE_CONNECTED || !droneInFlight) return false;

    Serial.println("[DRONE] Searching for mission pad...");
    Serial.printf("[DRONE] Timeout: %d seconds\n", timeoutMs / 1000);

    unsigned long searchStart = millis();
    int lastSecPrinted = -1;

    while (millis() - searchStart < (unsigned long)timeoutMs) {
        drone_readState();

        int elapsedSec = (millis() - searchStart) / 1000;
        int remainSec  = (timeoutMs / 1000) - elapsedSec;

        if (elapsedSec != lastSecPrinted) {
            lastSecPrinted = elapsedSec;
            if (padId > 0) {
                Serial.printf("[DRONE] PAD FOUND! ID=%d X:%d Y:%d Z:%d\n",
                              padId, padX, padY, padZ);
                return true;
            } else {
                Serial.printf("[DRONE] Searching... %ds remaining\n", remainSec);
            }
        }

        delay(100);
    }

    Serial.println("[DRONE] Search TIMEOUT — pad not found");
    return false;
}

bool drone_centerOnPad() {
    if (!DRONE_CONNECTED || !droneInFlight) return false;

    Serial.println("[DRONE] Centering on pad...");
    centeredCount = 0;
    unsigned long startTime = millis();

    while (millis() - startTime < (unsigned long)PAD_CENTER_TIMEOUT_MS) {
        drone_readState();

        if (padId <= 0) {
            Serial.println("[DRONE] Lost pad — hovering...");
            drone_stopMovement();
            centeredCount = 0;
            delay(500);
            continue;
        }

        int errorX = padX - PAD_X_OFFSET;
        int errorY = padY - PAD_Y_OFFSET;

        bool xOk = abs(errorX) <= PAD_TARGET_ZONE;
        bool yOk = abs(errorY) <= PAD_TARGET_ZONE;
        bool centered = xOk && yOk;

        if (centered) {
            centeredCount++;
        } else {
            centeredCount = 0;
        }

        // Print status every 500ms
        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            Serial.printf("[DRONE] PAD:%d | X:%4d Y:%4d Z:%3d | Err X:%4d Y:%4d | ",
                          padId, padX, padY, padZ, errorX, errorY);
            if (centered) {
                Serial.printf("CENTERED [%d/%d]\n", centeredCount, PAD_CENTERED_NEEDED);
            } else {
                Serial.println("Adjusting...");
            }
        }

        if (centeredCount >= PAD_CENTERED_NEEDED) {
            Serial.println("[DRONE] CENTERING COMPLETE!");
            drone_stopMovement();
            return true;
        }

        // Compute RC corrections
        int roll = 0, pitch = 0;

        if (errorX > PAD_TARGET_ZONE)       pitch = -PAD_NUDGE_SPEED;
        else if (errorX < -PAD_TARGET_ZONE) pitch =  PAD_NUDGE_SPEED;

        if (errorY > PAD_TARGET_ZONE)       roll = -PAD_NUDGE_SPEED;
        else if (errorY < -PAD_TARGET_ZONE) roll =  PAD_NUDGE_SPEED;

        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL)  roll  = -roll;

        drone_sendRC(roll, pitch, 0, 0);
        delay(100);
    }

    Serial.println("[DRONE] Centering TIMEOUT!");
    return false;
}

bool drone_descendAndLand() {
    if (!DRONE_CONNECTED || !droneInFlight) return false;

    Serial.println("[DRONE] Descending to pad...");
    Serial.printf("[DRONE] Will land when Z < %dcm\n", PAD_LAND_HEIGHT);

    unsigned long startTime = millis();

    while (millis() - startTime < (unsigned long)PAD_DESCEND_TIMEOUT_MS) {
        drone_readState();

        int errorX = padX - PAD_X_OFFSET;
        int errorY = padY - PAD_Y_OFFSET;

        // Print status every 500ms
        if (millis() - lastPrintTime > 500) {
            lastPrintTime = millis();
            if (padId > 0) {
                Serial.printf("[DRONE] DESCENDING | Z:%3d | Err X:%4d Y:%4d\n",
                              padZ, errorX, errorY);
            } else {
                Serial.println("[DRONE] DESCENDING | Pad lost — continuing down...");
            }
        }

        // Close enough to land
        if (padId > 0 && padZ < PAD_LAND_HEIGHT) {
            Serial.printf("[DRONE] Z=%dcm — LOW ENOUGH, landing!\n", padZ);
            break;
        }

        // Pad lost during descent — land immediately
        if (padId <= 0) {
            Serial.println("[DRONE] Pad lost during descent — landing now!");
            break;
        }

        // RC corrections + descend
        int roll = 0, pitch = 0;

        if (errorX > PAD_TARGET_ZONE)       pitch = -PAD_NUDGE_SPEED;
        else if (errorX < -PAD_TARGET_ZONE) pitch =  PAD_NUDGE_SPEED;

        if (errorY > PAD_TARGET_ZONE)       roll = -PAD_NUDGE_SPEED;
        else if (errorY < -PAD_TARGET_ZONE) roll =  PAD_NUDGE_SPEED;

        if (INVERT_PITCH) pitch = -pitch;
        if (INVERT_ROLL)  roll  = -roll;

        drone_sendRC(roll, pitch, -15, 0);  // -15 throttle = descend
        delay(100);
    }

    drone_land();
    return true;
}

// ══════════════════════════════════════════════════════════════
//  ROBOT HELPER FUNCTIONS
// ══════════════════════════════════════════════════════════════

void stop_robot() {
    vel = {0.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, 600);
    delay(300);
}

void robot_pause() {
    stop_robot();
    delay(500);
}

void set_height(float target_z) {
    Serial.printf("[ROBOT] Height -> %.1f cm\n", target_z);
    current_height = target_z;
    vel = {0.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, 1000);
    delay(800);
}

void walk_forward(int steps, float speed, int gait_ms) {
    vel = {0.0f, speed, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, gait_ms, steps);
    delay((steps * gait_ms) + 500);
    stop_robot();
}

void walk_strafe(int steps, float speed, int gait_ms, bool left) {
    float vx = left ? -speed : speed;
    vel = {vx, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, gait_ms, steps);
    delay((steps * gait_ms) + 500);
    stop_robot();
}

void turn_left_90() {
    vel = {0.0f, 0.0f, TURN_SPEED};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, 1000, TURN_90_STEPS);
    delay((TURN_90_STEPS * 1000) + 500);
    stop_robot();
}

void turn_right_90() {
    vel = {0.0f, 0.0f, -TURN_SPEED};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, 1000, TURN_90_STEPS);
    delay((TURN_90_STEPS * 1000) + 500);
    stop_robot();
}

bool walk_until_wall(int max_steps, float speed, int gait_ms, int stop_dist_mm) {
    Serial.printf("[ROBOT] Walking to wall (stop at %dmm)...\n", stop_dist_mm);

    for (int i = 0; i < max_steps; i++) {
        uint16_t dist = minihexa.sensor.get_distance();
        Serial.printf("[ROBOT]   Step %d — Dist: %d mm\n", i + 1, dist);

        if (dist > 0 && dist <= stop_dist_mm) {
            Serial.println("[ROBOT]   Wall detected! Stopping.");
            stop_robot();
            return true;
        }

        vel = {0.0f, speed, 0.0f};
        pos = {0.0f, 0.0f, current_height};
        minihexa.move(&vel, &pos, &att, gait_ms, 1);
        delay(gait_ms + 100);
    }

    stop_robot();
    return false;
}

// ══════════════════════════════════════════════════════════════
//  PHASE 1: STRAFE LEFT + WIFI CONNECT (PARALLEL)
//
//  Robot strafes left via FreeRTOS timer while main thread
//  connects to Tello WiFi and enters SDK mode. (~15s total)
// ══════════════════════════════════════════════════════════════

void phase1_strafe_and_connect() {
    Serial.println("\n[Phase 1] Strafe left + WiFi connect (parallel)...");

    // Set max height — robot will rise during first strafe step
    current_height = MAX_HEIGHT;

    // Start strafe — non-blocking, FreeRTOS timer drives servos
    Serial.printf("[ROBOT] Strafing left: %d steps\n", PHASE1_LEFT_STEPS);
    vel = {-STRAFE_SPEED, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, SLOW_GAIT_MS, PHASE1_LEFT_STEPS);
    unsigned long strafeEnd = millis() + ((unsigned long)PHASE1_LEFT_STEPS * SLOW_GAIT_MS) + 500;

    // --- WiFi connect while robot strafes ---
    Serial.printf("[DRONE] Connecting to %s...\n", TELLO_SSID);
    minihexa.sensor.set_ultrasound_rgb(0, RGB_YELLOW, RGB_YELLOW);
    WiFi.begin(TELLO_SSID, TELLO_PASS);

    unsigned long wifiStart = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - wifiStart > (unsigned long)WIFI_TIMEOUT_MS) {
            Serial.println("\n[DRONE] WiFi FAILED — timeout!");
            minihexa.sensor.set_ultrasound_rgb(0, RGB_RED, RGB_RED);
            DRONE_CONNECTED = false;
            // Wait for strafe to finish
            while (millis() < strafeEnd) delay(50);
            stop_robot();
            return;
        }
        delay(500);
        Serial.print(".");
    }
    Serial.println(" Connected!");

    // Setup UDP sockets
    udpCmd.begin(TELLO_CMD_PORT);
    udpState.begin(TELLO_STATE_PORT);
    delay(1000);
    DRONE_CONNECTED = true;

    // Enter SDK mode
    if (!drone_sendWithRetry("command", 500)) {
        Serial.println("[DRONE] SDK mode FAILED!");
        minihexa.sensor.set_ultrasound_rgb(0, RGB_RED, RGB_RED);
        DRONE_CONNECTED = false;
    } else {
        // Check battery
        drone_sendCommand("battery?");
        String bat = drone_waitResponse(3000);
        droneBattery = bat.toInt();
        Serial.printf("[DRONE] Battery: %d%%\n", droneBattery);
        minihexa.sensor.set_ultrasound_rgb(0, RGB_GREEN, RGB_GREEN);
        Serial.println("[DRONE] Ready!");
    }

    // Wait for strafe to finish (WiFi usually takes longer, so this may already be done)
    if (millis() < strafeEnd) {
        Serial.printf("[ROBOT] Waiting %lu ms for strafe to finish...\n", strafeEnd - millis());
        while (millis() < strafeEnd) delay(50);
    }
    stop_robot();
}

// ══════════════════════════════════════════════════════════════
//  PHASE 3: ROBOT FORWARD + DRONE 180/RETURN/LAND (PARALLEL)
//
//  Robot walks forward using FreeRTOS timer (autonomous servos).
//  During the walk, we send drone return commands.
//  Robot does NOT wait for drone — goes straight to crater.
// ══════════════════════════════════════════════════════════════

void phase3_forward_with_drone() {
    Serial.println("\n[Phase 3] Robot forward + drone 180/return/land (parallel)...");

    // Start robot walking — FreeRTOS timer drives servos in background
    vel = {0.0f, SLOW_SPEED, 0.0f};
    pos = {0.0f, 0.0f, current_height};
    minihexa.move(&vel, &pos, &att, SLOW_GAIT_MS, PHASE3_FWD_STEPS);
    unsigned long walkEnd = millis() + ((unsigned long)PHASE3_FWD_STEPS * SLOW_GAIT_MS) + 500;

    // --- Drone commands during walk (time-budgeted to walk duration) ---
    if (DRONE_CONNECTED && droneInFlight) {
        long timeLeft;

        // 1) Rotate 180 degrees
        Serial.println("[DRONE] Rotating 180 degrees...");
        timeLeft = (long)(walkEnd - millis());
        if (timeLeft > 1000) {
            drone_sendCommand("cw 180");
            drone_waitResponse(min(timeLeft, (long)DRONE_CMD_TIMEOUT_MS));

            timeLeft = (long)(walkEnd - millis());
            if (timeLeft > DRONE_PAUSE_AFTER_MS) delay(DRONE_PAUSE_AFTER_MS);
        }

        // 2) Forward back toward start
        timeLeft = (long)(walkEnd - millis());
        if (timeLeft > 1000) {
            char cmd[30];
            sprintf(cmd, "forward %d", DRONE_RETURN_CM);
            drone_sendCommand(cmd);
            drone_waitResponse(min(timeLeft, (long)DRONE_CMD_TIMEOUT_MS));

            timeLeft = (long)(walkEnd - millis());
            if (timeLeft > DRONE_PAUSE_AFTER_MS) delay(DRONE_PAUSE_AFTER_MS);
        }

        // 3) Land — always send, even if walk time is up
        drone_sendCommand("land");
        timeLeft = (long)(walkEnd - millis());
        if (timeLeft > 1000) {
            drone_waitResponse(min(timeLeft, (long)DRONE_CMD_TIMEOUT_MS));
        }
        droneInFlight = false;
    }

    // Wait for walk to finish (may already be done)
    while (millis() < walkEnd) delay(50);
    stop_robot();
    // NO pause — go straight to crater
}

// ══════════════════════════════════════════════════════════════
//  PHASE 4: CRATER SQUARE ORBIT (robot only, no drone)
// ══════════════════════════════════════════════════════════════

void phase4_crater_orbit() {
    Serial.println("\n[Phase 4] Crater square orbit (max height, hard steps)...");

    // Entry half-side: turn right, walk east
    turn_right_90();
    robot_pause();
    Serial.printf("[ROBOT]   Half-side 1: %d steps east\n", CRATER_HALF_SIDE);
    walk_forward(CRATER_HALF_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Full side: turn left, walk north
    turn_left_90();
    robot_pause();
    Serial.printf("[ROBOT]   Full side 2: %d steps north\n", CRATER_FULL_SIDE);
    walk_forward(CRATER_FULL_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Full side: turn left, walk west
    turn_left_90();
    robot_pause();
    Serial.printf("[ROBOT]   Full side 3: %d steps west\n", CRATER_FULL_SIDE);
    walk_forward(CRATER_FULL_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Full side: turn left, walk south
    turn_left_90();
    robot_pause();
    Serial.printf("[ROBOT]   Full side 4: %d steps south\n", CRATER_FULL_SIDE);
    walk_forward(CRATER_FULL_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Exit half-side: turn left, walk east back to entry
    turn_left_90();
    robot_pause();
    Serial.printf("[ROBOT]   Half-side 5: %d steps east (back to entry)\n", CRATER_HALF_SIDE);
    walk_forward(CRATER_HALF_SIDE, SLOW_SPEED, HEAVY_GAIT_MS);
    robot_pause();

    // Robot is now back at crater entry, facing EAST
}

// ══════════════════════════════════════════════════════════════
//  MAIN MISSION — runs entirely in setup()
// ══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n[BOOT] Waiting 10 seconds (download mode window)...");
    delay(10000);  // Remove this for competition

    Serial.println("[BOOT] Initializing robot...");
    minihexa.begin();
    Serial.println("[BOOT] Robot ready. Stabilizing...");
    delay(3000);   // Stand up and stabilize

    Serial.println("══════════════════════════════════════════════════════════");
    Serial.println("  SOUTHEASTCON 2026 — FULL MISSION (ROBOT + DRONE)");
    Serial.println("══════════════════════════════════════════════════════════\n");

    // -----------------------------------------------
    //  PHASE 1: Strafe left + WiFi connect (parallel)
    // -----------------------------------------------
    phase1_strafe_and_connect();

    if (DRONE_CONNECTED) {
        Serial.println("[Phase 1] DRONE READY — full mission with UAV\n");
    } else {
        Serial.println("[Phase 1] DRONE NOT CONNECTED — robot-only mission\n");
    }

    // -----------------------------------------------
    //  PHASE 2: Drone takeoff + forward 50cm
    // -----------------------------------------------
    if (DRONE_CONNECTED) {
        Serial.println("\n[Phase 2] Drone takeoff + forward...");
        if (drone_takeoff()) {
            char cmd[30];
            sprintf(cmd, "forward %d", DRONE_FWD_CM);
            drone_sendWithRetry(cmd, DRONE_PAUSE_AFTER_MS);
            Serial.printf("[DRONE] Hovering %dcm ahead of robot\n", DRONE_FWD_CM);
        }
    } else {
        Serial.println("\n[Phase 2] Skipped — no drone");
    }

    // -----------------------------------------------
    //  PHASE 3: Robot forward + drone 180/return/land
    //  (parallel — robot walks while drone returns)
    //  Robot does NOT wait for drone.
    // -----------------------------------------------
    phase3_forward_with_drone();

    // -----------------------------------------------
    //  PHASE 4: Crater square orbit (robot only)
    // -----------------------------------------------
    phase4_crater_orbit();

    // Robot is now at crater entry, facing EAST

    // -----------------------------------------------
    //  PHASE 5: Turn south + drone takeoff
    // -----------------------------------------------
    Serial.println("\n[Phase 5] Turn south + drone takeoff...");
    set_height(MAX_HEIGHT);

    // Enable mission pad detection before takeoff
    if (DRONE_CONNECTED) {
        drone_sendWithRetry("mon", 300);
        drone_sendWithRetry("mdirection 0", 300);
    }

    // Turn to face south
    turn_right_90();  // East -> South

    // Drone takes off right after turn
    if (DRONE_CONNECTED) {
        Serial.println("[Phase 5] Drone second takeoff...");
        drone_takeoff();
    }

    // -----------------------------------------------
    //  PHASE 6: Robot to south wall + strafe left
    //  (drone is hovering in the air)
    // -----------------------------------------------
    Serial.println("\n[Phase 6] Robot to south wall + strafe left...");

    // Walk to south wall
    walk_until_wall(100, SLOW_SPEED, SLOW_GAIT_MS, STOP_DIST_MM);
    robot_pause();

    // Strafe left back to starting area
    Serial.printf("[ROBOT] Strafing left: %d steps\n", PHASE6_STRAFE_STEPS);
    walk_strafe(PHASE6_STRAFE_STEPS, STRAFE_SPEED, SLOW_GAIT_MS, PHASE6_STRAFE_LEFT);
    stop_robot();

    // -----------------------------------------------
    //  PHASE 7: Drone left 20cm + pad search + land
    //  (robot is stationary in starting area)
    // -----------------------------------------------
    if (DRONE_CONNECTED && droneInFlight) {
        Serial.println("\n[Phase 7] Drone left 20cm + pad search + land...");

        // Drone goes left toward robot
        char cmd[30];
        sprintf(cmd, "left %d", DRONE_LEFT_CM);
        drone_sendWithRetry(cmd, DRONE_PAUSE_AFTER_MS);

        // Search for mission pad on robot's back
        bool padFound = drone_searchForPad(PAD_SEARCH_TIMEOUT_MS);

        if (padFound) {
            Serial.println("[DRONE] Centering on pad...");
            if (drone_centerOnPad()) {
                drone_descendAndLand();
            } else {
                Serial.println("[DRONE] Centering failed — landing wherever");
                drone_land();
            }
        } else {
            Serial.println("[DRONE] Pad not found — landing wherever");
            drone_land();
        }
    } else {
        Serial.println("\n[Phase 7] Skipped — no drone");
    }

    // -----------------------------------------------
    //  MISSION COMPLETE
    // -----------------------------------------------
    stop_robot();
    set_height(NORMAL_HEIGHT);
    minihexa.sensor.set_ultrasound_rgb(0, RGB_BLUE, RGB_BLUE);

    Serial.println("\n══════════════════════════════════════════════════════════");
    Serial.println("  === MISSION COMPLETE ===");
    Serial.println("══════════════════════════════════════════════════════════\n");
}

void loop() {
    // Mission complete — nothing to do
}
