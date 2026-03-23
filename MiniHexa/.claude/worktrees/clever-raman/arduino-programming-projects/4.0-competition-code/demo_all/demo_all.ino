// demo_all.ino
// IEEE SoutheastCon 2026 - MiniHexa Full Capability Demo
//
// Runs through ALL robot capabilities one by one.
// Each case prints what it's doing to Serial Monitor (115200 baud).
//
// AUTONOMOUS mode (set below):
//   true  = runs all phases automatically, no Serial input needed
//   false = waits for Serial input between phases ('n'=next, 'r'=repeat)

#include "hiwonder_robot.h"

// ============================================================
//  SET THIS BEFORE UPLOADING
// ============================================================
const bool AUTONOMOUS = true;  // true = auto-run all, false = Serial control

Robot minihexa;

Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t   pos = {0.0f, 0.0f, 0.0f};
Euler_t    att = {0.0f, 0.0f, 0.0f};

uint8_t count = 0;
bool demo_done = false;

// Phase start cases (for repeat functionality)
const uint8_t PHASE1_START = 0;   // Movement
const uint8_t PHASE2_START = 11;  // Body Position
const uint8_t PHASE3_START = 18;  // Body Tilt
const uint8_t PHASE4_START = 25;  // Animations
const uint8_t PHASE5_START = 31;  // Ultrasonic

// Track which phase we just finished so 'r' knows where to jump back
uint8_t current_phase_start = PHASE1_START;

// ============================================================
//  HELPERS
// ============================================================

// Stop all movement and reset structs to zero
void stop_and_reset() {
    vel = {0.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, 0.0f};
    att = {0.0f, 0.0f, 0.0f};
    minihexa.move(&vel, &pos, &att);
    delay(500);
}

// Print case info to Serial
void print_case(uint8_t num, const char* phase, const char* description) {
    Serial.printf("\n>>> Case %d [%s]: %s\n", num, phase, description);
}

// Wait for Serial input between phases (or auto-advance if AUTONOMOUS)
void wait_for_serial(const char* current_phase, const char* next_phase) {
    stop_and_reset();
    Serial.println();
    Serial.println("========================================");
    Serial.printf("  FINISHED: %s\n", current_phase);
    Serial.printf("  UP NEXT:  %s\n", next_phase);
    Serial.println("========================================");

    if (AUTONOMOUS) {
        Serial.println("  [AUTONOMOUS] Continuing in 3 seconds...");
        delay(3000);
        Serial.printf("\n>> Starting: %s\n\n", next_phase);
        return;
    }

    Serial.println("----------------------------------------");
    Serial.println("  Type 'n' + Enter = next phase");
    Serial.println("  Type 'r' + Enter = repeat this phase");
    Serial.println("========================================");

    // Flush any leftover serial data
    while (Serial.available()) Serial.read();

    // Wait for input
    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            // Flush the rest (newline, etc.)
            delay(50);
            while (Serial.available()) Serial.read();

            if (c == 'n' || c == 'N') {
                Serial.printf("\n>> Starting: %s\n\n", next_phase);
                return;
            } else if (c == 'r' || c == 'R') {
                Serial.printf("\n>> Repeating: %s\n\n", current_phase);
                count = current_phase_start;
                return;
            } else {
                Serial.println("  Invalid input. Type 'n' for next or 'r' to repeat.");
            }
        }
        delay(50);
    }
}

// Standard 2-second pause between cases
void pause_between() {
    stop_and_reset();
    delay(2000);
}

// ============================================================
//  SETUP
// ============================================================

void setup() {
    Serial.begin(115200);
    minihexa.begin();
    delay(2000);

    Serial.println("============================================");
    Serial.println("  MiniHexa Full Capability Demo");
    Serial.println("  IEEE SoutheastCon 2026");
    Serial.printf("  Mode: %s\n", AUTONOMOUS ? "AUTONOMOUS" : "SERIAL CONTROL");
    Serial.println("============================================");
    Serial.println();

    if (AUTONOMOUS) {
        Serial.println("  [AUTONOMOUS] Starting in 3 seconds...");
        delay(3000);
        Serial.println(">> Starting Phase 1: Basic Movement\n");
    } else {
        Serial.println("  Type 'n' + Enter to start Phase 1: Basic Movement");
        Serial.println();

        // Flush any leftover serial data
        while (Serial.available()) Serial.read();

        // Wait for 'n' to start
        while (true) {
            if (Serial.available()) {
                char c = Serial.read();
                delay(50);
                while (Serial.available()) Serial.read();
                if (c == 'n' || c == 'N') {
                    Serial.println(">> Starting Phase 1: Basic Movement\n");
                    break;
                }
            }
            delay(50);
        }
    }
}

// ============================================================
//  LOOP
// ============================================================

void loop() {
    if (demo_done) return;

    switch (count) {

        // ========================================================
        //  PHASE 1: BASIC MOVEMENT (cases 0-9)
        // ========================================================

        case 0:
            current_phase_start = PHASE1_START;
            print_case(0, "Movement", "Walk FORWARD");
            vel = {0.0f, 3.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600, 5);
            delay(5 * 600 + 200);
            pause_between();
            count++;
            break;

        case 1:
            print_case(1, "Movement", "Walk BACKWARD");
            vel = {0.0f, -3.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600, 5);
            delay(5 * 600 + 200);
            pause_between();
            count++;
            break;

        case 2:
            print_case(2, "Movement", "Strafe RIGHT");
            vel = {3.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600, 5);
            delay(5 * 600 + 200);
            pause_between();
            count++;
            break;

        case 3:
            print_case(3, "Movement", "Strafe LEFT");
            vel = {-3.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600, 5);
            delay(5 * 600 + 200);
            pause_between();
            count++;
            break;

        case 4:
            print_case(4, "Movement", "Diagonal FORWARD-RIGHT");
            vel = {2.0f, 2.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600, 5);
            delay(5 * 600 + 200);
            pause_between();
            count++;
            break;

        case 5:
            print_case(5, "Movement", "Diagonal FORWARD-LEFT");
            vel = {-2.0f, 2.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600, 5);
            delay(5 * 600 + 200);
            pause_between();
            count++;
            break;

        case 6:
            print_case(6, "Movement", "Diagonal BACKWARD-RIGHT");
            vel = {2.0f, -2.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600, 5);
            delay(5 * 600 + 200);
            pause_between();
            count++;
            break;

        case 7:
            print_case(7, "Movement", "Diagonal BACKWARD-LEFT");
            vel = {-2.0f, -2.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600, 5);
            delay(5 * 600 + 200);
            pause_between();
            count++;
            break;

        case 8:
            print_case(8, "Movement", "Turn LEFT (CCW) in place");
            vel = {0.0f, 0.0f, 2.0f};
            minihexa.move(&vel, &pos, &att, 1000, 4);
            delay(4 * 1000 + 200);
            pause_between();
            count++;
            break;

        case 9:
            print_case(9, "Movement", "Turn RIGHT (CW) in place");
            vel = {0.0f, 0.0f, -2.0f};
            minihexa.move(&vel, &pos, &att, 1000, 4);
            delay(4 * 1000 + 200);
            pause_between();
            count++;
            break;

        case 10:
            wait_for_serial("Phase 1: Basic Movement",
                            "Phase 2: Body Position (Pose Shifts)");
            if (count == PHASE1_START) break;  // user chose repeat
            count = PHASE2_START;
            break;

        // ========================================================
        //  PHASE 2: POSE CONTROL — BODY POSITION (cases 11-16)
        // ========================================================

        case 11:
            current_phase_start = PHASE2_START;
            print_case(11, "Pose", "Shift body RIGHT (+3cm X)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {3.0f, 0.0f, 0.0f};
            att = {0.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 12:
            print_case(12, "Pose", "Shift body LEFT (-3cm X)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {-3.0f, 0.0f, 0.0f};
            att = {0.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 13:
            print_case(13, "Pose", "Shift body FORWARD (+3cm Y)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, 3.0f, 0.0f};
            att = {0.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 14:
            print_case(14, "Pose", "Shift body BACKWARD (-3cm Y)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, -3.0f, 0.0f};
            att = {0.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 15:
            print_case(15, "Pose", "Raise body UP (+3cm Z)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, 0.0f, 3.0f};
            att = {0.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 16:
            print_case(16, "Pose", "Lower body DOWN (-2cm Z)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, 0.0f, -2.0f};
            att = {0.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 17:
            wait_for_serial("Phase 2: Body Position",
                            "Phase 3: Body Tilt (Euler Angles)");
            if (count == PHASE2_START) break;  // user chose repeat
            count = PHASE3_START;
            break;

        // ========================================================
        //  PHASE 3: POSE CONTROL — BODY TILT (cases 18-23)
        // ========================================================

        case 18:
            current_phase_start = PHASE3_START;
            print_case(18, "Tilt", "Roll RIGHT (+8 degrees)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, 0.0f, 0.0f};
            att = {8.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 19:
            print_case(19, "Tilt", "Roll LEFT (-8 degrees)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, 0.0f, 0.0f};
            att = {-8.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 20:
            print_case(20, "Tilt", "Pitch FORWARD (+12 degrees)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, 0.0f, 0.0f};
            att = {0.0f, 12.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 21:
            print_case(21, "Tilt", "Pitch BACKWARD (-12 degrees)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, 0.0f, 0.0f};
            att = {0.0f, -12.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 22:
            print_case(22, "Tilt", "Yaw LEFT (+12 degrees)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, 0.0f, 0.0f};
            att = {0.0f, 0.0f, 12.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 23:
            print_case(23, "Tilt", "Yaw RIGHT (-12 degrees)");
            vel = {0.0f, 0.0f, 0.0f};
            pos = {0.0f, 0.0f, 0.0f};
            att = {0.0f, 0.0f, -12.0f};
            minihexa.move(&vel, &pos, &att, 600);
            delay(2000);
            pause_between();
            count++;
            break;

        case 24:
            wait_for_serial("Phase 3: Body Tilt",
                            "Phase 4: Preprogrammed Animations");
            if (count == PHASE3_START) break;  // user chose repeat
            count = PHASE4_START;
            break;

        // ========================================================
        //  PHASE 4: PREPROGRAMMED MOVES (cases 25-29)
        // ========================================================

        case 25:
            current_phase_start = PHASE4_START;
            print_case(25, "Animation", "Short stretch (_wake_up)");
            minihexa._wake_up();
            pause_between();
            count++;
            break;

        case 26:
            print_case(26, "Animation", "Full wake up + walk (wake_up)");
            minihexa.wake_up();
            pause_between();
            count++;
            break;

        case 27:
            print_case(27, "Animation", "Acting cute (nod + sway)");
            minihexa.acting_cute();
            pause_between();
            count++;
            break;

        case 28:
            print_case(28, "Animation", "Twist body CCW");
            minihexa.twist(15.0f, 3, 15, COUNTER_CLOCKWISE);
            pause_between();
            count++;
            break;

        case 29:
            print_case(29, "Animation", "Twist body CW");
            minihexa.twist(15.0f, 3, 15, CLOCKWISE);
            pause_between();
            count++;
            break;

        case 30:
            wait_for_serial("Phase 4: Animations",
                            "Phase 5: Ultrasonic Sensor Demo");
            if (count == PHASE4_START) break;  // user chose repeat
            count = PHASE5_START;
            break;

        // ========================================================
        //  PHASE 5: ULTRASONIC SENSOR (cases 31-32)
        // ========================================================

        case 31: {
            current_phase_start = PHASE5_START;
            print_case(31, "Sensor", "Read ultrasonic distance for 5 seconds");
            Serial.println("  Move your hand closer/further to see readings change.");
            unsigned long start = millis();
            while (millis() - start < 5000) {
                uint16_t dis = minihexa.sensor.get_distance();
                Serial.printf("  Distance: %d mm\n", dis);
                delay(200);
            }
            pause_between();
            count++;
            break;
        }

        case 32: {
            print_case(32, "Sensor", "Walk forward, STOP when obstacle < 200mm");
            Serial.println("  Place an obstacle in front. Robot walks until it detects it.");
            Serial.println("  (15 second timeout if no obstacle)");

            // Start walking forward continuously
            vel = {0.0f, 2.0f, 0.0f};
            pos = {0.0f, 0.0f, 0.0f};
            att = {0.0f, 0.0f, 0.0f};
            minihexa.move(&vel, &pos, &att, 800);

            // Poll ultrasonic until obstacle detected or 15 second timeout
            unsigned long start = millis();
            while (millis() - start < 15000) {
                uint16_t dis = minihexa.sensor.get_distance();
                Serial.printf("  Walking... Distance: %d mm\n", dis);
                if (dis > 0 && dis < 200) {
                    Serial.printf("  OBSTACLE DETECTED at %d mm! Stopping.\n", dis);
                    break;
                }
                delay(100);
            }

            stop_and_reset();
            pause_between();
            count++;
            break;
        }

        // ========================================================
        //  DONE
        // ========================================================

        case 33:
            Serial.println("\n============================================");
            Serial.println("  DEMO COMPLETE!");
            Serial.println("  All capabilities demonstrated.");
            Serial.println("  Type 'r' + Enter to restart from Phase 5,");
            Serial.println("  or reset the board to start over.");
            Serial.println("============================================");
            minihexa.reset();
            demo_done = true;
            break;
    }
}
