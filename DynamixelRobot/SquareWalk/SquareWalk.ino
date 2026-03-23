// SquareWalk.ino
// Robot drives in a square: 10s forward → 90° CW spin → repeat 4 times
// Based on your friend's calibrated motor values and DynamixelShield library

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);  // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#else
  #define DEBUG_SERIAL Serial
#endif

using namespace ControlTableItem;

// ============================================================
//  MOTOR IDS (same as your friend's robot - don't change these
//  unless you physically rewired motors)
// ============================================================

// Steering motors (Position/Joint Mode) - rotate tire angles
const uint8_t steer_A = 4;
const uint8_t steer_B = 10;
const uint8_t steer_C = 6;
const uint8_t steer_D = 2;

// Drive motors (Velocity/Wheel Mode) - spin the wheels
const uint8_t drive_A = 26;
const uint8_t drive_B = 11;
const uint8_t drive_C = 7;
const uint8_t drive_D = 3;

const float dxl_protocol_version = 1.0;

// ============================================================
//  CALIBRATED VALUES (from your friend's testing)
//  These are tuned per-motor so the robot drives straight
// ============================================================

const uint16_t STEER_MOVE_SPEED = 100;  // How fast steering motors rotate

// Drive speeds (tuned per motor for straight-line driving)
const float SPD_A = 498.0;   // ID 26
const float SPD_B = 565.0;   // ID 11
const float SPD_C = 535.0;   // ID 7
const float SPD_D = 509.3;   // ID 3

// Circle/spin speeds (tuned for in-place rotation)
const float SPIN_SPD_A = 250.0;  // ID 26 (inner wheels slower)
const float SPIN_SPD_B = 250.0;  // ID 11
const float SPIN_SPD_C = 500.0;  // ID 7
const float SPIN_SPD_D = 500.0;  // ID 3

// ============================================================
//  CALIBRATED STEERING ANGLES (degrees)
//  These are specific to your robot's physical build.
//  Each motor has a different mechanical offset, so "straight"
//  is a different degree value for each one.
// ============================================================

// Straight - all wheels pointing forward
const float STRAIGHT_A = 116.02;
const float STRAIGHT_B = 187.50;
const float STRAIGHT_C = 111.00;
const float STRAIGHT_D = 201.86;

// Circle/Spin - wheels angled for in-place rotation
const float CIRCLE_A = 212.44;
const float CIRCLE_B = 80.02;
const float CIRCLE_C = 175.80;
const float CIRCLE_D = 137.79;

// ============================================================
//  TIMING (adjust these to tune the square)
// ============================================================

const uint32_t FORWARD_TIME_MS    = 10000;  // 10 seconds per side
const uint32_t SPIN_90_TIME_MS    = 6825;   // ~1/4 of 27300ms full circle
const uint32_t STEER_PAUSE_MS     = 1000;   // Wait before/after steering change
const uint32_t SETTLE_MS          = 1000;   // Pause between maneuvers
const int      NUM_SIDES          = 4;      // 4 sides = square

// ============================================================
//  DYNAMIXEL SHIELD OBJECT
// ============================================================
DynamixelShield dxl;

// ============================================================
//  CORE MOTOR FUNCTIONS
// ============================================================

// --- Stop all drive motors ---
void stopDrive() {
  // Motors A,D (IDs 26,3): value 0 = stopped in CCW context
  // Motors B,C (IDs 11,7): value 1024 = stopped in CW context
  // Both mean "zero speed" — different direction bits because
  // these motors are mounted on opposite sides of the robot
  dxl.setGoalVelocity(drive_A, 0);
  dxl.setGoalVelocity(drive_B, 1024);
  dxl.setGoalVelocity(drive_C, 1024);
  dxl.setGoalVelocity(drive_D, 0);
}

// --- Set a motor to spin "forward" ---
// IDs 11,7 (right side) are mirror-mounted, so their direction is flipped
void setForwardForDrive(uint8_t id, float value) {
  if (id == 11 || id == 7) {
    dxl.setGoalVelocity(id, 1024 + value);  // CW direction + speed
  } else {
    dxl.setGoalVelocity(id, value);          // CCW direction + speed
  }
}

// --- Set a motor to spin "backward" ---
void setBackwardForDrive(uint8_t id, float value) {
  if (id == 11 || id == 7) {
    dxl.setGoalVelocity(id, value);          // CCW direction + speed
  } else {
    dxl.setGoalVelocity(id, 1024 + value);   // CW direction + speed
  }
}

// ============================================================
//  STEERING FUNCTIONS
// ============================================================

void setSteeringStraight() {
  dxl.setGoalPosition(steer_A, STRAIGHT_A, UNIT_DEGREE);
  dxl.setGoalPosition(steer_B, STRAIGHT_B, UNIT_DEGREE);
  dxl.setGoalPosition(steer_C, STRAIGHT_C, UNIT_DEGREE);
  dxl.setGoalPosition(steer_D, STRAIGHT_D, UNIT_DEGREE);
}

void setSteeringCircle() {
  dxl.setGoalPosition(steer_A, CIRCLE_A, UNIT_DEGREE);
  dxl.setGoalPosition(steer_B, CIRCLE_B, UNIT_DEGREE);
  dxl.setGoalPosition(steer_C, CIRCLE_C, UNIT_DEGREE);
  dxl.setGoalPosition(steer_D, CIRCLE_D, UNIT_DEGREE);
}

// Safe steering change: stop wheels → wait → steer → wait
void safeSteerStraight() {
  stopDrive();
  delay(STEER_PAUSE_MS);
  setSteeringStraight();
  stopDrive();
  delay(STEER_PAUSE_MS);
}

void safeSteerCircle() {
  stopDrive();
  delay(STEER_PAUSE_MS);
  setSteeringCircle();
  stopDrive();
  delay(STEER_PAUSE_MS);
}

// ============================================================
//  MOVEMENT FUNCTIONS
// ============================================================

// All 4 wheels drive forward
void moveForward() {
  setForwardForDrive(drive_A, SPD_A);
  setForwardForDrive(drive_B, SPD_B);
  setForwardForDrive(drive_C, SPD_C);
  setForwardForDrive(drive_D, SPD_D);
}

// Spin clockwise (right turn) in place
// Left-side wheels go forward, right-side wheels go backward
// This creates a CW rotation when viewed from above
// NOTE: If robot spins the WRONG way, swap setForwardForDrive
//       and setBackwardForDrive for each motor below.
void spinCW() {
  setForwardForDrive(drive_A, SPIN_SPD_A);
  setBackwardForDrive(drive_B, SPIN_SPD_B);
  setForwardForDrive(drive_C, SPIN_SPD_C);
  setBackwardForDrive(drive_D, SPIN_SPD_D);
}

// ============================================================
//  SETUP — Runs the square walk sequence once on power-up
// ============================================================

void setup() {
  // --- Serial for debug messages ---
  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("SquareWalk starting...");

  // --- Initialize Dynamixel communication ---
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(dxl_protocol_version);

  // --- Ping all motors (verify they're connected) ---
  dxl.ping(steer_A);
  dxl.ping(steer_B);
  dxl.ping(steer_C);
  dxl.ping(steer_D);
  dxl.ping(drive_A);
  dxl.ping(drive_B);
  dxl.ping(drive_C);
  dxl.ping(drive_D);

  // --- Configure steering motors: Position Mode ---
  // Must turn torque OFF before changing operating mode,
  // then turn it back ON after.
  uint8_t steerMotors[] = {steer_A, steer_B, steer_C, steer_D};
  for (int i = 0; i < 4; i++) {
    dxl.torqueOff(steerMotors[i]);
    dxl.setOperatingMode(steerMotors[i], OP_POSITION);
    dxl.writeControlTableItem(MOVING_SPEED, steerMotors[i], STEER_MOVE_SPEED);
    dxl.torqueOn(steerMotors[i]);
  }

  // --- Configure drive motors: Velocity (Wheel) Mode ---
  uint8_t driveMotors[] = {drive_A, drive_B, drive_C, drive_D};
  for (int i = 0; i < 4; i++) {
    dxl.torqueOff(driveMotors[i]);
    dxl.setOperatingMode(driveMotors[i], OP_VELOCITY);
    dxl.torqueOn(driveMotors[i]);
  }

  // --- Start with wheels straight and stopped ---
  safeSteerStraight();
  delay(SETTLE_MS);

  DEBUG_SERIAL.println("Motors initialized. Starting square walk!");

  // ============================================================
  //  THE SQUARE WALK — 4 sides
  //
  //    Side 1          Turn 1
  //  --------->          |
  //  ^                   v
  //  | Turn 4       Side 2
  //  |                   |
  //  Side 4         Turn 2
  //  ^                   |
  //  |                   v
  //  <---------
  //    Side 3          Turn 3
  // ============================================================

  for (int side = 0; side < NUM_SIDES; side++) {

    // --- DRIVE STRAIGHT for 10 seconds ---
    DEBUG_SERIAL.print("Side ");
    DEBUG_SERIAL.print(side + 1);
    DEBUG_SERIAL.println(": Driving forward...");

    safeSteerStraight();
    moveForward();
    delay(FORWARD_TIME_MS);
    stopDrive();
    delay(SETTLE_MS);

    // --- SPIN 90° CLOCKWISE (skip after last side) ---
    if (side < NUM_SIDES - 1) {
      DEBUG_SERIAL.print("Turn ");
      DEBUG_SERIAL.print(side + 1);
      DEBUG_SERIAL.println(": Spinning 90 CW...");

      safeSteerCircle();
      spinCW();
      delay(SPIN_90_TIME_MS);
      stopDrive();
      delay(SETTLE_MS);
    }
  }

  // --- Final: straighten wheels and stop ---
  safeSteerStraight();
  stopDrive();
  DEBUG_SERIAL.println("Square walk complete!");
}

// ============================================================
//  LOOP — Keep motors stopped after routine finishes
// ============================================================

void loop() {
  stopDrive();
  delay(1000);
}
