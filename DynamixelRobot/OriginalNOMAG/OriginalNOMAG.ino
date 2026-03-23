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

// Steering motors: A=4, B=10, C=6, D=2
const uint8_t steer_A = 4;
const uint8_t steer_B = 10;
const uint8_t steer_C = 6;
const uint8_t steer_D = 2;

// Drive motors: A=26, B=11, C=7, D=3
const uint8_t drive_A = 26;
const uint8_t drive_B = 11;
const uint8_t drive_C = 7;
const uint8_t drive_D = 3;

const float dxl_protocol_version = 1.0;
const uint16_t SLOW_SPEED = 100;

// Default magnitudes (non-circle)
const float SPD_A = 498.0;   // ID 26
const float SPD_B = 565.0;   // ID 11
const float SPD_C = 535.0;   // ID 7
const float SPD_D = 509.3;   // ID 3

// Circle magnitudes
const float CIRCLE_SPD_A = 250.0;  // ID 26
const float CIRCLE_SPD_B = 250.0;  // ID 11
const float CIRCLE_SPD_C = 500.0;  // ID 7
const float CIRCLE_SPD_D = 500.0;  // ID 3

// Routine timing (ms)
const uint32_t SHIFT_RIGHT_BACKWARD_MS = 5000;
const uint32_t FORWARD_MS = 13000;
const uint32_t CIRCLE_MS = 30000;
const uint32_t RETURN_BACKWARD_MS = 13000;
const uint32_t RETURN_RIGHT_MS = 5300;

// Safety timing
const uint32_t STEER_STOP_BEFORE_MS = 1000;
const uint32_t STEER_STOP_AFTER_MS = 1000;
const uint32_t STOP_SETTLE_MS = 1000;

DynamixelShield dxl;

void stopDrive() {
  dxl.setGoalVelocity(drive_A, 0);
  dxl.setGoalVelocity(drive_B, 1024);
  dxl.setGoalVelocity(drive_C, 1024);
  dxl.setGoalVelocity(drive_D, 0);
}

void setForwardForDrive(uint8_t id, float value) {
  if (id == 11 || id == 7) {
    dxl.setGoalVelocity(id, 1024 + value);
  } else {
    dxl.setGoalVelocity(id, value);
  }
}

void setBackwardForDrive(uint8_t id, float value) {
  if (id == 11 || id == 7) {
    dxl.setGoalVelocity(id, value);
  } else {
    dxl.setGoalVelocity(id, 1024 + value);
  }
}

// Straight steering angles
void setSteeringStraight() {
  dxl.setGoalPosition(steer_A, 114.02, UNIT_DEGREE);
  dxl.setGoalPosition(steer_B, 184.50, UNIT_DEGREE);
  dxl.setGoalPosition(steer_C, 111.00, UNIT_DEGREE);
  dxl.setGoalPosition(steer_D, 201.86, UNIT_DEGREE);
}

// Right-90 steering angles
void setSteeringRight90() {
  dxl.setGoalPosition(steer_A, 200.44, UNIT_DEGREE);
  dxl.setGoalPosition(steer_B, 96.02, UNIT_DEGREE);
  dxl.setGoalPosition(steer_C, 199.80, UNIT_DEGREE);
  dxl.setGoalPosition(steer_D, 112.79, UNIT_DEGREE);
}

// Circle steering angles (shift-right style)
void setSteeringCircle() {
  dxl.setGoalPosition(steer_A, 203.44, UNIT_DEGREE);
  dxl.setGoalPosition(steer_B, 82.02, UNIT_DEGREE);
  dxl.setGoalPosition(steer_C, 185.80, UNIT_DEGREE);
  dxl.setGoalPosition(steer_D, 122.79, UNIT_DEGREE);
}

void setSteeringStraightWithStops() {
  stopDrive();
  delay(STEER_STOP_BEFORE_MS);
  setSteeringStraight();
  stopDrive();
  delay(STEER_STOP_AFTER_MS);
}

void setSteeringRight90WithStops() {
  stopDrive();
  delay(STEER_STOP_BEFORE_MS);
  setSteeringRight90();
  stopDrive();
  delay(STEER_STOP_AFTER_MS);
}

void setSteeringCircleWithStops() {
  stopDrive();
  delay(STEER_STOP_BEFORE_MS);
  setSteeringCircle();
  stopDrive();
  delay(STEER_STOP_AFTER_MS);
}

// Shift right using DEFAULT speeds.
void shiftRight() {
  setForwardForDrive(drive_A, SPD_A);
  setForwardForDrive(drive_B, SPD_B);
  setBackwardForDrive(drive_C, SPD_C);
  setBackwardForDrive(drive_D, SPD_D);
}

// Return-step variant: only B and C are flipped relative to shiftRight().
void shiftRightReturnAdjusted() {
  setForwardForDrive(drive_A, SPD_A);
  setBackwardForDrive(drive_B, SPD_B);
  setForwardForDrive(drive_C, SPD_C);
  setBackwardForDrive(drive_D, SPD_D);
}

// Shift right with all wheel directions reversed from shiftRight().
void shiftRightBackwardAll() {
  setBackwardForDrive(drive_A, SPD_A);
  setForwardForDrive(drive_B, SPD_B);
  setBackwardForDrive(drive_C, SPD_C);
  setForwardForDrive(drive_D, SPD_D);
}

// Move forward using DEFAULT speeds.
void moveForward() {
  setForwardForDrive(drive_A, SPD_A);
  setForwardForDrive(drive_B, SPD_B);
  setForwardForDrive(drive_C, SPD_C);
  setForwardForDrive(drive_D, SPD_D);
}

// Move backward using DEFAULT speeds (inverse of moveForward).
void moveBackward() {
  setBackwardForDrive(drive_A, SPD_A);
  setBackwardForDrive(drive_B, SPD_B);
  setBackwardForDrive(drive_C, SPD_C);
  setBackwardForDrive(drive_D, SPD_D);
}

// Circle turn = shift-right pattern with CIRCLE angles and CIRCLE speeds.
void spinCircle() {
  setSteeringCircleWithStops();

  setForwardForDrive(drive_A, CIRCLE_SPD_A);
  setBackwardForDrive(drive_B, CIRCLE_SPD_B);
  setForwardForDrive(drive_C, CIRCLE_SPD_C);
  setBackwardForDrive(drive_D, CIRCLE_SPD_D);

  delay(CIRCLE_MS);
  stopDrive();
}

void setup() {
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(dxl_protocol_version);

  dxl.ping(steer_A);
  dxl.ping(steer_B);
  dxl.ping(steer_C);
  dxl.ping(steer_D);
  dxl.ping(drive_A);
  dxl.ping(drive_B);
  dxl.ping(drive_C);
  dxl.ping(drive_D);

  dxl.torqueOff(steer_A);
  dxl.setOperatingMode(steer_A, OP_POSITION);
  dxl.writeControlTableItem(MOVING_SPEED, steer_A, SLOW_SPEED);
  dxl.torqueOn(steer_A);

  dxl.torqueOff(steer_B);
  dxl.setOperatingMode(steer_B, OP_POSITION);
  dxl.writeControlTableItem(MOVING_SPEED, steer_B, SLOW_SPEED);
  dxl.torqueOn(steer_B);

  dxl.torqueOff(steer_C);
  dxl.setOperatingMode(steer_C, OP_POSITION);
  dxl.writeControlTableItem(MOVING_SPEED, steer_C, SLOW_SPEED);
  dxl.torqueOn(steer_C);

  dxl.torqueOff(steer_D);
  dxl.setOperatingMode(steer_D, OP_POSITION);
  dxl.writeControlTableItem(MOVING_SPEED, steer_D, SLOW_SPEED);
  dxl.torqueOn(steer_D);

  dxl.torqueOff(drive_A);
  dxl.setOperatingMode(drive_A, OP_VELOCITY);
  dxl.torqueOn(drive_A);

  dxl.torqueOff(drive_B);
  dxl.setOperatingMode(drive_B, OP_VELOCITY);
  dxl.torqueOn(drive_B);

  dxl.torqueOff(drive_C);
  dxl.setOperatingMode(drive_C, OP_VELOCITY);
  dxl.torqueOn(drive_C);

  dxl.torqueOff(drive_D);
  dxl.setOperatingMode(drive_D, OP_VELOCITY);
  dxl.torqueOn(drive_D);

  delay(15000);
  setSteeringStraightWithStops();
  delay(STOP_SETTLE_MS);

  //1) Turn to 90 and shift-right with reversed wheel spin and do press button.
  setSteeringRight90WithStops();
  delay(1000);
  shiftRightBackwardAll();
  delay(SHIFT_RIGHT_BACKWARD_MS);
  stopDrive();
  delay(STOP_SETTLE_MS);
  delay(15000);
  shiftRightBackwardAll();
  delay(3000);
  stopDrive();
  delay(STOP_SETTLE_MS);
  shiftRightReturnAdjusted();
  delay(500);
  shiftRightBackwardAll();
  delay(1000);
  shiftRightReturnAdjusted();
  delay(500);
  shiftRightBackwardAll();
  delay(1000);
  shiftRightReturnAdjusted();
  delay(500);
  shiftRightBackwardAll();
  delay(1000);
  shiftRightReturnAdjusted();
  delay(3000);


  //2) Straighten and move forward for 7600ms.
  //delay(60000);
  setSteeringStraightWithStops();
  delay(1000);
  moveBackward();
  delay(2000);
  stopDrive();
  moveForward();
  delay(1000);
  stopDrive();
  delay(50000);
  stopDrive();
  moveForward();
  delay(FORWARD_MS);
  stopDrive();
  delay(STOP_SETTLE_MS);

  //3) Spin circle.
  spinCircle();
  delay(STOP_SETTLE_MS);

  // 4) Straighten and move backward for 13000ms.
  setSteeringStraightWithStops();
  delay(1000);
  moveBackward();
  delay(RETURN_BACKWARD_MS);
  stopDrive();
  delay(STOP_SETTLE_MS);

  //5) Turn to 90 and move right for 5600ms.
  setSteeringRight90WithStops();
  delay(1000);
  shiftRightReturnAdjusted();
  delay(RETURN_RIGHT_MS);
  stopDrive();
  delay(STOP_SETTLE_MS);

  //6) Straighten and final stop.
  setSteeringStraightWithStops();
  stopDrive();
}

void loop() {
  stopDrive();
  delay(1000);
}







