#include <DynamixelShield.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Print debug to Serial (USB) at 1M baud — same as Dynamixel.
// You'll see some Dynamixel garbage mixed in, but debug prints will be readable.
#define DEBUG_SERIAL Serial

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
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Crater navigation constants
const uint32_t CRATER_BLIND_MS   = 15000;  // 15s blind strafing (~50% of lap)
const uint32_t CRATER_POLL_MS    = 500;    // poll heading every 500ms
const float    HEADING_OVERLAP   = 5.0;    // degrees PAST entry before stopping (no underlap)
const float    HEADING_TOLERANCE = 10.0;   // degrees window for match detection
const uint32_t CRATER_TIMEOUT_MS = 30000;  // max polling time after blind phase

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

// Single compass read. Returns 0.0 to 360.0 degrees.
float readHeadingSingle() {
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  if (heading < 0) heading += 2.0 * M_PI;
  heading = heading * 180.0 / M_PI;
  delay(250);
  return heading;
}

// Filtered compass read: takes 5 readings, bins to nearest 5°, returns mode.
// More reliable than Richie's 1° bins where mode often fails.
float readHeading() {
  const int N = 5;
  float readings[N];
  int binned[N];

  for (int i = 0; i < N; i++) {
    readings[i] = readHeadingSingle();
    binned[i] = ((int)(readings[i] + 2.5)) / 5 * 5;  // round to nearest 5°
  }

  // Find mode of binned values
  int modeValue = binned[0];
  int maxCount = 1;
  for (int i = 0; i < N; i++) {
    int count = 0;
    for (int j = 0; j < N; j++) {
      if (binned[j] == binned[i]) count++;
    }
    if (count > maxCount) {
      maxCount = count;
      modeValue = binned[i];
    }
  }

  // Average the raw readings that fell into the mode bin for precision
  float sum = 0;
  int sumCount = 0;
  for (int i = 0; i < N; i++) {
    if (binned[i] == modeValue) {
      sum += readings[i];
      sumCount++;
    }
  }
  return sum / sumCount;
}

// Wraparound-safe heading difference. Returns shortest angular distance (0-180).
float headingDiff(float a, float b) {
  float diff = fabs(a - b);
  if (diff > 180.0) diff = 360.0 - diff;
  return diff;
}

// Returns true if current heading has PASSED the target heading
// in the anticlockwise direction (no underlap allowed).
// target = entryHeading - HEADING_OVERLAP (shifted to guarantee overlap)
bool headingMatchesEntry(float current, float target, float tolerance) {
  return headingDiff(current, target) <= tolerance;
}

// Just the circle drive commands — no steering, no delay, no stop.
void spinCircleDrive() {
  setForwardForDrive(drive_A, CIRCLE_SPD_A);
  setBackwardForDrive(drive_B, CIRCLE_SPD_B);
  setForwardForDrive(drive_C, CIRCLE_SPD_C);
  setBackwardForDrive(drive_D, CIRCLE_SPD_D);
}

// Circle turn = shift-right pattern with CIRCLE angles and CIRCLE speeds.
void spinCircle() {
  setSteeringCircleWithStops();
  spinCircleDrive();
  delay(CIRCLE_MS);
  stopDrive();
}

// Crater navigation using magnetometer for lap completion detection.
// 1) Record entry heading while stopped (clean read, no motor noise)
// 2) Shift target by -5° so robot must OVERLAP (no underlap)
// 3) Strafe in circle for 15s (blind phase)
// 4) Poll filtered heading every 500ms until it matches target
// 5) Stop drive — robot is slightly past entry position (good)
void craterNavigate() {
  // Read heading while stopped — cleanest possible reading
  float entryHeading = readHeading();

  // Shift target so robot goes 5° PAST the entry point (anticlockwise)
  float targetHeading = entryHeading - HEADING_OVERLAP;
  if (targetHeading < 0) targetHeading += 360.0;

  DEBUG_SERIAL.print("Entry heading: ");
  DEBUG_SERIAL.println(entryHeading);
  DEBUG_SERIAL.print("Target heading (with overlap): ");
  DEBUG_SERIAL.println(targetHeading);

  // Phase 1: Set circle steering and start driving (blind for 15s)
  setSteeringCircleWithStops();
  spinCircleDrive();
  delay(CRATER_BLIND_MS);

  // Phase 2: Keep driving, poll magnetometer until heading matches target
  DEBUG_SERIAL.println("Blind phase done. Polling magnetometer...");
  uint32_t pollStart = millis();
  while (millis() - pollStart < CRATER_TIMEOUT_MS) {
    float currentHeading = readHeading();
    float diff = headingDiff(currentHeading, targetHeading);

    DEBUG_SERIAL.print("Current: ");
    DEBUG_SERIAL.print(currentHeading);
    DEBUG_SERIAL.print(" | Target: ");
    DEBUG_SERIAL.print(targetHeading);
    DEBUG_SERIAL.print(" | Diff: ");
    DEBUG_SERIAL.println(diff);

    if (headingMatchesEntry(currentHeading, targetHeading, HEADING_TOLERANCE)) {
      DEBUG_SERIAL.println("Heading matched — lap complete (overlapped)!");
      break;
    }
    delay(CRATER_POLL_MS);
  }

  // Phase 3: Stop
  stopDrive();
}

void setup() {
  dxl.begin(1000000);  // Opens Serial (USB) at 1M baud — also used for debug prints
  dxl.setPortProtocolVersion(dxl_protocol_version);

  Wire.begin();
  if (!mag.begin()) {
    DEBUG_SERIAL.println("HMC5883L not found! Check wiring.");
    while (1);
  }
  DEBUG_SERIAL.println("HMC5883L initialized.");

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
  delay(5000);
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
  delay(5000);
  stopDrive();
  moveForward();
  delay(FORWARD_MS);
  stopDrive();
  delay(STOP_SETTLE_MS);

  //3) Crater lap with magnetometer.
  craterNavigate();
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







