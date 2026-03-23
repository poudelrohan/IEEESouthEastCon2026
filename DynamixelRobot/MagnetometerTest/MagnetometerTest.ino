#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

const uint32_t BLIND_MS          = 5000;   // 5s blind phase (rotate freely)
const uint32_t POLL_MS           = 500;    // poll every 500ms
const float    HEADING_TOLERANCE = 10.0;   // degrees
const uint32_t TIMEOUT_MS        = 30000;  // max polling time

float readHeading() {
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading = heading * 180.0 / M_PI;
  if (heading < 0) heading += 360.0;
  return heading;
}

bool headingMatchesEntry(float current, float entry, float tolerance) {
  float diff = fabs(current - entry);
  if (diff > 180.0) diff = 360.0 - diff;
  return diff <= tolerance;
}

void setup() {
  Serial.begin(115200);
  delay(3000);  // 3s delay so you can open Serial Monitor after upload
  Serial.println();
  Serial.println("Booting...");

  Wire.begin();

  if (!mag.begin()) {
    Serial.println("ERROR: HMC5883L not found! Check wiring:");
    Serial.println("  SDA -> pin 20");
    Serial.println("  SCL -> pin 21");
    Serial.println("  VCC -> 5V or 3.3V");
    Serial.println("  GND -> GND");
    Serial.println("Halted. Reset to retry.");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("HMC5883L initialized OK.");
  Serial.println();
  Serial.println("== Magnetometer Crater Lap Test ==");
  Serial.println("Press 's' in Serial Monitor to start a test.");
  Serial.println("Then rotate the sensor slowly by hand.");
  Serial.println();
}

void loop() {
  // Wait for 's' command
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 's' || c == 'S') {
      runTest();
    }
  }
}

void runTest() {
  // Record entry heading
  float entryHeading = readHeading();
  Serial.println("----------------------------------");
  Serial.print("ENTRY HEADING: ");
  Serial.print(entryHeading, 1);
  Serial.println(" deg");
  Serial.println("----------------------------------");
  Serial.println();

  // Phase 1: Blind phase — just print heading for 5 seconds
  Serial.println(">> BLIND PHASE (5s) — rotate the sensor now!");
  uint32_t blindStart = millis();
  while (millis() - blindStart < BLIND_MS) {
    float h = readHeading();
    float diff = fabs(h - entryHeading);
    if (diff > 180.0) diff = 360.0 - diff;
    Serial.print("  [blind] heading: ");
    Serial.print(h, 1);
    Serial.print("  | distance from entry: ");
    Serial.print(diff, 1);
    Serial.println(" deg");
    delay(POLL_MS);
  }

  Serial.println();
  Serial.println(">> POLLING PHASE — keep rotating, waiting for lap...");
  Serial.print("   Looking for heading within ");
  Serial.print(HEADING_TOLERANCE, 1);
  Serial.print(" deg of ");
  Serial.print(entryHeading, 1);
  Serial.println(" deg");
  Serial.println();

  // Phase 2: Active polling — check for heading match
  uint32_t pollStart = millis();
  bool matched = false;
  while (millis() - pollStart < TIMEOUT_MS) {
    float currentHeading = readHeading();
    float diff = fabs(currentHeading - entryHeading);
    if (diff > 180.0) diff = 360.0 - diff;

    Serial.print("  [poll]  heading: ");
    Serial.print(currentHeading, 1);
    Serial.print("  | distance from entry: ");
    Serial.print(diff, 1);

    if (headingMatchesEntry(currentHeading, entryHeading, HEADING_TOLERANCE)) {
      Serial.println("  *** MATCH — LAP COMPLETE! ***");
      matched = true;
      break;
    } else {
      Serial.println("  (no match)");
    }
    delay(POLL_MS);
  }

  Serial.println();
  Serial.println("----------------------------------");
  if (matched) {
    float finalHeading = readHeading();
    float overlap = fabs(finalHeading - entryHeading);
    if (overlap > 180.0) overlap = 360.0 - overlap;
    Serial.println("RESULT: LAP DETECTED");
    Serial.print("  Entry heading:  ");
    Serial.print(entryHeading, 1);
    Serial.println(" deg");
    Serial.print("  Final heading:  ");
    Serial.print(finalHeading, 1);
    Serial.println(" deg");
    Serial.print("  Overlap:        ");
    Serial.print(overlap, 1);
    Serial.println(" deg");
    uint32_t totalTime = millis() - (pollStart - BLIND_MS);
    Serial.print("  Total time:     ");
    Serial.print(totalTime / 1000.0, 1);
    Serial.println(" s");
  } else {
    Serial.println("RESULT: TIMEOUT — no lap detected in 30s");
  }
  Serial.println("----------------------------------");
  Serial.println();
  Serial.println("Press 's' to run another test.");
  Serial.println();
}
