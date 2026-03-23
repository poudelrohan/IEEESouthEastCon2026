#include <Wire.h>

void setup() {
  Serial.begin(9600);
  Wire.begin();
  delay(1000);
  Serial.println("Scanning I2C bus...");

  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at address 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);

      if (addr == 0x0D) Serial.println("  <-- QMC5883L chip");
      else if (addr == 0x1E) Serial.println("  <-- HMC5883L chip");
      else Serial.println();

      count++;
    }
  }

  if (count == 0) Serial.println("No I2C devices found! Check wiring.");
  else {
    Serial.print(count);
    Serial.println(" device(s) found.");
  }
}

void loop() {}