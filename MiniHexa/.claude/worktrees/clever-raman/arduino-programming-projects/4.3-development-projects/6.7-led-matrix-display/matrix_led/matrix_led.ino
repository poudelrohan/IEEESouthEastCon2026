#include "hiwonder_robot.h"
#include "hiwonder_sensor.h"
#include "WMMatrixLED.h"

// Create MiniHexa object
Robot minihexa;
// Initialize LED matrix module pins
WMMatrixLed matrix(14, 32);  // SCK / DIN pin numbers

void setup() {
  Serial.begin(115200);
  minihexa.begin();
  matrix.setBrightness(5); // Set brightness
  matrix.clearScreen(); // Clear screen
}
int x;
void loop() {
  const char* text = "Hiwonder";
  int textWidth = 6 * strlen(text); // ~6 pixels per character
  for (int x = 16; x > -textWidth; x--) {
    matrix.drawStr(x, 8, text);       // Start display
    delay(100);
  }
}
