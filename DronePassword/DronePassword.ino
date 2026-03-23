#include <WiFi.h>
#include <WiFiUdp.h>

// Your current Tello WiFi credentials
const char* TELLO_WIFI = "TELLO-Juan's";
const char* TELLO_PASS = "southeast2026";

// NEW credentials you want to set
const char* NEW_SSID = "OLLET";              // Shows as TELLO-OLLET
const char* NEW_PASSWORD = "southeast2026"; // Same password as before

const char* TELLO_IP = "192.168.10.1";
const int CMD_PORT = 8889;

WiFiUDP udp;

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n══════════════════════════════════════");
    Serial.println("   TELLO WIFI PASSWORD SETUP");
    Serial.println("══════════════════════════════════════\n");
    
    // Connect to Tello
    Serial.print("Connecting to Tello...");
    WiFi.begin(TELLO_WIFI, TELLO_PASS);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFailed to connect!");
        Serial.println("Make sure Tello is ON and you're connecting to the right network.");
        while(1) delay(1000);
    }
    
    Serial.println(" Connected!\n");
    
    udp.begin(CMD_PORT);
    delay(1000);
    
    // Enter SDK mode
    Serial.println("Entering SDK mode...");
    udp.beginPacket(TELLO_IP, CMD_PORT);
    udp.print("command");
    udp.endPacket();
    delay(2000);
    
    // Read response
    if (udp.parsePacket()) {
        char buf[64];
        int len = udp.read(buf, 63);
        buf[len] = 0;
        Serial.printf("Response: %s\n\n", buf);
    }
    
    // Set new WiFi credentials
    Serial.println("══════════════════════════════════════");
    Serial.printf("Setting new WiFi:\n");
    Serial.printf("  SSID:     %s\n", NEW_SSID);
    Serial.printf("  Password: %s\n", NEW_PASSWORD);
    Serial.println("══════════════════════════════════════\n");
    
    char cmd[100];
    sprintf(cmd, "wifi %s %s", NEW_SSID, NEW_PASSWORD);
    
    Serial.printf("Sending: %s\n", cmd);
    udp.beginPacket(TELLO_IP, CMD_PORT);
    udp.print(cmd);
    udp.endPacket();
    
    delay(3000);
    
    // Read response
    if (udp.parsePacket()) {
        char buf[64];
        int len = udp.read(buf, 63);
        buf[len] = 0;
        Serial.printf("Response: %s\n\n", buf);
    }
    
    Serial.println("══════════════════════════════════════");
    Serial.println("DONE!");
    Serial.println("");
    Serial.println("Tello will now reboot with new WiFi settings.");
    Serial.println("");
    Serial.printf("To connect, use:\n");
    Serial.printf("  SSID:     %s\n", NEW_SSID);
    Serial.printf("  Password: %s\n", NEW_PASSWORD);
    Serial.println("");
    Serial.println("UPDATE YOUR FLIGHT CODE with new credentials!");
    Serial.println("══════════════════════════════════════");
}

void loop() {
    delay(1000);
}