# MiniHexa - Hiwonder Hexapod Robot (ESP32)

Arduino programming examples for the **Hiwonder MiniHexa** hexapod robot kit, updated for **ESP32 Arduino Core 3.x** compatibility.

> Forked from [Hiwonder/MiniHexa](https://github.com/Hiwonder/MiniHexa) with English translations and ESP32 3.x API updates.

---

## Hardware

| Component | Specification |
|---|---|
| **Main Controller** | ESP32 (Xtensa dual-core) |
| **Flash** | 4 MB |
| **Servos** | 18x serial bus servos (6 legs x 3 joints) |
| **IMU** | QMI8658 (6-axis accelerometer + gyroscope) |
| **Ultrasonic Sensor** | With RGB LED indicators |
| **Communication** | UART, BLE, Wi-Fi |
| **Optional Modules** | ESP32-S3 AI Vision, ASR Voice Recognition, WonderLLM AI, LED Matrix |

---

## Arduino IDE Setup

### 1. Install ESP32 Board Package

1. Open **Arduino IDE** > **File** > **Preferences**
2. In **Additional Board Manager URLs**, add:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
3. Go to **Tools** > **Board** > **Boards Manager**
4. Search for **esp32** and install version **3.3.7** (or latest 3.x)

### 2. Board Settings

| Setting | Value |
|---|---|
| **Board** | `ESP32 Dev Module` |
| **Partition Scheme** | `No OTA (2MB APP / 2MB SPIFFS)` |
| **Upload Speed** | `921600` |
| **Flash Frequency** | `80MHz` |
| **Flash Size** | `4MB (32Mb)` |
| **Core Debug Level** | `None` (or `Warn` for debugging) |
| **PSRAM** | `Disabled` |

> **The partition scheme is critical.** The BLE library in ESP32 Core 3.x is significantly larger than 2.x. The default partition (`1.2MB APP`) will fail with "Sketch too big" errors. You **must** select `No OTA (2MB APP / 2MB SPIFFS)` under **Tools > Partition Scheme**.

### 3. Required Libraries

Install these via **Sketch** > **Include Library** > **Manage Libraries**:

- **SensorLib** (by Lewis He) - for QMI8658 IMU sensor
- **ESP32 BLE Arduino** (included with ESP32 board package)

The robot's kinematics and hardware abstraction libraries are included directly in each example folder (no separate installation needed).

### 4. For AI Vision Projects (Section 4.4 / 4.7)

These projects use an **ESP32-S3** camera module in addition to the main ESP32 board. When uploading to the ESP32-S3 vision module:

| Setting | Value |
|---|---|
| **Board** | `ESP32-S3 Dev Module` |
| **PSRAM** | `OPI PSRAM` |
| **Partition Scheme** | `Huge APP (3MB No OTA / 1MB SPIFFS)` |

### 5. For AI Voice Projects (Section 4.5)

These projects require the **ASR (Automatic Speech Recognition) module** connected via I2C. No additional Arduino libraries are needed beyond what's included in the example folders.

### 6. For WonderLLM Projects (Section 4.6 / 4.7)

These projects require the **WonderLLM AI module**. The module communicates over I2C and needs network connectivity (Wi-Fi) for cloud LLM inference. Allow ~18 seconds after power-on for the module to complete network setup before sending commands.

---

## Project Structure

```
arduino-programming-projects/
|
+-- 4.1-arduino-ide-setup-and-calibration/   # Remote control (BLE + Wi-Fi + UART)
|
+-- 4.2-motion-control-basics/               # Movement fundamentals
|   +-- 5.2-omnidirectional-movement/        #   8-direction + rotation movement
|   +-- 5.3-turning-left-and-right/          #   Arc turning
|   +-- 5.4-speed-adjustment/                #   Speed control demo
|   +-- 5.5-gait-parameter-tuning/           #   Step count, duration, timing
|   +-- 5.6-pose-adjustment/                 #   Body position + orientation (6DOF)
|
+-- 4.3-development-projects/                # Intermediate projects
|   +-- 6.1-action-groups/                   #   Pre-recorded action sequences
|   +-- 6.2-voice-control/                   #   Sound-triggered movement
|   +-- 6.3-ultrasonic-ranging/              #   Distance measurement + RGB feedback
|   +-- 6.4-obstacle-avoidance/              #   Ultrasonic obstacle avoidance
|   +-- 6.5-auto-follow/                     #   Object following
|   +-- 6.6-self-balancing/                  #   IMU-based self-balancing
|   +-- 6.7-led-matrix-display/              #   Scrolling text on LED matrix
|   +-- 6.8-ultrasonic-range-display/        #   Distance shown on LED matrix
|   +-- 6.9-touch-control/                   #   Touch sensor triggered movement
|   +-- 6.10-ir-obstacle-avoidance/          #   IR sensor obstacle avoidance
|   +-- 6.11-fall-prevention/                #   Edge detection with IR sensors
|
+-- 4.4-ai-vision-projects/                  # Camera-based AI (requires ESP32-S3)
|   +-- 7.2-color-detection/                 #   Detect red/green/blue objects
|   +-- 7.3-color-tracking/                  #   Track green objects with yaw
|   +-- 7.4-line-following/                  #   Follow red line on ground
|   +-- 7.5-face-detection/                  #   React to detected faces
|
+-- 4.5-ai-voice-projects/                   # ASR voice recognition
|   +-- 8.3-ultrasonic-distance-alarm/       #   Voice alarm when object too close
|   +-- 8.4-human-robot-interaction/         #   Voice command responses
|   +-- 8.5-voice-control/                   #   Full voice-controlled movement
|
+-- 4.6-ai-llm-applications/                 # WonderLLM AI integration
|
+-- 4.7-ai-llm-offline-projects/             # Offline AI vision + LLM
    +-- 01-image-streaming/                  #   Camera web server
    +-- 02-color-detection/                  #   Color detection (offline)
    +-- 03-color-tracking/                   #   Color tracking (offline)
    +-- 04-line-following/                   #   Line following (offline)
    +-- 05-face-detection/                   #   Face detection (offline)
```

Each example folder contains all necessary source files (`.ino`, `.cpp`, `.h`) so it can be opened and compiled independently.

---

## Quick Start

1. Open any example `.ino` file in Arduino IDE (e.g., `4.2-motion-control-basics/5.2-omnidirectional-movement/omnidirectional_movement/omnidirectional_movement.ino`)
2. Select **Tools** > **Board** > **ESP32 Dev Module**
3. Set **Tools** > **Partition Scheme** > **No OTA (2MB APP / 2MB SPIFFS)**
4. Select the correct **Port**
5. Click **Upload**

---

## ESP32 Core 3.x Migration Notes

This fork has been updated from ESP32 Arduino Core 2.x to **3.x**. Key API changes applied:

| Change | Old (Core 2.x) | New (Core 3.x) |
|---|---|---|
| LEDC PWM | `ledcSetup()` + `ledcAttachPin()` | `ledcAttach(pin, freq, resolution)` |
| ADC Width | `ADC_WIDTH_12Bit` | `ADC_WIDTH_BIT_12` |
| IMU Config | `configAccelerometer(..., true)` | `configAccelerometer(...)` (4th param removed) |
| IMU Config | `configGyroscope(..., true)` | `configGyroscope(...)` (4th param removed) |
| BLE Strings | `std::string` | `String` (Arduino) |
| BLE setValue | `setValue(std::string)` | `setValue(String)` |

---

## Troubleshooting

### "Sketch too big" / "text section exceeds available space"
Change **Tools** > **Partition Scheme** to **No OTA (2MB APP / 2MB SPIFFS)**.

### BLE `#pragma message` warnings about BR/EDR
These are harmless informational notes from the ESP32 Bluetooth stack, not errors. They appear on every build that includes BLE and can be safely ignored.

### Servo calibration
Use the `4.1-arduino-ide-setup-and-calibration/remote/` example with the Hiwonder app to calibrate servo offsets before running other examples.

---

## Links

- [Hiwonder MiniHexa Wiki](https://wiki.hiwonder.com/projects/miniHexa/en/latest/)
- [ESP32 Arduino Core Documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/)
- [Original Repository (Hiwonder)](https://github.com/Hiwonder/MiniHexa)
