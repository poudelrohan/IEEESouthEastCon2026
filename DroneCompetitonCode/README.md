# IEEE SoutheastCon 2026 — Hardware Competition

**3rd Place** among 40+ universities at IEEE SoutheastCon 2026, Huntsville, Alabama.

This repository contains the drone subsystem code for the autonomous hardware competition. An ESP32 controls a Tello EDU drone over WiFi/UDP, coordinating with an Arduino Mega ground robot via serial communication to complete a multi-phase autonomous mission.

## Demo

<!-- Add demo GIF/video here -->
> Demo coming soon

![Competition Demo](https://via.placeholder.com/800x400?text=Demo+Coming+Soon)

## Competition Overview

The hardware competition required teams to build an autonomous system combining a ground robot and a micro UAV. The robot and drone must coordinate without human intervention to navigate an obstacle course, with the drone performing precision takeoff, flight, and landing sequences on top of the moving robot.

**Our result:** 3rd place out of 40+ competing university teams.

## System Architecture

```
┌──────────────┐   Serial (UART)   ┌──────────────┐   WiFi/UDP   ┌──────────────┐
│ Arduino Mega │ ◄──────────────► │    ESP32     │ ◄──────────► │  Tello EDU   │
│ Ground Robot │                   │ Flight Ctrl  │              │    Drone     │
│              │                   │              │              │              │
│ • Motors     │                   │ • WiFi Mgmt  │              │ • Camera     │
│ • Navigation │                   │ • UDP Cmds   │              │ • Mission Pad│
│ • Pad Mount  │                   │ • State Parse│              │ • IMU/Baro   │
└──────────────┘                   └──────────────┘              └──────────────┘
```

## Flight Sequence

### Flight 1: Crater Overflight (T=29s)
1. Takeoff → Forward 40cm (over crater) → 180° turn → Forward 30cm → Land

### Flight 2: Precision Landing on Robot (T=50s+)
1. Takeoff → Reposition → Find mission pad on robot via vision
2. Vision-guided descent: step down from 60cm → 25cm using `go` command
3. RC fine-tuning: manual joystick correction within 8cm zone
4. Precision land on robot platform

## Tech Stack

| Component | Technology |
|-----------|-----------|
| **Microcontroller** | ESP32 DevKit V1 |
| **Drone** | DJI Tello EDU (SDK 2.0) |
| **Communication** | WiFi (UDP ports 8889/8890) + Serial UART |
| **Language** | C++ / Arduino |
| **Sensors** | Mission Pad vision detection, barometer, battery telemetry |
| **Ground Robot** | Arduino Mega (motor control, navigation) |

## Key Features

- **Autonomous Flight** — Pre-programmed multi-phase flight sequences with no human input
- **Vision-Guided Landing** — Mission pad detection for precision drone-to-robot docking
- **Height-Stepping Descent** — Gradual approach algorithm prevents overshooting (60→50→40→30→25cm)
- **RC Fine-Tuning** — Joystick simulation for sub-centimeter final positioning
- **Serial Coordination** — Phase-based handshake protocol between ground robot and drone controller
- **Adaptive Recovery** — Retry logic and emergency stop handling for competition reliability
- **Real-Time Telemetry** — UDP state stream parsing (battery, position, mission pad coordinates)

## Project Structure

```
IEEESouthEastCon2026/
├── Automatic17/              # Final production code used in competition
├── Automatic13-16/           # Earlier iteration versions
├── ESP32_PhaseControl/       # Serial command-based coordination architecture
├── DroneMissionPadCheck/     # Mission pad detection testing
├── DroneSimplified10-13/     # Simplified flight sequence iterations
├── AutomaticButton*/         # Manual phase-trigger variants
├── Hover30s/                 # Hover stability testing
├── Shortcut/                 # Simplified flight sequences
└── CONTEXT_FOR_NEW_CHAT.md   # Mission timeline and coordination specs
```

> `Automatic17.ino` is the final competition version (620+ lines).

## Getting Started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) 2.0+
- ESP32 board package installed in Arduino IDE
- DJI Tello EDU drone
- WiFi connection to Tello's access point

### Hardware Setup

1. Connect ESP32 to Arduino Mega via Serial2 (UART)
2. Power both from shared battery with single power switch
3. Mount Tello on robot platform with mission pad visible

### Upload Code

1. Open `Automatic17/Automatic17.ino` in Arduino IDE
2. Select **ESP32 Dev Module** as the board
3. Update WiFi credentials for your Tello drone
4. Upload to ESP32

## Team

Built by the **Bethune-Cookman University IEEE Robotics Club**.

## License

This project is open source and available under the [MIT License](LICENSE).
