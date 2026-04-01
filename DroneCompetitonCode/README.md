# IEEE SoutheastCon 2026 — Hardware Competition

**3rd Place** among 40+ universities at IEEE SoutheastCon 2026, Huntsville, Alabama.

This repository contains the drone subsystem code for the autonomous hardware competition. An ESP32 controls a **DJI Tello EDU** drone over WiFi/UDP, using its **mission pad** vision system for precision landing on a moving ground robot — all coordinated via serial communication with an Arduino Mega.

## Demo

<!-- Add demo GIF/video here -->
> Demo coming soon

![Competition Demo](https://via.placeholder.com/800x400?text=Demo+Coming+Soon)

## Competition Overview

The hardware competition required teams to build a fully autonomous system combining a ground robot and a micro UAV. Once the single power switch is flipped, the robot and drone must coordinate without any human intervention to navigate an obstacle course. The drone takes off from the robot, flies over a crater, returns, and precision-lands back on the robot platform — all timed to the robot's movement schedule.

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

### How the Tello EDU + Mission Pads Work

The **DJI Tello EDU** is a programmable micro drone controlled via the **Tello SDK 2.0** over WiFi. We send UDP commands (port 8889) and receive real-time state telemetry (port 8890). The key feature we rely on is **mission pad detection** — the Tello's downward camera recognizes special QR-code-like pads and reports the drone's X, Y, Z position relative to the pad center. This gives us centimeter-level positioning for precision landing.

- **Command port (8889):** Send flight commands (`takeoff`, `forward 40`, `go x y z speed mid`, `land`)
- **State port (8890):** Receive telemetry stream (`mid:` pad ID, `x:` `y:` `z:` position, `bat:` battery)
- **Mission pad detection range:** 30-120cm altitude
- **`go` command:** Tells the drone to fly to a specific X, Y, Z coordinate relative to a detected pad — this is the core of our precision landing

## Flight Sequence

The drone executes a single continuous flight, timed to the ground robot's autonomous movement:

```
T=0s     Power on. ESP32 connects to Tello WiFi, enters SDK mode, enables mission pads.
T=29s    Robot reaches position. DRONE TAKEOFF.
         → Forward 40cm (fly over crater zone)
         → CW 180° turn
         → Forward 30cm (return toward robot)
         → CW 180° turn (face original direction)
         → Find mission pad on robot
         → 5-step vision-guided descent
         → RC fine-tuning for final centering
         → Land on robot platform
```

### Precision Landing: The 5-Step Descent

The hardest part of the competition was landing back on the robot. We couldn't just drop down — the drone overshoots if you descend too fast, and the mission pad goes out of detection range below 30cm. Our solution was a **gradual 5-step descent using the `go` command**:

```
Step 1: go to pad center at 60cm
Step 2: go to pad center at 50cm
Step 3: go to pad center at 40cm
Step 4: go to pad center at 30cm
Step 5: go to pad center at 25cm
```

At each step, the drone re-centers itself over the pad before descending further. After the final step, **RC fine-tuning** kicks in — we simulate joystick inputs at 10 cm/s within a tight ±8cm zone until 5 consecutive readings confirm the drone is locked on center. Then we land.

## How the Code Evolved

The repo contains 17+ iterations showing how we went from basic hover tests to competition-grade autonomous landing. Here's the story:

### Phase 1: Basic Flight & Discovery (v10-v13)

| Version | What Changed | Key Discovery |
|---------|-------------|---------------|
| **v10** | Replaced manual RC centering with Tello's `go` command | RC joystick oscillates badly; `go` with internal PID is far more reliable |
| **v11** | Tested lowest altitude for mission pad detection | RC throttle stalls at ~48cm — Tello's height controller won't descend further via joystick |
| **v12** | Removed RC descent entirely, `go`-only approach | Two-step descent (50→25cm) works but overshoots on large drops |
| **v13** | Introduced 5-step gradual descent (60→50→40→30→25cm) | Camera tilts ~10° forward, compensated with `GO_X_OFFSET = -5` |

### Phase 2: Robot Coordination (v14-v16)

| Version | What Changed | Key Discovery |
|---------|-------------|---------------|
| **v14** | Added time-based sync with robot schedule (two-flight approach) | Absolute timestamps from power-on control takeoff/landing phases |
| **v15** | Fixed centering — center at current height first, then descend | Centering + descending simultaneously causes `go` command failures |
| **v16** | Updated for new robot schedule, added Y-offset compensation | Added `GO_Y_OFFSET = -5` and experimented with emergency landing (motors-off drop) |

### Phase 3: Competition Simplification (v17+)

| Version | What Changed | Key Discovery |
|---------|-------------|---------------|
| **v17** | Simplified to single continuous flight (no intermediate landing) | Eliminated two-flight complexity; reduced return distance to 30cm |
| **ButtonMAX** | Competition-day timing variant | Tuned for actual competition schedule |
| **ButtonMAXTwo** | Added 5s hover stabilization before pad detection | Letting the drone settle before scanning dramatically improves pad lock reliability |

### Testing & Validation Variants

| Sketch | Purpose |
|--------|---------|
| **Hover30s** | Basic hover stability test — does it stay in place? |
| **DroneMissionPadCheck** | Test which mission pads are detectable on different surfaces |
| **Shortcut** | Isolated landing test — skip the flight, just test descent + land |
| **ESP32_PhaseControl** | Alternative architecture: Mega sends serial commands to trigger phases instead of timing |

## Engineering Challenges We Solved

**1. RC vs. `go` Command**
Early versions used RC joystick simulation to center the drone over the pad. This oscillated — the drone would overshoot, correct, overshoot again. Switching to Tello's `go` command (which uses the drone's internal PID controller to fly to pad-relative coordinates) solved this completely.

**2. The 48cm Floor**
We discovered that RC throttle commands can't reliably descend below ~48cm — the Tello's internal height controller fights it. This forced us to use `go` commands for all descent, which turned out to be more accurate anyway.

**3. Overshoot on Large Drops**
A single `go` from 60cm to 25cm overshoots. The 5-step approach (60→50→40→30→25) lets the drone re-center at each height before going lower.

**4. Camera Tilt Drift**
The Tello's camera tilts ~10° forward, causing a consistent +5cm drift in X during `go` commands. We compensated with `GO_X_OFFSET = -5`. Combined with landing drift (+3-4cm), the Shortcut test sketch uses a total offset of -9.

**5. Timing Coordination**
The drone and robot share one battery and one power switch. The ESP32 takes ~17s to boot and connect to the Tello's WiFi. The robot starts moving immediately. All drone actions are timed to absolute timestamps from power-on to stay in sync with the robot's autonomous path.

## Tech Stack

| Component | Technology |
|-----------|-----------|
| **Microcontroller** | ESP32 DevKit V1 |
| **Drone** | DJI Tello EDU (SDK 2.0) |
| **Vision** | Tello mission pad detection (downward camera, QR-code-like pads) |
| **Communication** | WiFi UDP (cmd: 8889, state: 8890) + Serial UART to Mega |
| **Language** | C++ / Arduino |
| **Ground Robot** | Arduino Mega (motors, navigation, mission pad mount) |

## Project Structure

```
IEEESouthEastCon2026/
├── Automatic17/              # Final production code (620+ lines)
├── Automatic13-16/           # Progressive autonomous versions
├── AutomaticButton*/         # Competition-day timing variants
├── DroneSimplified10-13/     # Early iteration versions
├── ESP32_PhaseControl/       # Serial command coordination prototype
├── DroneMissionPadCheck/     # Mission pad detection testing
├── Hover30s/                 # Hover stability test
├── Shortcut, Shortcut17/    # Isolated landing tests
└── CONTEXT_FOR_NEW_CHAT.md   # Mission timeline and coordination specs
```

> **`Automatic17.ino`** is the final competition version. **`AutomaticButtonMAXTwo.ino`** is the last iteration with pre-landing hover stabilization.

## Getting Started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) 2.0+
- ESP32 board package installed in Arduino IDE
- DJI Tello EDU drone + mission pads
- WiFi connection to Tello's access point

### Hardware Setup

1. Connect ESP32 to Arduino Mega via Serial2 (GPIO16 RX, GPIO17 TX) with voltage divider on Mega TX
2. Power both from shared battery with single power switch
3. Mount Tello on robot platform with mission pad visible to downward camera

### Upload & Run

1. Open `Automatic17/Automatic17.ino` in Arduino IDE
2. Select **ESP32 Dev Module** as the board
3. Update `WIFI_NAME` and `WIFI_PASSWORD` for your Tello
4. Upload to ESP32
5. Flip the power switch — the drone connects, waits for the timed takeoff, and flies autonomously

## Team

Built by the **Bethune-Cookman University IEEE Robotics Club**.

## License

This project is open source and available under the [MIT License](LICENSE).
