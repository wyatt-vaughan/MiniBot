# MiniBot

A swarm robotics platform built around autonomous chess. The system runs 34 small
differential-drive robots across an 8x8 board, each independently localized and
coordinated by a central Python application running on a Raspberry Pi.

The design is general-purpose — chess is the primary use case, but the platform
works for any scenario requiring coordinated movement of many small autonomous
robots in a bounded space.

Per-bot hardware cost is approximately $4 excluding PCB.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Hardware](#hardware)
3. [Communication Pathways](#communication-pathways)
4. [Coordinator](#coordinator)
5. [Server Firmware](#server-firmware)
6. [MiniBot Firmware](#minibot-firmware)
7. [Localization](#localization)
8. [Critical Objects](#critical-objects)
9. [PCB and CAD](#pcb-and-cad)
10. [Build and Setup](#build-and-setup)

---

## System Architecture

The system has three layers:

**Coordinator** is a Python/PyQt6 application running on a Raspberry Pi. It holds
board state, runs path planning, drives the GUI, and sends motion commands to the
Server over USB serial.

**Server** is an ESP32 on the motherboard. It acts as a wireless gateway: it receives
CSV commands from the Coordinator over USB serial, translates them to binary ESP-NOW
messages, and broadcasts them to the bots. It also drives the electromagnets used for
localization and collects ACK/NACK responses back up to the Coordinator.

**MiniBots** are the robots themselves. Each runs an ESP32-C3 with five concurrent
FreeRTOS tasks handling motion control, wireless communication, position estimation,
and battery management entirely onboard.

---

## Hardware

### MiniBot

| Component | Details |
|-----------|---------|
| MCU | ESP32-C3 (RISC-V, 80 MHz, 2.4 GHz WiFi) |
| Motors | 2x PMO8-2 miniature stepper motors |
| Motor drivers | STSPIN220 (1/128 microstepping mode) |
| Magnetometer | MMC5633NJL (I2C) |
| Power | 150 mAh LiPo |
| Chassis | Fully 3D printed; 2x M2x5mm bolts for motors, 2x 9mm ID o-rings for tires |
| PCB | Custom; fits in the base of the chassis |

Physical constants: wheel radius 5.25 mm, wheel spacing 23.4 mm, 160 microsteps/rev.

### Server / Motherboard

| Component | Details |
|-----------|---------|
| MCU | ESP32 (Xtensa dual-core, 240 MHz) |
| Electromagnets | 3x, positioned at fixed known coordinates on the board |
| Interface | USB serial to Raspberry Pi (921600 baud) |
| WiFi | Soft AP, SSID `ChessBot-Server`, channel 6, IP `192.168.4.1` |

The electromagnets fire in a timed sequence that the MiniBots detect with their
onboard magnetometers. This is the primary localization mechanism.

---

## Communication Pathways

```
Coordinator (Raspberry Pi)
    |
    |  USB Serial -- CSV protocol, 921600 baud, newline-terminated
    |  Format: >type,args
    |
Server ESP32 (Motherboard)
    |
    |  ESP-NOW -- 2.4 GHz, channel 6, broadcast + unicast
    |  Binary packed structs
    |
MiniBot #1 ... MiniBot #34 (ESP32-C3)
```

### CSV Protocol (Host to Server)

All messages are prefixed with `>`. The Server parses these and forwards the
appropriate ESP-NOW message to the targeted bot(s).

| Type | Direction | Format |
|------|-----------|--------|
| 0 | Host to ESP32 | `>0,{id},{mode},{duty1},{duty2}` -- motor test |
| 1 | Host to ESP32 | `>1,{id},{x},{y},{theta_rad},{duration_ms}` -- position command |
| 2 | Host to ESP32 | `>2,{id}` -- position request |
| 3 | ESP32 to Host | `>3,{id},{x},{y},{theta_rad},{timestamp_ms},{battery_v}` -- ACK |
| 4 | ESP32 to Host | `>4,{id},{err_type},{timestamp_ms}` -- NACK |
| 5 | Host to ESP32 | `>5,{id}` -- mag field request |
| 6 | ESP32 to Host | `>6,{id},{bx},{by},{bz},{timestamp_ms}` -- mag field response |
| 7 | Host to ESP32 | `>7` -- sync broadcast |
| 254 | Host to ESP32 | `>254,{0 or 1}` -- electromagnet enable/disable |
| 255 | Both | `>255` -- ping/pong |

Theta is transmitted in radians on the wire. The Coordinator converts internally;
all application-level code uses degrees.

### ESP-NOW Protocol (Server to MiniBots)

Binary packed structs defined in `ESPNowMessages.h` (Server) and
`messages_espnow.h` (MiniBot). Key message types: `PositionCommand`,
`MotTestCommand`, `PositionRequest`, `PosSync`, `AckMessage`, `NackMessage`,
`MagneticFieldRequest`, `MagneticFieldResponse`.

---

## Coordinator

Source: `firmware/MiniBot_Coordinator/`

Entry point is `main.py`, which starts a PyQt6 `MainWindow`. All system constants
live in `config.py`, organized into classes:

| Class | Contents |
|-------|---------|
| `BOARD` | Physical geometry: 400x400 mm playing area, 50 mm squares, border margins |
| `PIECES` | 34 piece IDs (0x01-0x11 white, 0x12-0x22 black), home positions, rank assignments |
| `COMM` | Serial protocol constants (921600 baud, CSV prefix `>`) |
| `GUI` | Colors, fonts, window dimensions |
| `PLANNING` | Path planner registry |
| `CHESS` | FEN string, rules engine adapter hooks |
| `SIMULATOR` | Software-in-the-loop config (80 mm/s nominal speed, 50 ms ticks) |

### GUI (gui/)

`ChessBoardWidget` renders all 34 pieces with live position and orientation updates.
The right panel contains tabbed controls: Path Planning, Debug, System Control, and
Position Tracker. Board clicks and manual inputs flow through the Path Planning tab
into the active planner, which produces a list of `MoveCommand` objects. These are
either sent to the `SerialHandler` (hardware) or the `Simulator` (software-in-the-loop).

### Path Planning (planning/)

Two planners derive from `BasePlanner`:

- `DirectPlanner`: All pieces move simultaneously to their targets. Every
  `MoveCommand` gets `sequence_num=0`.
- `QueuedPlanner`: Pieces move one at a time in ascending ID order. Each command
  gets an incrementing `sequence_num` that the dispatcher uses for ordering.

Both accept an optional `validator(piece_id, x, y) -> bool` callback for chess
legality checks.

### Serial Handler (comms/)

`SerialHandler` runs a background `QThread` (`_SerialWorker`) for non-blocking I/O.
Outgoing commands are queued with backpressure: stale commands are dropped if the
queue exceeds its depth limit. Incoming lines are parsed by `protocol.py` into
`ParsedMessage` dataclasses.

Qt signals emitted: `position_received`, `ack_received`, `error_received`,
`mag_field_received`, `connection_changed`.

### Simulator (simulation/)

`MotionSimulator` is a drop-in replacement for the serial link. It runs on a 50 ms
`QTimer`, simulates two-phase motion (rotate to face target, then translate),
enforces board boundaries, and detects collisions (pieces within 2 radii + margin
are blocked for that tick). Emits the same Qt signals as the serial handler so the
rest of the application is unaware of the difference.

---

## Server Firmware

Source: `firmware/MiniBot_Server/`

Built with PlatformIO. All FreeRTOS tasks are created in `main.cpp`; configuration
constants are in `include/config.h`.

### FreeRTOS Tasks

| Task | Core | Priority | Main Loop |
|------|------|----------|-----------|
| `CommunicatorTask` | 0 | 3 | Poll command queue from serial, ESP-NOW broadcast to bots, collect ACK/NACK/mag field responses into response queues |
| `SerialTask` | 0 | 3 | Parse incoming USB serial CSV, enqueue structured commands |
| `ElectromagnetTask` | 1 | 4 | Drive 3 electromagnets in a repeating 100 ms frame (7 ms forward + 7 ms reverse per slot) for synchronized localization |
| `GUITask` | 0 | 2 | Serve web interface for position tracking and manual control (conditional on `ENABLE_WEB_GUI`) |
| `JoystickTask` | 1 | 2 | Read joystick input and send real-time steering commands (conditional on `ENABLE_JOYSTICK_MODE`) |
| `LEDStatusTask` | 0 | 1 | Status LED indicator |

---

## MiniBot Firmware

Source: `firmware/MiniBot_StepperClient/`

Built with PlatformIO. Device ID is read from NVS at boot. CPU runs at 80 MHz for
power efficiency. All tasks are created in `main.cpp`; configuration constants are
in `include/config.h`.

Note: Status LED task is disabled on v3 boards and left commented out for future use.

### FreeRTOS Tasks

| Task | Priority | Main Loop |
|------|----------|-----------|
| `KinematicsController` | 5 | Dequeue `MotionCommand`, compute inverse kinematics (Cartesian to wheel velocities), drive steppers via ESP32 RMT peripheral; also handles `MotorTestQueue` for direct velocity commands |
| `EspNowCommunicator` | 3 | Receive ESP-NOW messages, dispatch to `MotionQueue` or `MotorTestQueue`, send `AckMessage` or `NackMessage` back to server; manages radio duty cycling for power savings |
| `PositionEstimator_Sensor` | 4 | Read MMC5633 magnetometer at 2 kHz, detect sync frame start (3 short pulses), timestamp samples per electromagnet slot, post complete `EmagFrameData` to internal queue |
| `PositionEstimator_Calc` | 2 | Dequeue `EmagFrameData`, trilaterate position and orientation from 2-3 electromagnet readings, apply confidence-weighted low-pass filter, call `robot.setTruePose()` |
| `BatteryMonitor` | 1 | ADC read battery voltage every 500 ms (20-sample running average, 1.84x divider ratio), update robot state; if voltage drops below 3.2 V, send `ERR_LOW_BATTERY` and suspend all other tasks |

The `KinematicsController` uses the ESP32 RMT peripheral to generate stepper pulse
trains at a constant frequency without blocking the CPU. The STSPIN220 drivers are
hardwired to 1/128 microstepping (160 microsteps/rev at the motor's 20 full steps/rev).

Inter-task communication uses FreeRTOS queues wrapped in `MotionQueue` and
`MotorTestQueue` helper modules.

---

## Localization

The Server's `ElectromagnetTask` fires 3 electromagnets in a repeating 100 ms frame.
Each slot is 7 ms forward + 7 ms reverse, with a known timing offset between slots.
The Server also broadcasts a `PosSync` message so each MiniBot knows when to expect
the frame.

On each MiniBot, `PositionEstimator_SensorTask` reads the MMC5633 magnetometer
continuously at 2 kHz. It detects the frame start via 3 short sync pulses, then
timestamps each subsequent sample relative to that start. Samples are assigned to
their electromagnet slot by timing alone.

Once a complete frame is collected, `PositionEstimator_CalcTask` performs
trilateration using the field strength readings from 2-3 electromagnets and their
known board positions. This yields an (x, y, orientation) estimate with a confidence
score. The confidence score is used to weight a low-pass filter before the result is
committed to the robot's true pose via `robot.setTruePose()`.

---

## Critical Objects

### Coordinator

| Object | Source | Purpose |
|--------|--------|---------|
| `Piece` | `models/piece.py` | One robot: ID, color, rank, position (mm), orientation (deg), battery voltage, capture/staged flags, last update timestamp |
| `BoardState` | `models/piece.py` | Container for all 34 `Piece` objects; home positions; accessors by ID, color, or active state |
| `MoveCommand` | `planning/base_planner.py` | Planner output: `piece_id`, `target_x_mm`, `target_y_mm`, `target_theta`, `duration_ms`, `sequence_num` |
| `ParsedMessage` | `comms/protocol.py` | Structured representation of one incoming serial line: type, position, battery, timestamp, error code |

### Server

| Object | Source | Purpose |
|--------|--------|---------|
| `PositionCommand` | `ESPNowMessages.h` | ESP-NOW command to a bot: target ID, x/y/theta, move duration |
| `AckMessage` | `ESPNowMessages.h` | Response from a bot: ID, x/y/theta, battery voltage, timestamp |
| `MotTestCommand` | `ESPNowMessages.h` | Direct motor velocity command for testing |
| `PosSync` | `ESPNowMessages.h` | Sync broadcast with electromagnet frame timing |

### MiniBot

| Object | Source | Purpose |
|--------|--------|---------|
| `Robot` | `robot.h` | Central state: position, orientation, true pose estimate, battery voltage, system status; shared by all tasks |
| `StepperDriver` | `stepper.h` | One wheel: step/dir GPIO control, direction, RMT channel, microstepping mode |
| `MotionCommand` | `messages_ipc.h` | IPC struct passed through `MotionQueue`: target x/y/theta + duration |
| `MotorTestRequest` | `messages_ipc.h` | IPC struct passed through `MotorTestQueue`: left/right wheel velocities (rad/s) |
| `EmagFrameData` | `position_estimator.h` | One complete localization frame: magnetic field samples for each electromagnet slot + background |
| `CalculatedPosition` | `position_estimator.h` | Trilateration result: x, y, orientation, confidence score |

---

## PCB and CAD

Current board designs are in `pcbs/`:

- `pcbs/MiniBot_MainBoard/` -- MiniBot PCB (ESP32-C3, stepper drivers, magnetometer, LiPo charger)
- `pcbs/Server_Motherboard/` -- Motherboard (ESP32, electromagnet drivers, connectors)

Gerber files are in the `GERBERS/` subdirectory of each board folder.

CAD (OnShape):
https://cad.onshape.com/documents/4f8eaef75458146767928ab5/w/f159d6d65b9091531e1ead34/e/13b51722310f27710438c727

---

## Build and Setup

### MiniBot Firmware and Server Firmware

Both use PlatformIO. Open the respective folder in VS Code with the PlatformIO
extension installed, or use the CLI:

```bash
# Build and upload (from firmware/MiniBot_StepperClient or firmware/MiniBot_Server)
pio run --target upload
```

Before flashing a MiniBot, write its device ID to NVS using the provisioning
utility or set it directly via the serial console.

### Coordinator

Requires Python 3.10+ and PyQt6.

```bash
cd firmware/MiniBot_Coordinator
pip install pyqt6 pyserial
python main.py
```

Connect the Server ESP32 over USB before launching. Select the serial port from
the System Control tab, and the Coordinator will begin receiving position updates
as bots are powered on.