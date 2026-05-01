# MiniBot Server Firmware

ESP32 firmware for the motherboard. Acts as a wireless gateway between the
Coordinator (USB serial) and the MiniBots (ESP-NOW). Also drives the electromagnets
used for localization.

Built with PlatformIO. All FreeRTOS tasks are created in `src/main.cpp`;
configuration constants are in `include/config.h`.

---

## FreeRTOS Tasks

| Task | Core | Priority | Main Loop |
|------|------|----------|-----------|
| `ElectromagnetTask` | 1 | 4 | Drive electromagnets in a repeating timed frame for synchronized localization |
| `CommunicatorTask` | 0 | 3 | Poll command queue from serial, ESP-NOW broadcast to bots, collect ACK/NACK/mag field responses |
| `SerialTask` | 0 | 3 | Parse incoming USB serial CSV, enqueue structured commands |
| `GUITask` | 0 | 2 | Serve web interface for position tracking and manual control (conditional on `ENABLE_WEB_GUI`) |
| `JoystickTask` | 1 | 2 | Read joystick input and send real-time steering commands (conditional on `ENABLE_JOYSTICK_MODE`) |
| `LEDStatusTask` | 0 | 1 | Status LED indicator |

---

## Key Objects

| Object | Source | Purpose |
|--------|--------|---------|
| `PositionCommand` | `ESPNowMessages.h` | ESP-NOW command to a bot: target ID, x/y/theta, move duration |
| `AckMessage` | `ESPNowMessages.h` | Response from a bot: ID, current x/y/theta, battery voltage, timestamp |
| `MotTestCommand` | `ESPNowMessages.h` | Direct motor velocity command for testing |
| `PosSyncCommand` | `ESPNowMessages.h` | Broadcast to prepare bots for an upcoming localization frame |
| `PosSync` | `ESPNowMessages.h` | Pulse sent during sync burst; carries time-to-next-frame |

---

## Build

```bash
pio run --target upload
```
