# MiniBot StepperClient Firmware

ESP32-C3 firmware for each individual MiniBot. Handles motion control, wireless
communication, position estimation, and battery management via five concurrent
FreeRTOS tasks.

Built with PlatformIO. All tasks are created in `src/main.cpp`; configuration
constants are in `include/config.h`. Device ID is read from NVS at boot.

Note: Status LED task is disabled on v3 boards and left commented out for future use.

---

## FreeRTOS Tasks

| Task | Priority | Main Loop |
|------|----------|-----------|
| `KinematicsController` | 5 | Dequeue `MotionCommand`, compute inverse kinematics (Cartesian to wheel velocities), drive steppers via ESP32 RMT peripheral; also handles `MotorTestQueue` for direct velocity commands |
| `PositionEstimator_Sensor` | 4 | Read MMC5633 magnetometer at high rate, detect sync frame start, timestamp samples per electromagnet slot, post complete `EmagFrameData` to internal queue |
| `EspNowCommunicator` | 3 | Receive ESP-NOW messages, dispatch to `MotionQueue` or `MotorTestQueue`, send `AckMessage` or `NackMessage` back to server; manages radio duty cycling for power savings |
| `PositionEstimator_Calc` | 2 | Dequeue `EmagFrameData`, trilaterate position and orientation from electromagnet readings, apply confidence-weighted low-pass filter, call `robot.setTruePose()` |
| `BatteryMonitor` | 1 | Periodically read battery voltage, update robot state; suspend all other tasks if voltage drops to a critical level |

The `KinematicsController` uses the ESP32 RMT peripheral to generate stepper pulse
trains without blocking the CPU. Inter-task communication uses FreeRTOS queues
wrapped in `MotionQueue` and `MotorTestQueue` helper modules.

---

## Key Objects

| Object | Source | Purpose |
|--------|--------|---------|
| `Robot` | `robot.h` | Central state: position, orientation, true pose estimate, battery voltage; shared across all tasks |
| `StepperDriver` | `stepper.h` | One wheel: step/dir GPIO, direction, RMT channel, microstepping mode |
| `MotionCommand` | `messages_ipc.h` | IPC struct passed through `MotionQueue`: target x/y/theta + duration |
| `MotorTestRequest` | `messages_ipc.h` | IPC struct passed through `MotorTestQueue`: left/right wheel velocities |
| `EmagFrameData` | `position_estimator.h` | One complete localization frame: magnetometer samples per electromagnet slot + background |
| `CalculatedPosition` | `position_estimator.h` | Trilateration result: x, y, orientation, confidence score |

---

## Build

```bash
pio run --target upload
```

Before flashing, write the device ID to NVS using the provisioning utility or
set it directly via the serial console.
