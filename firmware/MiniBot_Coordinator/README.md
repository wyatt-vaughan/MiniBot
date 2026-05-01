# MiniBot Coordinator

Python/PyQt6 application that runs on the Raspberry Pi. It manages board state,
runs path planning, drives the GUI, and sends motion commands to the Server over
USB serial.

---

## Structure

```
MiniBot_Coordinator/
    main.py             Entry point; starts the PyQt6 MainWindow
    config.py           All system constants (see below)
    comms/              Serial protocol and handler
    gui/                Top-level window, board widget, and control tabs
    models/             Board state and piece data
    planning/           Path planners
    simulation/         Software-in-the-loop motion simulator
```

---

## config.py

All system constants live here, organized into classes:

| Class | Contents |
|-------|---------|
| `BOARD` | Physical geometry: playing area dimensions, square size, border margins |
| `PIECES` | 34 piece IDs (0x01-0x11 white, 0x12-0x22 black), home positions, rank assignments |
| `COMM` | Serial protocol constants |
| `GUI` | Colors, fonts, window dimensions |
| `PLANNING` | Path planner registry |
| `CHESS` | FEN string, rules engine adapter hooks |
| `SIMULATOR` | Software-in-the-loop motion parameters |

---

## GUI (gui/)

`ChessBoardWidget` renders all 34 pieces with live position and orientation updates.
The right panel has tabbed controls: Path Planning, Debug, System Control, and
Position Tracker.

Board clicks and manual inputs flow through the Path Planning tab into the active
planner, which produces a list of `MoveCommand` objects. These are sent to either
the `SerialHandler` (hardware) or the `Simulator` (software-in-the-loop).

---

## Path Planning (planning/)

Two planners derive from `BasePlanner`:

- `DirectPlanner`: All pieces move simultaneously to their targets.
- `QueuedPlanner`: Pieces move one at a time in ascending ID order.

Both accept an optional `validator(piece_id, x, y) -> bool` callback for chess
legality checks.

Key objects:

| Object | Purpose |
|--------|---------|
| `MoveCommand` | Planner output: `piece_id`, target x/y/theta, `duration_ms`, `sequence_num` |
| `BoardState` | Container for all 34 `Piece` objects; home positions; accessors by ID, color, or active state |
| `Piece` | One robot: ID, color, rank, position, orientation, battery voltage, capture/staged flags |

---

## Serial Handler (comms/)

`SerialHandler` runs a background `QThread` for non-blocking I/O. Outgoing commands
are queued with backpressure; stale commands are dropped if the queue backs up.
Incoming lines are parsed by `protocol.py` into `ParsedMessage` dataclasses.

Qt signals emitted: `position_received`, `ack_received`, `error_received`,
`mag_field_received`, `connection_changed`.

---

## Simulator (simulation/)

`MotionSimulator` is a drop-in replacement for the serial link. It simulates
two-phase motion (rotate to face target, then translate), enforces board boundaries,
and detects collisions between pieces. Emits the same Qt signals as `SerialHandler`
so the rest of the application is unaware of the difference.

---

## Setup

Requires Python 3.10+ and PyQt6.

```bash
pip install pyqt6 pyserial
python main.py
```

Connect the Server ESP32 over USB before launching. Select the serial port from
the System Control tab.
