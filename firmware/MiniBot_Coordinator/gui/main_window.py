"""
gui/main_window.py  —  MiniBot Chess Swarm Coordinator

Top-level QMainWindow.  Assembles: ChessBoardWidget (left) + QTabWidget
(right) and wires all inter-component signals.

Signal routing summary:
  Board.piece_selected      → PathPlanningTab.on_piece_selected
  Board.target_set          → PathPlanningTab.on_target_set
  PathPlanningTab.send_commands → _on_send_move_commands (serial or sim)
  DebugTab.send_raw         → _on_debug_send_raw (serial or sim)
  DebugTab.simulator_mode_changed → _on_sim_mode_changed
  SystemControlTab.send_raw → serial_handler.send
  PositionTrackerTab.send_raw → serial_handler.send
  serial_handler.position_received → _on_position_received → board + tracker
  serial_handler.raw_line_received → DebugTab.on_raw_line_received
  serial_handler.ack_received      → DebugTab.on_ack_received
  serial_handler.error_received    → DebugTab.on_error_received
  serial_handler.serial_error      → status bar
  simulator.position_updated       → _on_position_received → board + tracker
  simulator.log_message            → DebugTab.on_sim_log
"""

from __future__ import annotations

from typing import List, Optional

from PyQt6.QtCore import QTimer, pyqtSlot
from PyQt6.QtWidgets import (
    QHBoxLayout, QMainWindow, QStatusBar, QTabWidget, QWidget,
)

from comms.protocol import build_poll, build_move
from comms.serial_handler import SerialHandler
from config import COMM, GUI
from models.piece import BoardState
from planning.base_planner import MoveCommand
from simulation.simulator import MotionSimulator

from gui.chessboard_widget import ChessBoardWidget
from gui.tabs.path_planning_tab   import PathPlanningTab
from gui.tabs.debug_tab           import DebugTab
from gui.tabs.system_control_tab  import SystemControlTab
from gui.tabs.position_tracker_tab import PositionTrackerTab


class MainWindow(QMainWindow):
    """Application main window."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(GUI.WINDOW_TITLE)
        self.setMinimumSize(GUI.WINDOW_MIN_WIDTH, GUI.WINDOW_MIN_HEIGHT)

        # Shared state
        self._board      = BoardState()
        self._board.reset_to_home()
        self._handler    = SerialHandler(self)
        self._simulator  = MotionSimulator(self._board, parent=self)
        self._sim_mode   = False  # True when debug tab simulator is active

        # Auto-poll timer (fires POLL at the configured interval when connected)
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(COMM.DEFAULT_POLL_INTERVAL_MS)
        self._poll_timer.timeout.connect(self._on_poll_timer)

        self._build_ui()
        self._wire_signals()

        # Status bar
        self._status_bar: QStatusBar = self.statusBar()
        self._status_bar.showMessage('Not connected')

    # ------------------------------------------------------------------
    # UI assembly
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)

        layout = QHBoxLayout(central)
        layout.setSpacing(6)
        layout.setContentsMargins(6, 6, 6, 6)

        # Left: chessboard
        self._board_widget = ChessBoardWidget(self._board, self)
        layout.addWidget(self._board_widget, stretch=1)

        # Right: tabbed control panel
        self._tabs = QTabWidget()
        self._tabs.setMinimumWidth(GUI.CONTROL_PANEL_MIN_WIDTH)
        self._tabs.setSizePolicy(
            self._tabs.sizePolicy().horizontalPolicy(),
            self._tabs.sizePolicy().verticalPolicy(),
        )

        self._path_tab   = PathPlanningTab(self._board, self)
        self._debug_tab  = DebugTab(self)
        self._sys_tab    = SystemControlTab(self._handler, self)
        self._track_tab  = PositionTrackerTab(self._board, self)

        self._tabs.addTab(self._path_tab,  'Path Planning')
        self._tabs.addTab(self._debug_tab, 'Debug')
        self._tabs.addTab(self._sys_tab,   'System Control')
        self._tabs.addTab(self._track_tab, 'Position Tracker')

        layout.addWidget(self._tabs, stretch=0)

    def _wire_signals(self) -> None:
        # Board → path planning tab (sync fields + queuing)\n        self._board_widget.piece_selected.connect(self._path_tab.on_piece_selected)\n        self._board_widget.target_set.connect(self._path_tab.on_target_set)\n        self._board_widget.target_queued.connect(self._path_tab.enqueue_from_board)\n\n        # Board left-click → immediate dispatch (do not require queue + send)\n        self._board_widget.target_set.connect(self._on_board_target_set)

        # Path planning → send (serial or sim depending on mode)
        self._path_tab.send_commands.connect(self._on_send_move_commands)

        # Debug tab → send (serial or sim) + simulator toggle
        self._debug_tab.send_raw.connect(self._on_debug_send_raw)
        self._debug_tab.simulator_mode_changed.connect(self._on_sim_mode_changed)

        # System control → send (always serial; system commands never simulated)
        self._sys_tab.send_raw.connect(self._handler.send)

        # Position tracker → poll (always serial)
        self._track_tab.send_raw.connect(self._handler.send)

        # Serial handler → board updates + UI
        self._handler.position_received.connect(self._on_position_received)
        self._handler.raw_line_received.connect(self._debug_tab.on_raw_line_received)
        self._handler.ack_received.connect(self._debug_tab.on_ack_received)
        self._handler.error_received.connect(self._debug_tab.on_error_received)
        self._handler.position_received.connect(self._track_tab.on_position_received)
        self._handler.connection_changed.connect(self._on_connection_changed)
        self._handler.serial_error.connect(self._on_serial_error)

        # Simulator → board updates + debug log
        self._simulator.position_updated.connect(self._on_position_received)
        self._simulator.position_updated.connect(self._track_tab.on_position_received)
        self._simulator.log_message.connect(self._debug_tab.on_sim_log)

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------

    @pyqtSlot(int, float, float, float)
    def _on_position_received(self, piece_id: int, x_mm: float, y_mm: float, theta_deg: float) -> None:
        """Update board state and refresh canvas."""
        self._board.update_piece_position(piece_id, x_mm, y_mm, theta_deg)
        self._board_widget.refresh()

    @pyqtSlot(int, float, float)
    def _on_board_target_set(self, piece_id: int, x_mm: float, y_mm: float) -> None:
        """Immediately dispatch a move when the user clicks a target on the board.

        Uses the DirectPlanner to build a single MoveCommand and sends it
        straight to _on_send_move_commands without touching the queue.
        """
        from planning.direct_planner import DirectPlanner
        piece = self._board.get_piece(piece_id)
        if piece is None:
            return
        planner   = DirectPlanner()
        positions = {piece_id: (piece.x_mm, piece.y_mm)}
        targets   = {piece_id: (x_mm, y_mm)}
        commands  = planner.plan_moves(positions, targets)
        if commands:
            self._on_send_move_commands(commands)

    @pyqtSlot(list)
    def _on_send_move_commands(self, commands: List[MoveCommand]) -> None:
        """Dispatch move commands to the simulator or serial handler."""
        sorted_cmds = sorted(commands, key=lambda c: c.sequence_num)

        if self._sim_mode:
            # Update simulator speed from current debug tab setting
            self._simulator.speed_mm_s = self._debug_tab.simulator_speed_mm_s
            self._simulator.queue_moves(sorted_cmds)
        else:
            for cmd in sorted_cmds:
                theta = cmd.target_theta if cmd.target_theta is not None else 0.0
                self._handler.send(build_move(
                    cmd.piece_id,
                    cmd.target_x_mm,
                    cmd.target_y_mm,
                    theta,
                    cmd.duration_ms,
                ))

    @pyqtSlot(bytes)
    def _on_debug_send_raw(self, data: bytes) -> None:
        """Route a raw bytes send from the debug tab to serial or simulator."""
        if self._sim_mode:
            cmd = self._parse_mov_bytes(data)
            if cmd is not None:
                self._simulator.speed_mm_s = self._debug_tab.simulator_speed_mm_s
                self._simulator.queue_moves([cmd])
        else:
            self._handler.send(data)

    @pyqtSlot(bool)
    def _on_sim_mode_changed(self, enabled: bool) -> None:
        self._sim_mode = enabled
        if enabled:
            self._status_bar.showMessage('Simulator mode active')
        else:
            self._simulator.stop_all()
            if self._handler.is_connected:
                self._status_bar.showMessage('Connected')
            else:
                self._status_bar.showMessage('Not connected')

    @pyqtSlot(bool)
    def _on_connection_changed(self, connected: bool) -> None:
        if connected:
            self._status_bar.showMessage('Connected')
            self._poll_timer.start()
        else:
            self._status_bar.showMessage('Disconnected')
            self._poll_timer.stop()

    @pyqtSlot(str)
    def _on_serial_error(self, message: str) -> None:
        self._status_bar.showMessage(f'Serial error: {message}')

    @pyqtSlot()
    def _on_poll_timer(self) -> None:
        """Periodically request a full position update from all bots."""
        if self._handler.is_connected:
            self._handler.send(build_poll())

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _parse_mov_bytes(data: bytes) -> Optional[MoveCommand]:
        """Parse a MOV command byte string back into a MoveCommand for the simulator.

        Format: MOV,{id},{x_mm},{y_mm},{theta_deg},{duration_ms}\\n
        Returns None if the data is not a valid MOV command.
        """
        from config import COMM
        try:
            line   = data.decode(COMM.ENCODING).strip()
            parts  = line.split(COMM.DELIMITER)
            if parts[0].upper() != COMM.CMD_MOVE or len(parts) != 6:
                return None
            return MoveCommand(
                piece_id     = int(parts[1]),
                target_x_mm  = float(parts[2]),
                target_y_mm  = float(parts[3]),
                target_theta = float(parts[4]),
                duration_ms  = int(parts[5]),
            )
        except (ValueError, IndexError, UnicodeDecodeError):
            return None
