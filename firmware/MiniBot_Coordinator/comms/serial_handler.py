"""
comms/serial_handler.py  —  MiniBot Chess Swarm Coordinator

Serial I/O runs on a QThread worker so the GUI thread is never blocked.

Usage:
    handler = SerialHandler()
    handler.position_received.connect(my_slot)
    handler.connect_port('/dev/ttyUSB0', 115200)
    handler.send(build_poll())
    handler.disconnect_port()
"""

from __future__ import annotations

import queue
from typing import Optional

import serial
import serial.tools.list_ports

from PyQt6.QtCore import QObject, QThread, pyqtSignal, pyqtSlot

from comms.protocol import parse_line, ParsedMessage
from config import COMM


# ---------------------------------------------------------------------------
# Worker — lives on a background QThread
# ---------------------------------------------------------------------------

class _SerialWorker(QObject):
    """Reads lines from serial port and emits structured signals.

    This object must be moved to a QThread via moveToThread() before
    starting.  All I/O blocking happens inside run(); send() enqueues
    outgoing bytes that are flushed each loop iteration.
    """

    # Outgoing: parsed responses
    position_received  = pyqtSignal(int, float, float, float)  # id, x, y, theta
    ack_received       = pyqtSignal(int)                        # id
    error_received     = pyqtSignal(int, str)                   # id, reason
    poll_done          = pyqtSignal()
    raw_line_received  = pyqtSignal(str)                        # raw text for debug log
    connection_changed = pyqtSignal(bool)                       # True=connected
    worker_error       = pyqtSignal(str)                        # unrecoverable error msg

    finished           = pyqtSignal()

    def __init__(self) -> None:
        super().__init__()
        self._port_name:  Optional[str] = None
        self._baud_rate:  int           = COMM.DEFAULT_BAUD_RATE
        self._running:    bool          = False
        self._send_queue: queue.Queue[bytes] = queue.Queue()
        self._serial:     Optional[serial.Serial] = None

    # ------------------------------------------------------------------
    # Public API (called from GUI thread — thread-safe via queue / flags)
    # ------------------------------------------------------------------

    def configure(self, port_name: str, baud_rate: int) -> None:
        self._port_name = port_name
        self._baud_rate = baud_rate

    def enqueue(self, data: bytes) -> None:
        """Enqueue bytes to be written on the next loop iteration."""
        self._send_queue.put(data)

    # ------------------------------------------------------------------
    # Main loop (runs on background QThread)
    # ------------------------------------------------------------------

    @pyqtSlot()
    def run(self) -> None:
        self._running = True
        try:
            self._serial = serial.Serial(
                self._port_name,
                self._baud_rate,
                timeout=COMM.READ_TIMEOUT_S,
            )
            self.connection_changed.emit(True)
        except serial.SerialException as exc:
            self.worker_error.emit(f"Cannot open {self._port_name}: {exc}")
            self.finished.emit()
            return

        try:
            while self._running:
                # --- drain send queue ---
                while not self._send_queue.empty():
                    try:
                        data = self._send_queue.get_nowait()
                        self._serial.write(data)
                    except queue.Empty:
                        break
                    except serial.SerialException as exc:
                        self.worker_error.emit(f"Write error: {exc}")
                        self._running = False
                        break

                if not self._running:
                    break

                # --- read one line (non-blocking due to timeout) ---
                try:
                    raw = self._serial.readline()
                except serial.SerialException as exc:
                    self.worker_error.emit(f"Read error: {exc}")
                    self._running = False
                    break

                if raw:
                    self._dispatch(raw)

        finally:
            if self._serial and self._serial.is_open:
                self._serial.close()
            self.connection_changed.emit(False)
            self.finished.emit()

    def stop(self) -> None:
        """Signal the run loop to exit cleanly."""
        self._running = False

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _dispatch(self, raw: bytes) -> None:
        """Parse a raw line and emit the appropriate signal."""
        msg: ParsedMessage = parse_line(raw)
        self.raw_line_received.emit(msg.raw)

        if msg.msg_type == COMM.RESP_POSITION and msg.piece_id is not None:
            self.position_received.emit(
                msg.piece_id,
                msg.x_mm     or 0.0,
                msg.y_mm     or 0.0,
                msg.theta_deg or 0.0,
            )
        elif msg.msg_type == COMM.RESP_ACK and msg.piece_id is not None:
            self.ack_received.emit(msg.piece_id)
        elif msg.msg_type == COMM.RESP_ERROR and msg.piece_id is not None:
            self.error_received.emit(msg.piece_id, msg.reason or '')
        elif msg.msg_type == COMM.RESP_DONE:
            self.poll_done.emit()
        # UNKNOWN lines are still emitted via raw_line_received above


# ---------------------------------------------------------------------------
# SerialHandler — public facade (lives on GUI thread)
# ---------------------------------------------------------------------------

class SerialHandler(QObject):
    """Manages the serial worker thread.

    Connect to the public signals and call connect_port() / disconnect_port()
    from the GUI thread.  Use send() to enqueue outgoing bytes.
    """

    # Mirror worker signals (re-emitted so callers connect here, not to worker)
    position_received  = pyqtSignal(int, float, float, float)
    ack_received       = pyqtSignal(int)
    error_received     = pyqtSignal(int, str)
    poll_done          = pyqtSignal()
    raw_line_received  = pyqtSignal(str)
    connection_changed = pyqtSignal(bool)          # True=connected
    serial_error       = pyqtSignal(str)

    def __init__(self, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._thread: Optional[QThread] = None
        self._worker: Optional[_SerialWorker] = None
        self._connected: bool = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    # ------------------------------------------------------------------
    # Connection management
    # ------------------------------------------------------------------

    def connect_port(self, port_name: str, baud_rate: int = COMM.DEFAULT_BAUD_RATE) -> None:
        """Open a serial connection on a background thread."""
        if self._thread and self._thread.isRunning():
            return  # already running; caller should disconnect first

        self._worker = _SerialWorker()
        self._worker.configure(port_name, baud_rate)

        self._thread = QThread(self)
        self._worker.moveToThread(self._thread)

        # Wire worker signals → our public signals
        self._worker.position_received.connect(self.position_received)
        self._worker.ack_received.connect(self.ack_received)
        self._worker.error_received.connect(self.error_received)
        self._worker.poll_done.connect(self.poll_done)
        self._worker.raw_line_received.connect(self.raw_line_received)
        self._worker.worker_error.connect(self.serial_error)
        self._worker.connection_changed.connect(self._on_connection_changed)

        # Lifecycle wiring
        self._thread.started.connect(self._worker.run)
        self._worker.finished.connect(self._thread.quit)
        self._worker.finished.connect(self._worker.deleteLater)
        self._thread.finished.connect(self._thread.deleteLater)
        self._thread.finished.connect(self._on_thread_finished)

        self._thread.start()

    def disconnect_port(self) -> None:
        """Stop the worker and close the serial port."""
        if self._worker:
            self._worker.stop()
        # Thread will finish and emit finished → quit chain above

    def send(self, data: bytes) -> None:
        """Enqueue bytes to be sent from the worker thread."""
        if self._worker and self._connected:
            self._worker.enqueue(data)

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------

    @pyqtSlot(bool)
    def _on_connection_changed(self, connected: bool) -> None:
        self._connected = connected
        self.connection_changed.emit(connected)

    @pyqtSlot()
    def _on_thread_finished(self) -> None:
        self._thread = None
        self._worker = None

    # ------------------------------------------------------------------
    # Utility (static, no serial needed)
    # ------------------------------------------------------------------

    @staticmethod
    def available_ports() -> list[str]:
        """Return a list of available serial port names on the current OS."""
        return sorted(p.device for p in serial.tools.list_ports.comports())
