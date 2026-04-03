"""
gui/tabs/system_control_tab.py  —  MiniBot Chess Swarm Coordinator

System Control tab:
  - COM port selector + baud rate + Connect / Disconnect buttons
  - Position update polling frequency (spinbox, ms)
  - Electromagnet mode slider (Off / On / Sync)
  - "Send Electromagnet Sync" button
  - Connection status label
"""

from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import pyqtSignal, pyqtSlot, Qt
from PyQt6.QtWidgets import (
    QComboBox, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSlider, QSpinBox, QVBoxLayout, QWidget,
)

from comms.protocol import build_mag, build_rate
from comms.serial_handler import SerialHandler
from config import COMM, GUI


class SystemControlTab(QWidget):
    """System control panel tab.

    Signals:
        send_raw(bytes) — emit bytes to the serial handler
    """

    send_raw = pyqtSignal(bytes)

    def __init__(
        self,
        serial_handler: SerialHandler,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._handler = serial_handler
        self._build_ui()
        self._wire_signals()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(10)

        # --- Connection ---
        conn_group = QGroupBox('Serial Connection')
        cl = QVBoxLayout(conn_group)

        port_row = QHBoxLayout()
        port_row.addWidget(QLabel('Port:'))
        self._port_combo = QComboBox()
        self._port_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self._refresh_ports()
        port_row.addWidget(self._port_combo)

        self._btn_refresh_ports = QPushButton('↺')
        self._btn_refresh_ports.setFixedWidth(28)
        self._btn_refresh_ports.setToolTip('Refresh serial port list')
        self._btn_refresh_ports.clicked.connect(self._refresh_ports)
        port_row.addWidget(self._btn_refresh_ports)
        cl.addLayout(port_row)

        baud_row = QHBoxLayout()
        baud_row.addWidget(QLabel('Baud rate:'))
        self._baud_combo = QComboBox()
        for baud in [9600, 57600, 115200, 230400, 460800]:
            self._baud_combo.addItem(str(baud), userData=baud)
        default_idx = self._baud_combo.findData(COMM.DEFAULT_BAUD_RATE)
        if default_idx >= 0:
            self._baud_combo.setCurrentIndex(default_idx)
        baud_row.addWidget(self._baud_combo)
        baud_row.addStretch()
        cl.addLayout(baud_row)

        btn_row = QHBoxLayout()
        self._btn_connect = QPushButton('Connect')
        self._btn_connect.setMinimumHeight(36)
        self._btn_connect.clicked.connect(self._on_connect)
        btn_row.addWidget(self._btn_connect)

        self._btn_disconnect = QPushButton('Disconnect')
        self._btn_disconnect.setMinimumHeight(36)
        self._btn_disconnect.setEnabled(False)
        self._btn_disconnect.clicked.connect(self._on_disconnect)
        btn_row.addWidget(self._btn_disconnect)
        cl.addLayout(btn_row)

        self._status_label = QLabel('Status: Disconnected')
        self._status_label.setStyleSheet('color: gray;')
        cl.addWidget(self._status_label)

        root.addWidget(conn_group)

        # --- Polling ---
        poll_group = QGroupBox('Position Polling')
        pl = QVBoxLayout(poll_group)
        poll_row = QHBoxLayout()
        poll_row.addWidget(QLabel('Interval (ms):'))
        self._poll_spin = QSpinBox()
        self._poll_spin.setRange(100, 60000)
        self._poll_spin.setSingleStep(100)
        self._poll_spin.setValue(COMM.DEFAULT_POLL_INTERVAL_MS)
        poll_row.addWidget(self._poll_spin)

        self._btn_set_rate = QPushButton('Set Rate')
        self._btn_set_rate.clicked.connect(self._on_set_rate)
        poll_row.addWidget(self._btn_set_rate)
        poll_row.addStretch()
        pl.addLayout(poll_row)

        root.addWidget(poll_group)

        # --- Electromagnets ---
        mag_group = QGroupBox('Electromagnets')
        ml = QVBoxLayout(mag_group)

        mag_label_row = QHBoxLayout()
        mag_label_row.addWidget(QLabel('Off'))
        mag_label_row.addStretch()
        mag_label_row.addWidget(QLabel('On'))
        ml.addLayout(mag_label_row)

        self._mag_slider = QSlider(Qt.Orientation.Horizontal)
        self._mag_slider.setRange(COMM.MAG_OFF, COMM.MAG_ON)
        self._mag_slider.setTickInterval(1)
        self._mag_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self._mag_slider.setValue(COMM.MAG_OFF)
        self._mag_slider.valueChanged.connect(self._on_mag_slider)
        ml.addWidget(self._mag_slider)

        self._btn_mag_sync = QPushButton('Send Electromagnet Sync')
        self._btn_mag_sync.setMinimumHeight(36)
        self._btn_mag_sync.clicked.connect(self._on_mag_sync)
        ml.addWidget(self._btn_mag_sync)

        root.addWidget(mag_group)
        root.addStretch()

    def _wire_signals(self) -> None:
        self._handler.connection_changed.connect(self._on_connection_changed)

    # ------------------------------------------------------------------
    # Public slots
    # ------------------------------------------------------------------

    @pyqtSlot(bool)
    def _on_connection_changed(self, connected: bool) -> None:
        if connected:
            self._status_label.setText('Status: Connected')
            self._status_label.setStyleSheet('color: green; font-weight: bold;')
        else:
            self._status_label.setText('Status: Disconnected')
            self._status_label.setStyleSheet('color: gray;')
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)

    # ------------------------------------------------------------------
    # Button / slider handlers
    # ------------------------------------------------------------------

    def _on_connect(self) -> None:
        port = self._port_combo.currentText()
        baud = self._baud_combo.currentData() or COMM.DEFAULT_BAUD_RATE
        if port:
            self._handler.connect_port(port, baud)

    def _on_disconnect(self) -> None:
        self._handler.disconnect_port()

    def _on_set_rate(self) -> None:
        interval_ms = self._poll_spin.value()
        self.send_raw.emit(build_rate(interval_ms))

    def _on_mag_slider(self, value: int) -> None:
        self.send_raw.emit(build_mag(value))

    def _on_mag_sync(self) -> None:
        self.send_raw.emit(build_mag(COMM.MAG_SYNC))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentText()
        self._port_combo.clear()
        ports = SerialHandler.available_ports()
        for p in ports:
            self._port_combo.addItem(p)
        if current in ports:
            self._port_combo.setCurrentText(current)
