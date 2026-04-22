"""
Serial Protocol
---------------
  Host → Device : b'T\n'
  Device → Host : b'T:<uint32_microseconds>\n'  e.g. b'T:1234567890\n'

Arduino Firmware (flash to each device)
-----------------------------------------

void setup() {
    Serial.begin(115200);
    Serial.println("READY");
}
void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'T' || c == 't') {
            char buf[22];
            snprintf(buf, sizeof(buf), "T:%lu", micros());
            Serial.println(buf);
        }
        // all other bytes (newlines, carriage returns, etc.) are ignored
    }
}

"""

from __future__ import annotations

import collections
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

import customtkinter as ctk

try:
    import serial
    import serial.tools.list_ports
    _SERIAL_AVAILABLE = True
except ImportError:
    _SERIAL_AVAILABLE = False

# ── Appearance ────────────────────────────────────────────────────────────────
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("blue")

CLOCK_FONT       = ("Consolas", 40, "bold")
DRIFT_FONT       = ("Consolas", 22, "bold")
LABEL_FONT       = ("Segoe UI", 12)
SUBHEADER_FONT   = ("Segoe UI", 13, "bold")
STATUS_FONT      = ("Segoe UI", 12)

COLOR_CONNECTED    = "#2ecc71"
COLOR_DISCONNECTED = "#6b6b6b"
COLOR_DRIFT_OK     = "#2ecc71"   # < 1 ms
COLOR_DRIFT_WARN   = "#f39c12"   # < 10 ms
COLOR_DRIFT_BAD    = "#e74c3c"   # ≥ 10 ms

BAUD_RATES = ["9600", "19200", "38400", "57600", "115200",
              "230400", "500000", "1000000"]
DEFAULT_BAUD = "500000"

POLL_HZ          = 50           # serial query rate
POLL_SLEEP       = 1.0 / POLL_HZ
UPDATE_MS        = 33           # GUI refresh interval (~30 fps)
_SAMPLE_WINDOW   = 500          # rolling fit window (~10 s at 50 Hz)
_MIN_FIT_SAMPLES = 20           # minimum samples before showing drift

# ── Data model ────────────────────────────────────────────────────────────────

@dataclass
class SideState:
    """All mutable state for one serial pane."""
    label: str                          # "Left" or "Right"
    conn: Optional["serial.Serial"] = None
    offset_us: Optional[int]       = None
    latest_us: Optional[int]       = None
    running: bool                   = False
    thread: Optional[threading.Thread] = None
    lock: threading.Lock            = field(default_factory=threading.Lock)


# ── Polling thread ─────────────────────────────────────────────────────────────

def _poll_thread(state: SideState, side_id: int,
                 result_queue: queue.Queue) -> None:
    """
    Background thread: sends 'T\\n' at POLL_HZ, parses 'T:<us>' responses,
    pushes (side_id, microseconds) tuples onto result_queue.
    Pushes (side_id, None) on error to signal a disconnect.
    """
    interval = POLL_SLEEP
    while state.running:
        loop_start = time.monotonic()
        try:
            if state.conn is None or not state.conn.is_open:
                result_queue.put((side_id, None))
                break
            state.conn.write(b"T\n")
            raw = state.conn.readline()
            if raw:
                host_time = time.perf_counter()  # stamp immediately after read
                text = raw.decode("ascii", errors="ignore").strip()
                if text.startswith("T:"):
                    us_str = text[2:]
                    if us_str.isdigit():
                        result_queue.put((side_id, int(us_str), host_time))
        except Exception:
            result_queue.put((side_id, None, 0.0))
            break
        # Sleep the remainder of the interval to maintain POLL_HZ
        elapsed = time.monotonic() - loop_start
        remaining = interval - elapsed
        if remaining > 0:
            time.sleep(remaining)


# ── Helper ─────────────────────────────────────────────────────────────────────

def format_elapsed(us: int) -> str:
    """Convert microseconds to MM:SS:mmm:uuu display string."""
    minutes = us // 60_000_000
    us     %= 60_000_000
    seconds = us // 1_000_000
    us     %= 1_000_000
    millis  = us // 1_000
    micros  = us  % 1_000
    return f"{minutes:02d}:{seconds:02d}:{millis:03d}:{micros:03d}"


def _list_ports() -> list[str]:
    if not _SERIAL_AVAILABLE:
        return ["(pyserial not installed)"]
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return ports if ports else ["(no ports found)"]


# ── Side Pane widget ───────────────────────────────────────────────────────────

class SidePane(ctk.CTkFrame):
    """One controller pane (left or right)."""

    def __init__(self, master, side_id: int, label: str, state: SideState,
                 result_queue: queue.Queue, **kwargs):
        super().__init__(master, corner_radius=12,
                         fg_color=("gray90", "gray17"), **kwargs)
        self.side_id      = side_id
        self.label        = label
        self.state        = state
        self.result_queue = result_queue

        self._build_ui()

    def _build_ui(self) -> None:
        self.grid_columnconfigure(0, weight=1)
        self.grid_columnconfigure(1, weight=1)

        # ── Title ──────────────────────────────────────────────────────
        ctk.CTkLabel(self, text=f"Controller  {self.label}",
                     font=SUBHEADER_FONT).grid(
            row=0, column=0, columnspan=2, pady=(16, 8), padx=16)

        # ── Port selector ──────────────────────────────────────────────
        ctk.CTkLabel(self, text="Port", font=LABEL_FONT).grid(
            row=1, column=0, padx=(16, 4), pady=(4, 0), sticky="w")
        ctk.CTkLabel(self, text="Baud", font=LABEL_FONT).grid(
            row=1, column=1, padx=(4, 16), pady=(4, 0), sticky="w")

        self._port_var = ctk.StringVar()
        ports = _list_ports()
        self._port_var.set(ports[0])
        self._port_cb = ctk.CTkComboBox(
            self, values=ports, variable=self._port_var,
            width=130, font=LABEL_FONT)
        self._port_cb.grid(row=2, column=0, padx=(16, 4), pady=4, sticky="ew")

        self._baud_var = ctk.StringVar(value=DEFAULT_BAUD)
        self._baud_cb = ctk.CTkComboBox(
            self, values=BAUD_RATES, variable=self._baud_var,
            width=110, font=LABEL_FONT)
        self._baud_cb.grid(row=2, column=1, padx=(4, 16), pady=4, sticky="ew")

        # ── Connect / Disconnect buttons ───────────────────────────────
        btn_frame = ctk.CTkFrame(self, fg_color="transparent")
        btn_frame.grid(row=3, column=0, columnspan=2,
                       padx=16, pady=(8, 4), sticky="ew")
        btn_frame.grid_columnconfigure(0, weight=1)
        btn_frame.grid_columnconfigure(1, weight=1)

        self._btn_connect = ctk.CTkButton(
            btn_frame, text="Connect", width=100,
            command=self._on_connect, font=LABEL_FONT)
        self._btn_connect.grid(row=0, column=0, padx=(0, 4), sticky="ew")

        self._btn_disconnect = ctk.CTkButton(
            btn_frame, text="Disconnect", width=100,
            fg_color=("gray70", "gray30"),
            hover_color=("gray60", "gray40"),
            command=self._on_disconnect, font=LABEL_FONT, state="disabled")
        self._btn_disconnect.grid(row=0, column=1, padx=(4, 0), sticky="ew")

        # ── Status indicator ───────────────────────────────────────────
        self._status_label = ctk.CTkLabel(
            self, text="●  Disconnected",
            font=STATUS_FONT, text_color=COLOR_DISCONNECTED)
        self._status_label.grid(
            row=4, column=0, columnspan=2, pady=(4, 8))

        # ── Clock display ──────────────────────────────────────────────
        clock_frame = ctk.CTkFrame(self, corner_radius=10,
                                   fg_color=("gray80", "gray12"))
        clock_frame.grid(row=5, column=0, columnspan=2,
                         padx=16, pady=(8, 20), sticky="ew")

        self._clock_label = ctk.CTkLabel(
            clock_frame, text="00:00:000:000",
            font=CLOCK_FONT, text_color=("gray20", "gray90"))
        self._clock_label.pack(pady=24, padx=20)

    # ── Public API ─────────────────────────────────────────────────────

    def refresh_ports(self) -> None:
        ports = _list_ports()
        self._port_cb.configure(values=ports)
        if self._port_var.get() not in ports:
            self._port_var.set(ports[0])

    def set_clock(self, elapsed_us: Optional[int]) -> None:
        if elapsed_us is None:
            self._clock_label.configure(text="00:00:00.000")
        else:
            self._clock_label.configure(text=format_elapsed(elapsed_us))

    def set_connected(self, connected: bool) -> None:
        if connected:
            self._status_label.configure(
                text="●  Connected", text_color=COLOR_CONNECTED)
            self._btn_connect.configure(state="disabled")
            self._btn_disconnect.configure(state="normal")
            self._port_cb.configure(state="disabled")
            self._baud_cb.configure(state="disabled")
        else:
            self._status_label.configure(
                text="●  Disconnected", text_color=COLOR_DISCONNECTED)
            self._btn_connect.configure(state="normal")
            self._btn_disconnect.configure(state="disabled")
            self._port_cb.configure(state="normal")
            self._baud_cb.configure(state="normal")

    # ── Internal handlers ──────────────────────────────────────────────

    def _on_connect(self) -> None:
        if not _SERIAL_AVAILABLE:
            self._status_label.configure(
                text="●  pyserial missing", text_color=COLOR_DRIFT_BAD)
            return
        port = self._port_var.get()
        baud = int(self._baud_var.get())
        try:
            conn = serial.Serial(port, baud, timeout=0.1)
            with self.state.lock:
                self.state.conn      = conn
                self.state.offset_us = None
                self.state.latest_us = None
                self.state.running   = True
            t = threading.Thread(
                target=_poll_thread,
                args=(self.state, self.side_id, self.result_queue),
                daemon=True)
            self.state.thread = t
            t.start()
            self.set_connected(True)
        except Exception as exc:
            self._status_label.configure(
                text=f"●  Error: {exc}", text_color=COLOR_DRIFT_BAD)

    def _on_disconnect(self) -> None:
        self._do_disconnect()

    def _do_disconnect(self) -> None:
        self.state.running = False
        if self.state.thread and self.state.thread.is_alive():
            self.state.thread.join(timeout=1.0)
        try:
            if self.state.conn and self.state.conn.is_open:
                self.state.conn.close()
        except Exception:
            pass
        with self.state.lock:
            self.state.conn      = None
            self.state.offset_us = None
            self.state.latest_us = None
        self.set_connected(False)
        self.set_clock(None)


# ── Main Application ───────────────────────────────────────────────────────────

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Clock Drift Demo")
        self.geometry("900x580")
        self.minsize(820, 520)
        self.resizable(True, True)

        self._result_queue = queue.Queue()
        self._state_left  = SideState(label="Left")
        self._state_right = SideState(label="Right")

        # Syncing flag — True after SYNC is pressed
        self._syncing = False

        # Per-side (host_time, device_us) sample deques for linear fitting
        self._samples_left: collections.deque  = collections.deque(maxlen=_SAMPLE_WINDOW)
        self._samples_right: collections.deque = collections.deque(maxlen=_SAMPLE_WINDOW)

        # EMA state for the final drift display (light smoothing over the fit output)
        self._drift_filtered: Optional[float] = None
        self._drift_alpha = 0.15
        self._drift_offset_us: float = 0.0  # zeroed from first stable fit after sync

        self._build_ui()
        self._schedule_update()

    # ── UI construction ────────────────────────────────────────────────

    def _build_ui(self) -> None:
        self.grid_rowconfigure(0, weight=0)   # header / drift display
        self.grid_rowconfigure(1, weight=1)   # panes
        self.grid_rowconfigure(2, weight=0)   # footer / sync button
        self.grid_columnconfigure(0, weight=1)

        # ── Header: drift indicator ────────────────────────────────────
        header = ctk.CTkFrame(self, corner_radius=0,
                              fg_color=("gray85", "gray15"), height=56)
        header.grid(row=0, column=0, sticky="ew")
        header.grid_columnconfigure(0, weight=1)
        header.grid_propagate(False)

        self._drift_label = ctk.CTkLabel(
            header, text="Drift:  —",
            font=DRIFT_FONT, text_color=COLOR_DISCONNECTED)
        self._drift_label.grid(row=0, column=0, pady=10)

        # ── Middle: two panes + separator ─────────────────────────────
        mid = ctk.CTkFrame(self, fg_color="transparent")
        mid.grid(row=1, column=0, sticky="nsew", padx=16, pady=(12, 0))
        mid.grid_columnconfigure(0, weight=1)
        mid.grid_columnconfigure(1, weight=0)   # separator
        mid.grid_columnconfigure(2, weight=1)
        mid.grid_rowconfigure(0, weight=1)

        self._pane_left = SidePane(
            mid, side_id=0, label="Left",
            state=self._state_left,
            result_queue=self._result_queue)
        self._pane_left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))

        sep = ctk.CTkFrame(mid, width=2, corner_radius=0,
                           fg_color=("gray70", "gray30"))
        sep.grid(row=0, column=1, sticky="ns", pady=12)

        self._pane_right = SidePane(
            mid, side_id=1, label="Right",
            state=self._state_right,
            result_queue=self._result_queue)
        self._pane_right.grid(row=0, column=2, sticky="nsew", padx=(6, 0))

        # ── Footer: sync button ────────────────────────────────────────
        footer = ctk.CTkFrame(self, corner_radius=0,
                              fg_color=("gray85", "gray15"), height=64)
        footer.grid(row=2, column=0, sticky="ew")
        footer.grid_columnconfigure(0, weight=1)
        footer.grid_propagate(False)

        self._sync_btn = ctk.CTkButton(
            footer, text="SYNC", width=180, height=38,
            font=("Segoe UI", 15, "bold"),
            command=self._on_sync)
        self._sync_btn.grid(row=0, column=0, pady=12)

        # ── Refresh port lists on focus ────────────────────────────────
        self.bind("<FocusIn>", self._on_focus)

    # ── Event handlers ─────────────────────────────────────────────────

    def _on_focus(self, _event) -> None:
        self._pane_left.refresh_ports()
        self._pane_right.refresh_ports()

    def _on_sync(self) -> None:
        """Reset both offsets so the next received timestamp becomes T=0."""
        with self._state_left.lock:
            self._state_left.offset_us = None
            self._state_left.latest_us = None
        with self._state_right.lock:
            self._state_right.offset_us = None
            self._state_right.latest_us = None

        self._pane_left.set_clock(None)
        self._pane_right.set_clock(None)
        self._samples_left.clear()
        self._samples_right.clear()
        self._drift_filtered = None  # reset filter on re-sync
        self._drift_offset_us = 0.0  # will be set from first stable fit
        self._drift_label.configure(
            text="Drift:  —", text_color=COLOR_DISCONNECTED)
        self._syncing = True

    # ── Update loop ────────────────────────────────────────────────────

    def _schedule_update(self) -> None:
        self._update_loop()
        self.after(UPDATE_MS, self._schedule_update)

    def _update_loop(self) -> None:
        # Drain the queue
        while True:
            try:
                side_id, us_val, host_time = self._result_queue.get_nowait()
            except queue.Empty:
                break

            if side_id == 0:
                state   = self._state_left
                pane    = self._pane_left
                samples = self._samples_left
            else:
                state   = self._state_right
                pane    = self._pane_right
                samples = self._samples_right

            if us_val is None:
                # Disconnect event from polling thread
                pane._do_disconnect()
                continue

            with state.lock:
                if state.offset_us is None:
                    state.offset_us = us_val  # first sample after sync = T=0
                state.latest_us = us_val
            samples.append((host_time, us_val))

        # Compute elapsed for each side and render
        elapsed_left  = self._elapsed(self._state_left)
        elapsed_right = self._elapsed(self._state_right)

        self._pane_left.set_clock(elapsed_left)
        self._pane_right.set_clock(elapsed_right)

        # Drift — computed via linear fit, not raw subtraction
        raw_drift_us = self._compute_drift()
        if raw_drift_us is not None:
            if self._drift_filtered is None:
                # First stable reading — capture it as the zero offset
                self._drift_offset_us = raw_drift_us
                self._drift_filtered  = 0.0
            else:
                self._drift_filtered += self._drift_alpha * (
                    (raw_drift_us - self._drift_offset_us) - self._drift_filtered)
            drift_us  = self._drift_filtered
            sign      = "+" if drift_us >= 0 else ""
            text      = f"Drift:  {sign}{drift_us:.1f} \u00b5s"
            abs_us    = abs(drift_us)
            if abs_us < 100:
                color = COLOR_DRIFT_OK
            elif abs_us < 1_000:
                color = COLOR_DRIFT_WARN
            else:
                color = COLOR_DRIFT_BAD
            self._drift_label.configure(text=text, text_color=color)

    def _compute_drift(self) -> Optional[float]:
        """
        Fit a linear model device_us = a * host_time + b for each side using
        the rolling sample window, then evaluate both models at a common
        host time.  The difference is pure clock-frequency drift with the
        inter-poll delay already cancelled out.
        Returns drift in microseconds (left − right), or None if not enough data.
        """
        if (len(self._samples_left)  < _MIN_FIT_SAMPLES or
                len(self._samples_right) < _MIN_FIT_SAMPLES):
            return None

        offset_L = self._state_left.offset_us
        offset_R = self._state_right.offset_us
        if offset_L is None or offset_R is None:
            return None

        tL, yL = zip(*self._samples_left)
        tR, yR = zip(*self._samples_right)

        # Normalise host time to the earliest anchor to keep floats well-conditioned
        t0   = min(tL[0], tR[0])
        tL_n = np.array(tL) - t0
        tR_n = np.array(tR) - t0

        pL = np.polyfit(tL_n, yL, 1)
        pR = np.polyfit(tR_n, yR, 1)

        t_eval = time.perf_counter() - t0
        est_L  = np.polyval(pL, t_eval) - offset_L
        est_R  = np.polyval(pR, t_eval) - offset_R

        return float(est_L - est_R)

    @staticmethod
    def _elapsed(state: SideState) -> Optional[int]:
        with state.lock:
            if state.latest_us is None or state.offset_us is None:
                return None
            return state.latest_us - state.offset_us

    # ── Graceful shutdown ──────────────────────────────────────────────

    def on_closing(self) -> None:
        for state in (self._state_left, self._state_right):
            state.running = False
            try:
                if state.conn and state.conn.is_open:
                    state.conn.close()
            except Exception:
                pass
        self.destroy()


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()
