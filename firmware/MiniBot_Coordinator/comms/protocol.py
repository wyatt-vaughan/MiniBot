"""
comms/protocol.py  —  MiniBot Chess Swarm Coordinator

Human-readable CSV-over-serial protocol (newline-terminated).

Host → ESP32 commands:
    MOV,{id},{x_mm},{y_mm},{theta_deg},{duration_ms}
    HOME
    POLL
    RATE,{interval_ms}
    MAG,{0|1|2}          0=off  1=on  2=sync

ESP32 → Host responses:
    POS,{id},{x_mm},{y_mm},{theta_deg}
    ACK,{id}
    ERR,{id},{reason}
    DONE                 (terminates a POLL position dump)

All values are decimal ASCII; no binary frames.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union

from config import COMM


# ---------------------------------------------------------------------------
# Parsed message dataclass
# ---------------------------------------------------------------------------

@dataclass
class ParsedMessage:
    """Structured representation of one line received from the ESP32.

    Attributes:
        msg_type:  One of COMM.RESP_* constants ('POS', 'ACK', 'ERR', 'DONE')
                   or 'UNKNOWN' for unrecognised lines.
        piece_id:  Integer piece ID, or None for DONE / UNKNOWN.
        x_mm:      X position (mm), present for POS only.
        y_mm:      Y position (mm), present for POS only.
        theta_deg: Orientation (degrees), present for POS only.
        reason:    Error string, present for ERR only.
        raw:       The original raw string for logging / debug.
    """
    msg_type:  str
    piece_id:  Optional[int]   = None
    x_mm:      Optional[float] = None
    y_mm:      Optional[float] = None
    theta_deg: Optional[float] = None
    reason:    Optional[str]   = None
    raw:       str             = ''


# ---------------------------------------------------------------------------
# Command builders  (host → ESP32)
# ---------------------------------------------------------------------------

def build_move(
    piece_id: int,
    x_mm: float,
    y_mm: float,
    theta_deg: float,
    duration_ms: int,
) -> bytes:
    """Build a MOV command.

    Format: ``MOV,{id},{x_mm:.1f},{y_mm:.1f},{theta_deg:.1f},{duration_ms}\\n``
    """
    line = (
        f"{COMM.CMD_MOVE}"
        f"{COMM.DELIMITER}{piece_id}"
        f"{COMM.DELIMITER}{x_mm:.1f}"
        f"{COMM.DELIMITER}{y_mm:.1f}"
        f"{COMM.DELIMITER}{theta_deg:.1f}"
        f"{COMM.DELIMITER}{duration_ms}"
        f"{COMM.TERMINATOR}"
    )
    return line.encode(COMM.ENCODING)


def build_home() -> bytes:
    """Build a HOME command — instructs all pieces to return to start."""
    return f"{COMM.CMD_HOME}{COMM.TERMINATOR}".encode(COMM.ENCODING)


def build_poll() -> bytes:
    """Build a POLL command — request a position dump for all pieces."""
    return f"{COMM.CMD_POLL}{COMM.TERMINATOR}".encode(COMM.ENCODING)


def build_rate(interval_ms: int) -> bytes:
    """Build a RATE command — set auto-poll interval.

    Format: ``RATE,{interval_ms}\\n``
    """
    line = (
        f"{COMM.CMD_RATE}"
        f"{COMM.DELIMITER}{interval_ms}"
        f"{COMM.TERMINATOR}"
    )
    return line.encode(COMM.ENCODING)


def build_mag(mode: int) -> bytes:
    """Build a MAG command — control electromagnets.

    Args:
        mode: COMM.MAG_OFF (0), COMM.MAG_ON (1), or COMM.MAG_SYNC (2).

    Format: ``MAG,{mode}\\n``
    """
    if mode not in (COMM.MAG_OFF, COMM.MAG_ON, COMM.MAG_SYNC):
        raise ValueError(f"Invalid MAG mode: {mode!r}")
    line = (
        f"{COMM.CMD_MAG}"
        f"{COMM.DELIMITER}{mode}"
        f"{COMM.TERMINATOR}"
    )
    return line.encode(COMM.ENCODING)


# ---------------------------------------------------------------------------
# Response parser  (ESP32 → host)
# ---------------------------------------------------------------------------

def parse_line(raw: Union[str, bytes]) -> ParsedMessage:
    """Parse one newline-terminated line from the ESP32.

    Returns a ParsedMessage with msg_type='UNKNOWN' for lines that do not
    match the expected protocol — these are logged by the caller but do not
    raise exceptions, ensuring robustness against stray debug prints.

    Args:
        raw: A single line (str or bytes).  Trailing whitespace is stripped.

    Returns:
        ParsedMessage with populated fields.
    """
    if isinstance(raw, bytes):
        try:
            raw = raw.decode(COMM.ENCODING)
        except UnicodeDecodeError:
            return ParsedMessage(msg_type='UNKNOWN', raw=repr(raw))

    raw = raw.strip()

    if not raw:
        return ParsedMessage(msg_type='UNKNOWN', raw=raw)

    parts = raw.split(COMM.DELIMITER)
    prefix = parts[0].upper()

    try:
        if prefix == COMM.RESP_POSITION and len(parts) == 5:
            # POS,{id},{x_mm},{y_mm},{theta_deg}
            return ParsedMessage(
                msg_type  = COMM.RESP_POSITION,
                piece_id  = int(parts[1]),
                x_mm      = float(parts[2]),
                y_mm      = float(parts[3]),
                theta_deg = float(parts[4]),
                raw       = raw,
            )

        if prefix == COMM.RESP_ACK and len(parts) == 2:
            # ACK,{id}
            return ParsedMessage(
                msg_type = COMM.RESP_ACK,
                piece_id = int(parts[1]),
                raw      = raw,
            )

        if prefix == COMM.RESP_ERROR and len(parts) >= 3:
            # ERR,{id},{reason}
            reason = COMM.DELIMITER.join(parts[2:])
            return ParsedMessage(
                msg_type = COMM.RESP_ERROR,
                piece_id = int(parts[1]),
                reason   = reason,
                raw      = raw,
            )

        if prefix == COMM.RESP_DONE:
            # DONE
            return ParsedMessage(msg_type=COMM.RESP_DONE, raw=raw)

    except (ValueError, IndexError):
        pass

    return ParsedMessage(msg_type='UNKNOWN', raw=raw)
