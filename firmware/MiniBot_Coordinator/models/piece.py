"""
models/piece.py  —  MiniBot Chess Swarm Coordinator

Defines the Piece dataclass and BoardState container.

Chess engine integration point:
    BoardState.validate_move() is a stub that always returns True.
    To integrate a rules engine (e.g., python-chess), replace the body
    of validate_move() with a call to your engine adapter, or assign
    CHESS.RULES_ENGINE_ADAPTER in config.py.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from config import BOARD, PIECES, CHESS
from firmware.MiniBot_Coordinator.engines.base_engine import BaseChessEngine


# ---------------------------------------------------------------------------
# Piece
# ---------------------------------------------------------------------------

@dataclass
class Piece:
    """Represents a single chess robot on the board.

    Attributes:
        piece_id:      Unique hardware ID (0x01–0x22).
        color:         'white' or 'black'.
        rank:          Piece type string, e.g. 'pawn', 'queen'.
        position_mm:   (x_mm, y_mm) center of piece in playing-area coordinates.
                       Origin (0,0) is the bottom-left corner of the playing
                       area. x increases rightward, y increases upward.
        orientation_deg: Heading in degrees (0 = facing +Y / toward opponent).
        is_captured:   True once the piece has been removed from play.
        is_staged:     True when the piece is in the off-board staging zone.
        last_updated:  Unix timestamp of the most recent position update.
    """
    piece_id:        int
    color:           str          # 'white' | 'black'
    rank:            str          # 'pawn' | 'rook' | 'knight' | 'bishop' | 'queen' | 'king'
    position_mm:     Tuple[float, float] = field(default_factory=lambda: (0.0, 0.0))
    orientation_deg: float = 0.0
    battery_v:       float = 0.0
    is_captured:     bool  = False
    is_staged:       bool  = False
    last_updated:    float = field(default_factory=time.time)

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    @property
    def rank_char(self) -> str:
        """Single-character label for the piece rank, e.g. 'P', 'Q'."""
        return PIECES.RANK_CHAR.get(self.rank, '?')

    @property
    def id_hex(self) -> str:
        """Piece ID formatted as a two-digit hex string, e.g. '0A'."""
        return f'{self.piece_id:02X}'

    @property
    def x_mm(self) -> float:
        return self.position_mm[0]

    @property
    def y_mm(self) -> float:
        return self.position_mm[1]

    def update_position(self, x_mm: float, y_mm: float, theta_deg: float, battery_v: float = 0.0) -> None:
        """Update position, orientation and battery voltage, recording the current timestamp."""
        self.position_mm     = (x_mm, y_mm)
        self.orientation_deg = theta_deg
        self.battery_v       = battery_v
        self.last_updated    = time.time()


# ---------------------------------------------------------------------------
# BoardState
# ---------------------------------------------------------------------------

class BoardState:
    """Container for all 34 piece objects plus board-level helpers.

    Usage:
        board = BoardState()
        board.reset_to_home()          # move all pieces to starting positions
        piece = board.get_piece(0x0D)  # fetch piece by ID
    """

    def __init__(self) -> None:
        self._pieces: Dict[int, Piece] = {}
        self._rules_engine: Optional[BaseChessEngine] = None
        self._build_pieces()

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _build_pieces(self) -> None:
        """Construct all 34 Piece objects with home positions and metadata."""
        for pid in range(PIECES.WHITE_ID_START, PIECES.WHITE_ID_END + 1):
            rank  = PIECES.PIECE_RANKS[pid]
            pos   = PIECES.HOME_POSITIONS[pid]
            piece = Piece(
                piece_id       = pid,
                color          = 'white',
                rank           = rank,
                position_mm    = (float(pos[0]), float(pos[1])),
                orientation_deg= float(pos[2]),
                is_staged      = pos[0] < 0,
            )
            self._pieces[pid] = piece

        for pid in range(PIECES.BLACK_ID_START, PIECES.BLACK_ID_END + 1):
            rank  = PIECES.PIECE_RANKS[pid]
            pos   = PIECES.HOME_POSITIONS[pid]
            piece = Piece(
                piece_id       = pid,
                color          = 'black',
                rank           = rank,
                position_mm    = (float(pos[0]), float(pos[1])),
                orientation_deg= float(pos[2]),
                is_staged      = pos[0] < 0,
            )
            self._pieces[pid] = piece

    def set_rules_engine(
        self,
        engine: Optional[BaseChessEngine],
    ) -> None:
        """Set the chess rules engine used for move validation.

        Passing None disables chess-rule validation.
        """
        self._rules_engine = engine

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    def get_piece(self, piece_id: int) -> Optional[Piece]:
        return self._pieces.get(piece_id)

    def all_pieces(self) -> List[Piece]:
        return list(self._pieces.values())

    def active_pieces(self) -> List[Piece]:
        """Pieces that are on the board and not captured."""
        return [p for p in self._pieces.values() if not p.is_captured]

    def pieces_by_color(self, color: str) -> List[Piece]:
        return [p for p in self._pieces.values() if p.color == color]
    
    @property
    def rules_engine(self) -> Optional[BaseChessEngine]:
        return self._rules_engine
        
    def _mm_to_square(
        x_mm: float,
        y_mm: float,
    ) -> Optional[str]:
        """Convert playing-area millimeters to an algebraic square.

        Examples:
            (25, 25)   -> "a1"
            (225, 75)  -> "e2"
            (225, 175) -> "e4"

        Returns:
            Algebraic square name, or None when outside the playing area.
        """
        if not (
            0.0 <= x_mm < BOARD.PLAYING_AREA_MM
            and 0.0 <= y_mm < BOARD.PLAYING_AREA_MM
        ):
            return None

        file_index = int(x_mm // BOARD.SQUARE_SIZE_MM)
        rank_index = int(y_mm // BOARD.SQUARE_SIZE_MM)

        if not (
            0 <= file_index < BOARD.NUM_SQUARES
            and 0 <= rank_index < BOARD.NUM_SQUARES
        ):
            return None

        file_name = chr(ord("a") + file_index)
        rank_name = str(rank_index + 1)

        return f"{file_name}{rank_name}"
    

    @staticmethod
    def _square_to_mm(square: str) -> Optional[tuple[float, float]]:
        """Convert an algebraic chess square to its center position in mm.

        Examples:
            "a1" -> (25.0, 25.0)
            "e2" -> (225.0, 75.0)
            "e4" -> (225.0, 175.0)

        Returns:
            A tuple of (x_mm, y_mm), or None if the square is invalid.
        """
        square = square.strip().lower()

        if len(square) != 2:
            return None

        file_name = square[0]
        rank_name = square[1]

        if file_name not in "abcdefgh":
            return None

        if rank_name not in "12345678":
            return None

        file_index = ord(file_name) - ord("a")
        rank_index = int(rank_name) - 1

        half_square = BOARD.SQUARE_SIZE_MM / 2.0

        x_mm = (
            file_index * BOARD.SQUARE_SIZE_MM
            + half_square
        )

        y_mm = (
            rank_index * BOARD.SQUARE_SIZE_MM
            + half_square
        )

        return x_mm, y_mm

    # ------------------------------------------------------------------
    # State management
    # ------------------------------------------------------------------

    def reset_to_home(self) -> None:
        """Move all pieces to their standard starting positions."""
        for pid, piece in self._pieces.items():
            home = PIECES.HOME_POSITIONS[pid]
            piece.position_mm     = (float(home[0]), float(home[1]))
            piece.orientation_deg = float(home[2])
            piece.is_captured     = False
            piece.is_staged       = home[0] < 0
            piece.last_updated    = time.time()

    def update_piece_position(
        self,
        piece_id: int,
        x_mm: float,
        y_mm: float,
        theta_deg: float,
        battery_v: float = 0.0,
    ) -> None:
        """Update a single piece's position (typically called from serial handler)."""
        piece = self._pieces.get(piece_id)
        if piece is not None:
            piece.update_position(x_mm, y_mm, theta_deg, battery_v)

    # ------------------------------------------------------------------
    # Chess engine integration — FEN export
    # ------------------------------------------------------------------

    def to_fen(self) -> str:
        """Export the current physical board position as a FEN string.

        Captured, staged, and off-board pieces are excluded.

        Important:
            BoardState only knows physical piece positions. It does not currently
            know whose turn it is, castling history, en-passant targets, or move
            counters. Therefore, the returned FEN string will have placeholders for these fields
            (active color = 'w', castling rights = '-', en-passant target = '-', halfmove clock = 0, fullmove number = 1).

        Returns:
            A complete FEN string representing the current piece placement.
        """
        square_size = BOARD.SQUARE_SIZE_MM
        board_size = BOARD.NUM_SQUARES

        # board[rank][file]
        # rank 0 = rank 1
        # file 0 = file a
        squares: List[List[Optional[str]]] = [
            [None for _ in range(board_size)]
            for _ in range(board_size)
        ]

        for piece in self._pieces.values():
            # Captured and staged pieces are not part of the playable board.
            if piece.is_captured or piece.is_staged:
                continue

            x_mm, y_mm = piece.position_mm

            # Ignore anything physically outside the 8x8 playing area.
            if not (
                0.0 <= x_mm < BOARD.PLAYING_AREA_MM
                and 0.0 <= y_mm < BOARD.PLAYING_AREA_MM
            ):
                continue

            file_index = int(x_mm // square_size)
            rank_index = int(y_mm // square_size)

            piece_char = PIECES.RANK_CHAR.get(piece.rank)

            if piece_char is None:
                raise ValueError(
                    f"Unknown rank '{piece.rank}' for piece "
                    f"0x{piece.piece_id:02X}"
                )

            # FEN uses uppercase for white and lowercase for black.
            if piece.color == "black":
                piece_char = piece_char.lower()
            elif piece.color != "white":
                raise ValueError(
                    f"Unknown color '{piece.color}' for piece "
                    f"0x{piece.piece_id:02X}"
                )

            existing_piece = squares[rank_index][file_index]

            if existing_piece is not None:
                file_name = chr(ord("a") + file_index)
                rank_name = rank_index + 1

                raise ValueError(
                    f"Multiple pieces detected on {file_name}{rank_name}: "
                    f"existing '{existing_piece}', "
                    f"piece 0x{piece.piece_id:02X} '{piece_char}'"
                )

            squares[rank_index][file_index] = piece_char

        fen_ranks: List[str] = []

        
        for rank_index in range(board_size - 1, -1, -1):
            fen_rank = ""
            empty_count = 0

            for file_index in range(board_size):
                piece_char = squares[rank_index][file_index]

                if piece_char is None:
                    empty_count += 1
                    continue

                if empty_count > 0:
                    fen_rank += str(empty_count)
                    empty_count = 0

                fen_rank += piece_char

            if empty_count > 0:
                fen_rank += str(empty_count)

            fen_ranks.append(fen_rank)

        piece_placement = "/".join(fen_ranks)

        # BoardState cannot determine these from physical positions alone:
        active_color = "w"
        castling_rights = "-"
        en_passant_target = "-"
        halfmove_clock = 0
        fullmove_number = 1

        return (
            f"{piece_placement} "
            f"{active_color} "
            f"{castling_rights} "
            f"{en_passant_target} "
            f"{halfmove_clock} "
            f"{fullmove_number}"
        )

    # ------------------------------------------------------------------
    # Chess engine integration — move validation
    # ------------------------------------------------------------------

    def validate_move(
        self,
        piece_id: int,
        target_x_mm: float,
        target_y_mm: float,
    ) -> bool:
        """Validate whether a physical move is legal according to chess rules."""

        # No configured engine means rules validation is disabled.
        if self._rules_engine is None:
            return True

        uci_move = self._to_uci(
            piece_id,
            target_x_mm,
            target_y_mm,
        )

        if uci_move is None:
            return False

        fen = self.to_fen()

        return self._rules_engine.validate_move(
            fen,
            uci_move,
        )
    
    def _to_uci(
        self,
        piece_id: int,
        target_x_mm: float,
        target_y_mm: float,
    ) -> Optional[str]:
        """Convert piece ID + target mm → UCI move string (e.g. 'e2e4').

        Example:
            White e-pawn, ID 0x05:
            current position e2
            target position e4
            result: "e2e4"

        Returns:
            A UCI move string, or None when the movement cannot be converted.
        """
        piece = self.get_piece(piece_id)

        if piece is None:
            return None

        if piece.is_captured or piece.is_staged:
            return None

        from_square = self._mm_to_square(
            piece.x_mm,
            piece.y_mm,
        )

        to_square = self._mm_to_square(
            target_x_mm,
            target_y_mm,
        )

        if from_square is None or to_square is None:
            return None

        if from_square == to_square:
            return None

        uci_move = f"{from_square}{to_square}"

        # UCI promotion moves require a promotion character.
        # For now, automatically promote to a queen because we have
        # an extra staged queen for each color.
        if piece.rank == "pawn" and to_square[1] in ("1", "8"):
            uci_move += "q"

        return uci_move
