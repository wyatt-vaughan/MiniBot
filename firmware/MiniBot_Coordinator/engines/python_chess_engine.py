"""
engines/python_chess_engine.py

Chess rules engine backed by the python-chess package.
"""

from __future__ import annotations

import chess

from engines.base_engine import BaseChessEngine


class PythonChessEngine(BaseChessEngine):
    def __init__(self) -> None:
        self.board = chess.Board()
        self.last_error: str | None = None

    @property
    def name(self) -> str:
        return "Python Chess"

    def set_fen(self, fen_string: str) -> tuple[bool, str | None]:
        try:
            self.board.set_fen(fen_string)
            self.last_error = None
            return True, None

        except ValueError as error:
            self.last_error = str(error)
            return False, self.last_error

    def get_fen(self) -> str:
        return self.board.fen()

    def validate_move(self, fen_string: str, uci_move: str) -> bool:
        fen_valid, _ = self.set_fen(fen_string)

        if not fen_valid:
            return False

        try:
            move = chess.Move.from_uci(
                uci_move.strip().lower()
            )
        except ValueError as error:
            self.last_error = str(error)
            return False

        if move not in self.board.legal_moves:
            self.last_error = (
                f"Move {uci_move!r} is not legal "
                f"for position {fen_string!r}"
            )
            return False

        self.last_error = None
        return True

    def apply_move(self, uci_move: str) -> bool:
        """Apply a legal move to the engine's current board."""
        try:
            move = chess.Move.from_uci(
                uci_move.strip().lower()
            )
        except ValueError as error:
            self.last_error = str(error)
            return False

        if move not in self.board.legal_moves:
            self.last_error = f"Move {uci_move!r} is not legal."
            return False

        self.board.push(move)
        self.last_error = None
        return True

    def undo_move(self) -> bool:
        """Undo the last applied move."""
        if not self.board.move_stack:
            self.last_error = "There is no move to undo."
            return False

        self.board.pop()
        self.last_error = None
        return True

    def reset(self) -> None:
        """Reset to the standard starting position."""
        self.board.reset()
        self.last_error = None