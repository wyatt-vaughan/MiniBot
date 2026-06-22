"""
engines/base_engine.py

Base interface and factory for chess rules engines.
"""

from __future__ import annotations

import importlib
from abc import ABC, abstractmethod

from config import CHESS


class BaseChessEngine(ABC):
    """Common interface implemented by all chess rules engines."""

    @property
    def name(self) -> str:
        """Human-readable engine name."""
        return self.__class__.__name__

    @abstractmethod
    def set_fen(self, fen_string: str) -> tuple[bool, str | None]:
        """Load a chess position from FEN."""
        raise NotImplementedError

    @abstractmethod
    def get_fen(self) -> str:
        """Return the engine's current FEN."""
        raise NotImplementedError

    @abstractmethod
    def validate_move(self, fen_string: str, uci_move: str) -> bool:
        """Return whether a UCI move is legal for the supplied position."""
        raise NotImplementedError

    def __call__(self, fen_string: str, uci_move: str) -> bool:
        """Allow the engine itself to be passed as a validation callback."""
        return self.validate_move(fen_string, uci_move)


def load_engine(display_name: str) -> BaseChessEngine:
    """Instantiate a chess engine from the registry in config.py.

    Args:
        display_name: A key from CHESS.ENGINES.

    Returns:
        An initialized chess-engine instance.

    Raises:
        KeyError: The display name is not registered.
        ImportError: The configured module cannot be imported.
        AttributeError: The configured class does not exist.
        TypeError: The class does not inherit BaseChessEngine.
    """
    module_path, class_name = CHESS.ENGINES[display_name]

    module = importlib.import_module(module_path)
    engine_class = getattr(module, class_name)

    engine = engine_class()

    if not isinstance(engine, BaseChessEngine):
        raise TypeError(
            f"{class_name} must inherit from BaseChessEngine"
        )

    return engine