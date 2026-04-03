"""
Helper functions for Chess Robot Coordinator
"""

from typing import Tuple
from .constants import BOARD_EXTRA_SIDE, BOARD_SQUARE_SIZE


def board_coords_to_world(col: int, row: int) -> Tuple[float, float]:
    """
    Convert chess board coordinates (column, row) to world coordinates (x, y) in mm.
    Centers the position within the square.
    
    Args:
        col: Column 0-7
        row: Row 0-7
        
    Returns:
        Tuple of (x, y) in millimeters, centered on the square
    """
    # Center of square
    x = BOARD_EXTRA_SIDE + col * BOARD_SQUARE_SIZE + BOARD_SQUARE_SIZE / 2
    y = row * BOARD_SQUARE_SIZE + BOARD_SQUARE_SIZE / 2
    return (x, y)
