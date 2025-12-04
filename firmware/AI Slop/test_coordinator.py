#!/usr/bin/env python3
"""Simple test to verify coordinator functionality"""
import math
import random

# Import from coordinator
from coordinator import Piece, PieceState, ChessBoard, BOARD_SIZE

def test_starting_positions():
    """Test that starting positions are correct"""
    print("Testing starting positions...")
    board = ChessBoard()
    
    print(f"Total pieces: {len(board.pieces)}")
    print(f"Starting positions count: {len(board.starting_positions)}")
    
    # Check white pieces (indices 0-15)
    white_back_indices = [0, 1, 2, 3, 4, 5, 6, 7]
    white_pawn_indices = [8, 9, 10, 11, 12, 13, 14, 15]
    black_back_indices = [16, 17, 18, 19, 20, 21, 22, 23]
    black_pawn_indices = [24, 25, 26, 27, 28, 29, 30, 31]
    
    print("\nWhite back row (should be at y ~27.5):")
    for i in white_back_indices:
        piece = board.pieces[i]
        pos = board.starting_positions[i]
        print(f"  {piece.name:2s}: ({pos[0]:6.1f}, {pos[1]:6.1f})")
    
    print("\nWhite pawns (should be at y ~82.5):")
    for i in white_pawn_indices:
        piece = board.pieces[i]
        pos = board.starting_positions[i]
        print(f"  {piece.name:2s}: ({pos[0]:6.1f}, {pos[1]:6.1f})")
    
    print("\nBlack back row (should be at y ~412.5):")
    for i in black_back_indices:
        piece = board.pieces[i]
        pos = board.starting_positions[i]
        print(f"  {piece.name:2s}: ({pos[0]:6.1f}, {pos[1]:6.1f})")
    
    print("\nBlack pawns (should be at y ~357.5):")
    for i in black_pawn_indices:
        piece = board.pieces[i]
        pos = board.starting_positions[i]
        print(f"  {piece.name:2s}: ({pos[0]:6.1f}, {pos[1]:6.1f})")

if __name__ == "__main__":
    test_starting_positions()
