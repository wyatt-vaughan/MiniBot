#!/usr/bin/env python3
"""Test script to verify starting positions"""

BOARD_SIZE = 440

def get_starting_positions():
    """Get standard chess starting positions."""
    sq = BOARD_SIZE / 8
    pos = []
    # White back row
    pos += [(i * sq + sq / 2, 0.5 * sq) for i in range(8)]
    # White pawns
    pos += [(i * sq + sq / 2, 1.5 * sq) for i in range(8)]
    # Black back row
    pos += [(i * sq + sq / 2, 7.5 * sq) for i in range(8)]
    # Black pawns
    pos += [(i * sq + sq / 2, 6.5 * sq) for i in range(8)]
    return pos

positions = get_starting_positions()
piece_names = [
    'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R',  # White back row (0-7)
    'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P',  # White pawns (8-15)
    'R', 'N', 'B', 'Q', 'K', 'B', 'N', 'R',  # Black back row (16-23)
    'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P',  # Black pawns (24-31)
]

print("Piece starting positions:")
print("Index | Piece | Color | X      | Y")
print("------|-------|-------|--------|--------")
for i, (x, y) in enumerate(positions):
    color = "White" if i < 16 else "Black"
    piece = piece_names[i]
    print(f"{i:5d} | {piece:5s} | {color:5s} | {x:6.1f} | {y:6.1f}")

print("\nBoard dimensions: 0-440x0-440")
print(f"Square size: {BOARD_SIZE / 8}")
