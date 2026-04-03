"""
Chess Robot Coordinator - Updated Piece Layout

Full 32-piece setup with proper chess piece placement
"""

BOARD_LAYOUT = """
CHESS ROBOT BOARD - STARTING POSITIONS
(Viewed from white's perspective)

ROW 7: r1    n1    b1    q     k     b2    n2    r2    (Black Back Row)
ROW 6: P1    P2    P3    P4    P5    P6    P7    P8    (Black Pawns)
ROW 5: [     ]     [     ]     [     ]     [     ]     (Empty)
ROW 4: [     ]     [     ]     [     ]     [     ]     (Empty)
ROW 3: [     ]     [     ]     [     ]     [     ]     (Empty)
ROW 2: [     ]     [     ]     [     ]     [     ]     (Empty)
ROW 1: p1    p2    p3    p4    p5    p6    p7    p8    (White Pawns)
ROW 0: R1    N1    B1    Q     K     B2    N2    R2    (White Back Row)
      -----col----col----col----col----col----col----col----col-----
       0     1     2     3     4     5     6     7

PIECE TYPES:
White Major Pieces (Row 0):
  R1 = Rook (left)         at (0, 0)
  N1 = Knight (left)       at (1, 0)
  B1 = Bishop (left)       at (2, 0)
  Q = Queen                at (3, 0)
  K = King                 at (4, 0)
  B2 = Bishop (right)      at (5, 0)
  N2 = Knight (right)      at (6, 0)
  R2 = Rook (right)        at (7, 0)

White Pawns (Row 1):
  p1-p8 = Pawns            at (0-7, 1)

Black Pawns (Row 6):
  P1-P8 = Pawns            at (0-7, 6)

Black Major Pieces (Row 7):
  r1 = Rook (left)         at (0, 7)
  n1 = Knight (left)       at (1, 7)
  b1 = Bishop (left)       at (2, 7)
  q = Queen                at (3, 7)
  k = King                 at (4, 7)
  b2 = Bishop (right)      at (5, 7)
  n2 = Knight (right)      at (6, 7)
  r2 = Rook (right)        at (7, 7)

TOTAL: 32 pieces (16 white, 16 black)


WORLD COORDINATES (in millimeters):
Position formula: board_coords_to_world(col, row)
  x = 100 + col * 55 + 27.5
  y = row * 55 + 27.5

Examples:
  (0, 0) → (127.5, 27.5)      White Rook
  (3, 0) → (292.5, 27.5)      White Queen
  (0, 1) → (127.5, 82.5)      White Pawn
  (7, 7) → (517.5, 412.5)     Black Rook


SIMULATOR STATE:
  Total pieces: 32
  Active pieces per side: 16
  Movement types: Rotation, Straight line, Arc
  Collision detection: Enabled (5mm safety margin)
  Movement interpolation: Distance-based smooth curves


CODE REFERENCES:
PIECE_START_POSITIONS dict - Full piece definitions
board_coords_to_world() - Convert board coords to world coords
SimulatorEngine.initialize_board() - Creates 32 pieces
SequentialPathPlanner - Plans paths for all 32 pieces
OptimizedPathPlanner - Plans with collision avoidance for all 32 pieces


MOVEMENT EXAMPLE:
Piece 'p1' (White Pawn at 0,1):
  Start: (127.5, 82.5)
  Target: (127.5, 200.0)
  Duration: ~3.17 seconds
  
  Motion sequence:
  1. Rotate from 0° to 90° (2 seconds) 
  2. Move straight from (127.5, 82.5) to (127.5, 200.0) (117.5mm movement)
  3. Total: smooth interpolated path


TESTING CHECKLIST:
✓ Initialize board - all 32 pieces at correct positions
✓ Randomize positions - randomly place all 32 pieces
✓ Plan movements - generate paths for all 32 pieces
✓ Execute movements - smooth movement for all pieces
✓ Collision detection - detect collisions among 32 pieces
✓ Benchmarking - test performance with 32 pieces
"""

# All piece IDs by side
WHITE_PIECES = {
    'p1', 'p2', 'p3', 'p4', 'p5', 'p6', 'p7', 'p8',  # Pawns
    'R1', 'N1', 'B1', 'Q', 'K', 'B2', 'N2', 'R2',    # Major pieces
}

BLACK_PIECES = {
    'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8',  # Pawns
    'r1', 'n1', 'b1', 'q', 'k', 'b2', 'n2', 'r2',    # Major pieces
}

ALL_PIECES = WHITE_PIECES | BLACK_PIECES

print(BOARD_LAYOUT)
print(f"\nTotal pieces: {len(ALL_PIECES)}")
print(f"White pieces: {len(WHITE_PIECES)}")
print(f"Black pieces: {len(BLACK_PIECES)}")
