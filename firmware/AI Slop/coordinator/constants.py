"""
Constants for Chess Robot Coordinator
"""

# Board dimensions
BOARD_SQUARE_SIZE = 50  # mm
BOARD_EXTRA_SIDE = 100  # mm on each side
PIECE_RADIUS = 15  # 30mm OD = 15mm radius

# Movement parameters
ANGULAR_VELOCITY = 180  # degrees per second
LINEAR_VELOCITY = 300  # mm per second

# Chess pieces: 8 pawns per side (row 1 & 6) + 8 major pieces per side (row 0 & 7)
# White pieces (bottom): pawns a2-h2, back row a1-h1
# Black pieces (top): pawns a7-h7, back row a8-h8
# Format: piece_id: (column, row)
PIECE_START_POSITIONS = {
    # White pawns (row 1)
    'p1': (0, 1), 'p2': (1, 1), 'p3': (2, 1), 'p4': (3, 1),
    'p5': (4, 1), 'p6': (5, 1), 'p7': (6, 1), 'p8': (7, 1),
    # White major pieces (row 0) - a1-h1
    'r1': (0, 0), 'n1': (1, 0), 'b1': (2, 0), 'q': (3, 0),
    'k': (4, 0), 'b2': (5, 0), 'n2': (6, 0), 'r2': (7, 0),
    # Black pawns (row 6)
    'P1': (0, 6), 'P2': (1, 6), 'P3': (2, 6), 'P4': (3, 6),
    'P5': (4, 6), 'P6': (5, 6), 'P7': (6, 6), 'P8': (7, 6),
    # Black major pieces (row 7) - a8-h8
    'R1': (0, 7), 'N1': (1, 7), 'B1': (2, 7), 'Q': (3, 7),
    'K': (4, 7), 'B2': (5, 7), 'N2': (6, 7), 'R2': (7, 7),
}

# Intermediate targets for pathing to reduce collisions
PIECE_INTERMEDIATE_POSITIONS = {
    # White pawns (row 1)
    'p1': (0, 1), 'p2': (0, 2), 'p3': (3, 1), 'p4': (3, 2),
    'p5': (4, 1), 'p6': (4, 2), 'p7': (7, 1), 'p8': (7, 2),
    # White major pieces (row 0) - a1-h1
    'r1': (0, 0), 'n1': (1, 0), 'b1': (2, 0), 'q': (3, 0),
    'k': (4, 0), 'b2': (5, 0), 'n2': (6, 0), 'r2': (7, 0),
    # Black pawns (row 6)
    'P1': (0, 6), 'P2': (0, 5), 'P3': (3, 6), 'P4': (3, 5),
    'P5': (4, 6), 'P6': (4, 5), 'P7': (7, 6), 'P8': (7, 5),
    # Black major pieces (row 7) - a8-h8
    'R1': (0, 7), 'N1': (1, 7), 'B1': (2, 7), 'Q': (3, 7),
    'K': (4, 7), 'B2': (5, 7), 'N2': (6, 7), 'R2': (7, 7),
}

# UI dimensions
WINDOW_WIDTH = 1300
WINDOW_HEIGHT = 800
BOARD_DISPLAY_OFFSET_X = 150
BOARD_DISPLAY_OFFSET_Y = 100
BOARD_DISPLAY_SQUARE_SIZE = 70  # pixels for display
