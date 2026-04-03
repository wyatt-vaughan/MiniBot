# =============================================================================
# config.py  —  MiniBot Chess Swarm Coordinator
# All constants and "magic numbers" live here, broken out by section.
# =============================================================================

# ---------------------------------------------------------------------------
# BOARD — physical geometry (all units in mm unless noted)
# ---------------------------------------------------------------------------
class BOARD:
    SQUARE_SIZE_MM         = 50       # one chess square side length
    NUM_SQUARES            = 8        # 8×8 board
    PLAYING_AREA_MM        = SQUARE_SIZE_MM * NUM_SQUARES  # 400 mm

    BORDER_TOP_MM          = 25
    BORDER_BOTTOM_MM       = 25
    BORDER_LEFT_MM         = 125
    BORDER_RIGHT_MM        = 125

    # Total canvas in mm (logical coordinate space)
    CANVAS_WIDTH_MM        = PLAYING_AREA_MM + BORDER_LEFT_MM + BORDER_RIGHT_MM  # 650
    CANVAS_HEIGHT_MM       = PLAYING_AREA_MM + BORDER_TOP_MM  + BORDER_BOTTOM_MM  # 450

    # Coordinate origin: (0,0) = bottom-left corner of the playing area
    # X increases → right, Y increases → up (standard math orientation)

    OUTLINE_THICKNESS_MM   = 2.0      # hard outer border line width
    GRID_LINE_THICKNESS_MM = 0.5


# ---------------------------------------------------------------------------
# PIECES — IDs, geometry, home positions
# ---------------------------------------------------------------------------
class PIECES:
    # ID ranges
    WHITE_ID_START         = 0x01
    WHITE_ID_END           = 0x11     # 17 white pieces (0x01–0x11)
    BLACK_ID_START         = 0x12
    BLACK_ID_END           = 0x22     # 17 black pieces (0x12–0x22)
    TOTAL_PIECES           = 34

    # Piece rank characters used for labels
    RANK_CHAR = {
        'king':   'K',
        'queen':  'Q',
        'rook':   'R',
        'bishop': 'B',
        'knight': 'N',
        'pawn':   'P',
    }

    CIRCLE_RADIUS_MM       = 20       # piece circle radius
    ORIENTATION_LINE_MM    = 16       # length of orientation marker from center
    ORIENTATION_LINE_WIDTH_MM = 1.5

    LABEL_FONT_SIZE_PT     = 9
    ID_FONT_SIZE_PT        = 6

    # Home positions: maps piece ID (int) → (x_mm, y_mm, theta_deg)
    # x,y are center of piece; theta=0 means facing +Y (toward opponent)
    # Origin (0,0) is bottom-left corner of the PLAYING AREA.
    # Squares are labeled a-h (columns, x) and 1-8 (rows, y).
    # Square center col c, row r  →  x = (c-0.5)*SQUARE_SIZE, y = (r-0.5)*SQUARE_SIZE
    _S = 50  # shorthand for SQUARE_SIZE_MM

    # White pieces (IDs 0x01–0x11)
    # Standard back rank: R  N  B  Q  K  B  N  R  (cols 1–8)
    # Extra queen staged in left border (off-board, x < 0)
    HOME_POSITIONS = {
        # White pawns  (IDs 0x01–0x08, row 2)
        0x01: ( 1*_S - _S//2,  2*_S - _S//2,   0),
        0x02: ( 2*_S - _S//2,  2*_S - _S//2,   0),
        0x03: ( 3*_S - _S//2,  2*_S - _S//2,   0),
        0x04: ( 4*_S - _S//2,  2*_S - _S//2,   0),
        0x05: ( 5*_S - _S//2,  2*_S - _S//2,   0),
        0x06: ( 6*_S - _S//2,  2*_S - _S//2,   0),
        0x07: ( 7*_S - _S//2,  2*_S - _S//2,   0),
        0x08: ( 8*_S - _S//2,  2*_S - _S//2,   0),
        # White back rank  (IDs 0x09–0x10)
        0x09: ( 1*_S - _S//2,  1*_S - _S//2,   0),  # Rook a1
        0x0A: ( 2*_S - _S//2,  1*_S - _S//2,   0),  # Knight b1
        0x0B: ( 3*_S - _S//2,  1*_S - _S//2,   0),  # Bishop c1
        0x0C: ( 4*_S - _S//2,  1*_S - _S//2,   0),  # Queen d1
        0x0D: ( 5*_S - _S//2,  1*_S - _S//2,   0),  # King e1
        0x0E: ( 6*_S - _S//2,  1*_S - _S//2,   0),  # Bishop f1
        0x0F: ( 7*_S - _S//2,  1*_S - _S//2,   0),  # Knight g1
        0x10: ( 8*_S - _S//2,  1*_S - _S//2,   0),  # Rook h1
        # White extra queen — staged in left border (off-board)
        0x11: (-40,             1*_S - _S//2,   0),

        # Black pawns  (IDs 0x12–0x19, row 7)
        0x12: ( 1*_S - _S//2,  7*_S - _S//2, 180),
        0x13: ( 2*_S - _S//2,  7*_S - _S//2, 180),
        0x14: ( 3*_S - _S//2,  7*_S - _S//2, 180),
        0x15: ( 4*_S - _S//2,  7*_S - _S//2, 180),
        0x16: ( 5*_S - _S//2,  7*_S - _S//2, 180),
        0x17: ( 6*_S - _S//2,  7*_S - _S//2, 180),
        0x18: ( 7*_S - _S//2,  7*_S - _S//2, 180),
        0x19: ( 8*_S - _S//2,  7*_S - _S//2, 180),
        # Black back rank  (IDs 0x1A–0x21)
        0x1A: ( 1*_S - _S//2,  8*_S - _S//2, 180),  # Rook a8
        0x1B: ( 2*_S - _S//2,  8*_S - _S//2, 180),  # Knight b8
        0x1C: ( 3*_S - _S//2,  8*_S - _S//2, 180),  # Bishop c8
        0x1D: ( 4*_S - _S//2,  8*_S - _S//2, 180),  # Queen d8
        0x1E: ( 5*_S - _S//2,  8*_S - _S//2, 180),  # King e8
        0x1F: ( 6*_S - _S//2,  8*_S - _S//2, 180),  # Bishop f8
        0x20: ( 7*_S - _S//2,  8*_S - _S//2, 180),  # Knight g8
        0x21: ( 8*_S - _S//2,  8*_S - _S//2, 180),  # Rook h8
        # Black extra queen — staged in left border (off-board)
        0x22: (-40,             8*_S - _S//2, 180),
    }

    # Rank assignments per piece ID
    PIECE_RANKS = {
        # White
        0x01: 'pawn', 0x02: 'pawn', 0x03: 'pawn', 0x04: 'pawn',
        0x05: 'pawn', 0x06: 'pawn', 0x07: 'pawn', 0x08: 'pawn',
        0x09: 'rook',   0x0A: 'knight', 0x0B: 'bishop', 0x0C: 'queen',
        0x0D: 'king',   0x0E: 'bishop', 0x0F: 'knight', 0x10: 'rook',
        0x11: 'queen',  # extra queen
        # Black
        0x12: 'pawn', 0x13: 'pawn', 0x14: 'pawn', 0x15: 'pawn',
        0x16: 'pawn', 0x17: 'pawn', 0x18: 'pawn', 0x19: 'pawn',
        0x1A: 'rook',   0x1B: 'knight', 0x1C: 'bishop', 0x1D: 'queen',
        0x1E: 'king',   0x1F: 'bishop', 0x20: 'knight', 0x21: 'rook',
        0x22: 'queen',  # extra queen
    }

    # Color assignment per ID range
    WHITE_IDS = set(range(0x01, 0x12))
    BLACK_IDS = set(range(0x12, 0x23))


# ---------------------------------------------------------------------------
# COMM — serial / USB protocol constants
# ---------------------------------------------------------------------------
class COMM:
    DEFAULT_BAUD_RATE      = 115200
    DEFAULT_POLL_INTERVAL_MS = 1000   # milliseconds between auto-polls
    ENCODING               = 'ascii'
    TERMINATOR             = '\n'

    # Command prefixes (host → ESP32)
    CMD_MOVE               = 'MOV'
    CMD_HOME               = 'HOME'
    CMD_POLL               = 'POLL'
    CMD_RATE               = 'RATE'
    CMD_MAG                = 'MAG'

    # Response prefixes (ESP32 → host)
    RESP_POSITION          = 'POS'
    RESP_ACK               = 'ACK'
    RESP_ERROR             = 'ERR'
    RESP_DONE              = 'DONE'

    # Electromagnet mode values
    MAG_OFF                = 0
    MAG_ON                 = 1
    MAG_SYNC               = 2

    DELIMITER              = ','

    # Serial read timeout (seconds) — controls how often worker checks stop flag
    READ_TIMEOUT_S         = 0.05


# ---------------------------------------------------------------------------
# GUI — colors, fonts, window dimensions
# ---------------------------------------------------------------------------
class GUI:
    WINDOW_TITLE           = 'MiniBot Chess Swarm Coordinator'
    WINDOW_MIN_WIDTH       = 1300
    WINDOW_MIN_HEIGHT      = 750

    SCALE_FACTOR           = 1.5     # px per mm (canvas logical → screen pixels)

    # Chessboard square colors
    LIGHT_SQUARE_COLOR     = '#F0D9B5'
    DARK_SQUARE_COLOR      = '#B58863'

    # Border / background
    BOARD_BACKGROUND_COLOR = '#2C2C2C'
    BOARD_OUTLINE_COLOR    = '#111111'

    # Piece colors
    WHITE_PIECE_FILL       = '#EFEFEF'
    WHITE_PIECE_OUTLINE    = '#222222'
    BLACK_PIECE_FILL       = '#333333'
    BLACK_PIECE_OUTLINE    = '#EFEFEF'

    # Selection / target indicators
    SELECTED_HIGHLIGHT_COLOR   = '#FFD700'   # gold
    TARGET_INDICATOR_COLOR     = '#00BFFF'   # deep sky blue
    ORIENTATION_LINE_COLOR_WHITE = '#222222'
    ORIENTATION_LINE_COLOR_BLACK = '#EFEFEF'

    # Staging area label color
    STAGED_PIECE_OPACITY   = 180             # 0–255 alpha

    # Control panel
    CONTROL_PANEL_MIN_WIDTH = 420

    # Position tracker table columns
    TRACKER_COLUMNS        = ['ID', 'Color', 'Rank', 'X (mm)', 'Y (mm)', 'θ (°)', 'Last Update']


# ---------------------------------------------------------------------------
# PLANNING — path planner registry
# ---------------------------------------------------------------------------
class PLANNING:
    # Each entry: display name → dotted module path: class name
    PLANNERS = {
        'Direct (straight-line)':  ('planning.direct_planner',  'DirectPlanner'),
        'Queued (sequential)':     ('planning.queued_planner',  'QueuedPlanner'),
    }

    DEFAULT_PLANNER        = 'Direct (straight-line)'

    # Move duration defaults
    DEFAULT_MOVE_DURATION_MS = 3000
    HOME_MOVE_DURATION_MS    = 5000


# ---------------------------------------------------------------------------
# CHESS — rules engine integration stubs
# ---------------------------------------------------------------------------
class CHESS:
    # Default starting FEN
    STARTING_FEN = 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1'

    # When a chess rules engine is integrated, set this to the engine adapter
    # callable: validate_move(fen: str, move_uci: str) -> bool
    # Leave as None to disable rules enforcement.
    RULES_ENGINE_ADAPTER   = None


# ---------------------------------------------------------------------------
# SIMULATOR — software-in-the-loop motion simulator
# ---------------------------------------------------------------------------
class SIMULATOR:
    # Nominal robot speed used when simulator is active
    DEFAULT_SPEED_MM_S     = 80.0

    # Timer tick rate for simulated motion (20 Hz)
    UPDATE_INTERVAL_MS     = 50

    # Maximum rotation rate in degrees per second
    ROTATION_SPEED_DEG_S   = 90.0

    # Heading must be within this many degrees of the target bearing before
    # the robot begins translating (differential drive constraint).
    HEADING_TOLERANCE_DEG  = 3.0

    # Extra clearance added to 2×radius for piece–piece collision detection
    COLLISION_MARGIN_MM    = 2.0

    # Physical boundary of the whole table in playing-area mm coordinates.
    # Pieces must stay within these limits (centre must be ≥ radius from wall).
    # Derived from BOARD constants — edit BOARD values to change these.
    X_MIN_MM = -BOARD.BORDER_LEFT_MM                              # -125
    X_MAX_MM =  BOARD.PLAYING_AREA_MM + BOARD.BORDER_RIGHT_MM    #  525
    Y_MIN_MM = -BOARD.BORDER_BOTTOM_MM                            #  -25
    Y_MAX_MM =  BOARD.PLAYING_AREA_MM + BOARD.BORDER_TOP_MM      #  425
