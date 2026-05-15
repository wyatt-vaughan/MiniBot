"""
gui/tabs/path_planning_tab.py  —  MiniBot Chess Swarm Coordinator

Path Planning tab:
  - Algorithm selector dropdown
  - Piece selector (or "All")
  - Target X / Y inputs
  - "Plan Move" button  — runs the selected algorithm, populates move queue
  - "Return All Home"   — plans home moves for all pieces
  - Move queue list
  - "Send Commands" button — dispatches queued moves to serial handler
"""

from __future__ import annotations

from typing import Callable, Dict, List, Optional, Tuple

from PyQt6.QtCore import pyqtSignal, pyqtSlot, Qt
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QPushButton, QSizePolicy,
    QVBoxLayout, QWidget,
)

from config import PIECES, PLANNING
from models.piece import BoardState
from planning.base_planner import BasePlanner, MoveCommand, load_planner


class PathPlanningTab(QWidget):
    """Control panel tab for path planning.

    Signals:
        send_commands(list[MoveCommand])   — emitted when "Send Commands" is clicked
        plan_visualized(list, dict)        — emitted after planning to update board arrows
    """

    send_commands   = pyqtSignal(list)  # list[MoveCommand]
    plan_visualized = pyqtSignal(list, dict)  # commands, initial_positions

    def __init__(self, board_state: BoardState, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._board         = board_state
        self._move_queue:   List[MoveCommand] = []
        self._selected_id:  Optional[int]     = None  # from chessboard click
        self._viz_enabled:  bool              = True
        # Snapshot of positions at the time the last plan was generated
        self._viz_positions: Dict[int, Tuple[float, float]] = {}

        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(8)

        # --- Algorithm ---
        algo_group = QGroupBox('Algorithm')
        algo_layout = QVBoxLayout(algo_group)
        self._algo_combo = QComboBox()
        for name in PLANNING.PLANNERS:
            self._algo_combo.addItem(name)
        default_idx = self._algo_combo.findText(PLANNING.DEFAULT_PLANNER)
        if default_idx >= 0:
            self._algo_combo.setCurrentIndex(default_idx)
        algo_layout.addWidget(self._algo_combo)
        root.addWidget(algo_group)

        # --- Target ---
        target_group = QGroupBox('Target')
        tl = QVBoxLayout(target_group)

        piece_row = QHBoxLayout()
        piece_row.addWidget(QLabel('Piece:'))
        self._piece_combo = QComboBox()
        self._piece_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._rebuild_piece_combo()
        piece_row.addWidget(self._piece_combo)
        tl.addLayout(piece_row)

        xy_row = QHBoxLayout()
        xy_row.addWidget(QLabel('X (mm):'))
        self._target_x = QDoubleSpinBox()
        self._target_x.setRange(-PIECES.CIRCLE_RADIUS_MM * 3, 500.0)
        self._target_x.setDecimals(1)
        self._target_x.setValue(0.0)
        xy_row.addWidget(self._target_x)

        xy_row.addWidget(QLabel('Y (mm):'))
        self._target_y = QDoubleSpinBox()
        self._target_y.setRange(-PIECES.CIRCLE_RADIUS_MM * 3, 500.0)
        self._target_y.setDecimals(1)
        self._target_y.setValue(0.0)
        xy_row.addWidget(self._target_y)
        tl.addLayout(xy_row)

        btn_row = QHBoxLayout()
        self._btn_plan = QPushButton('Plan Move')
        self._btn_plan.clicked.connect(self._on_plan_move)
        btn_row.addWidget(self._btn_plan)

        self._btn_home = QPushButton('Return All Home')
        self._btn_home.clicked.connect(self._on_return_home)
        btn_row.addWidget(self._btn_home)
        tl.addLayout(btn_row)

        root.addWidget(target_group)

        # --- Move queue ---
        queue_group = QGroupBox('Move Queue')
        ql = QVBoxLayout(queue_group)
        self._queue_list = QListWidget()
        self._queue_list.setMaximumHeight(180)
        ql.addWidget(self._queue_list)

        clr_row = QHBoxLayout()
        self._btn_clear_queue = QPushButton('Clear Queue')
        self._btn_clear_queue.clicked.connect(self._on_clear_queue)
        clr_row.addWidget(self._btn_clear_queue)

        self._btn_clear_paths = QPushButton('Clear Paths')
        self._btn_clear_paths.clicked.connect(self._on_clear_paths)
        clr_row.addWidget(self._btn_clear_paths)

        self._chk_show_paths = QCheckBox('Show Paths')
        self._chk_show_paths.setChecked(True)
        self._chk_show_paths.toggled.connect(self._on_show_paths_toggled)
        clr_row.addWidget(self._chk_show_paths)

        clr_row.addStretch()
        ql.addLayout(clr_row)

        root.addWidget(queue_group)

        # --- Send ---
        self._btn_send = QPushButton('Send Commands')
        self._btn_send.setMinimumHeight(40)
        self._btn_send.setStyleSheet('font-weight: bold;')
        self._btn_send.clicked.connect(self._on_send)
        root.addWidget(self._btn_send)

        root.addStretch()

    # ------------------------------------------------------------------
    # Public slots (wired by MainWindow)
    # ------------------------------------------------------------------

    @pyqtSlot(int)
    def on_piece_selected(self, piece_id: int) -> None:
        """Sync piece selector when user clicks a piece on the board."""
        self._selected_id = piece_id
        idx = self._piece_combo.findData(piece_id)
        if idx >= 0:
            self._piece_combo.setCurrentIndex(idx)

    @pyqtSlot(int, float, float)
    def on_target_set(self, piece_id: int, x_mm: float, y_mm: float) -> None:
        """Receive left-click target from chessboard — sync fields only."""
        self._selected_id = piece_id
        idx = self._piece_combo.findData(piece_id)
        if idx >= 0:
            self._piece_combo.setCurrentIndex(idx)
        self._target_x.setValue(x_mm)
        self._target_y.setValue(y_mm)

    @pyqtSlot(int, float, float)
    def enqueue_from_board(self, piece_id: int, x_mm: float, y_mm: float) -> None:
        """Add a move to the queue from a right-click on the board.

        Uses the currently selected algorithm to build the MoveCommand.
        """
        piece = self._board.get_piece(piece_id)
        if piece is None:
            return

        # Sync the UI fields so the user can see what was queued
        idx = self._piece_combo.findData(piece_id)
        if idx >= 0:
            self._piece_combo.setCurrentIndex(idx)
        self._target_x.setValue(x_mm)
        self._target_y.setValue(y_mm)

        planner   = self._get_planner()
        positions = {piece_id: (piece.x_mm, piece.y_mm)}
        targets   = {piece_id: (x_mm, y_mm)}

        def _validator(pid: int, tx: float, ty: float) -> bool:
            return self._board.validate_move(pid, tx, ty)

        commands = planner.plan_moves(positions, targets, validator=_validator)
        self._enqueue(commands, snap_positions=dict(positions))

    # ------------------------------------------------------------------
    # Button handlers
    # ------------------------------------------------------------------

    def _on_plan_move(self) -> None:
        piece_id = self._piece_combo.currentData()
        if piece_id is None:
            return

        target_x = self._target_x.value()
        target_y = self._target_y.value()

        piece = self._board.get_piece(piece_id)
        if piece is None:
            return

        planner = self._get_planner()
        positions = {piece_id: (piece.x_mm, piece.y_mm)}
        targets   = {piece_id: (target_x, target_y)}
        validator = self._board.validate_move  # chess engine hook

        def _validator(pid: int, tx: float, ty: float) -> bool:
            return validator(pid, tx, ty)

        self._viz_positions = {}  # fresh snapshot for new plan
        commands = planner.plan_moves(positions, targets, validator=_validator)
        self._enqueue(commands, snap_positions=dict(positions))

    def _on_return_home(self) -> None:
        from config import PIECES as P, PLANNING as PL
        planner   = self._get_planner()
        positions = {}
        targets   = {}

        for piece in self._board.active_pieces():
            home = P.HOME_POSITIONS.get(piece.piece_id)
            if home:
                positions[piece.piece_id] = (piece.x_mm, piece.y_mm)
                targets[piece.piece_id]   = (float(home[0]), float(home[1]))

        # Override duration from config
        commands = planner.plan_moves(positions, targets)
        for cmd in commands:
            cmd.duration_ms = PL.HOME_MOVE_DURATION_MS
        self._viz_positions = {}  # fresh snapshot for new plan
        self._enqueue(commands, snap_positions=dict(positions))

    def _on_clear_queue(self) -> None:
        self._move_queue.clear()
        self._queue_list.clear()
        self.plan_visualized.emit([], {})

    def _on_clear_paths(self) -> None:
        self.plan_visualized.emit([], {})

    def _on_show_paths_toggled(self, checked: bool) -> None:
        self._viz_enabled = checked
        if not checked:
            self.plan_visualized.emit([], {})
        elif self._move_queue:
            self.plan_visualized.emit(list(self._move_queue), dict(self._viz_positions))

    def _on_send(self) -> None:
        if self._move_queue:
            self.send_commands.emit(list(self._move_queue))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_planner(self) -> BasePlanner:
        name = self._algo_combo.currentText()
        try:
            return load_planner(name)
        except (KeyError, ImportError, AttributeError):
            from planning.direct_planner import DirectPlanner
            return DirectPlanner()

    def _enqueue(self, commands: List[MoveCommand], snap_positions: Optional[Dict[int, Tuple[float, float]]] = None) -> None:
        if snap_positions and not self._viz_positions:
            self._viz_positions = dict(snap_positions)
        elif snap_positions:
            # Merge: only add positions for pieces not already in the snapshot
            for pid, pos in snap_positions.items():
                if pid not in self._viz_positions:
                    self._viz_positions[pid] = pos
        self._move_queue.extend(commands)
        for cmd in commands:
            label = (
                f"[seq {cmd.sequence_num:02d}] "
                f"0x{cmd.piece_id:02X}"
                f" → ({cmd.target_x_mm:.0f}, {cmd.target_y_mm:.0f}) mm"
                f"  {cmd.duration_ms} ms"
            )
            if cmd.planner_debug:
                label += f"  | {cmd.planner_debug}"
            self._queue_list.addItem(QListWidgetItem(label))
        if self._viz_enabled and commands:
            self.plan_visualized.emit(list(self._move_queue), dict(self._viz_positions))

    def _rebuild_piece_combo(self) -> None:
        self._piece_combo.clear()
        for piece in sorted(self._board.all_pieces(), key=lambda p: p.piece_id):
            label = f"0x{piece.piece_id:02X}  {piece.color[0].upper()}  {piece.rank_char}"
            self._piece_combo.addItem(label, userData=piece.piece_id)
