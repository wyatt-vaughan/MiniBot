"""Script to extract AI_Planner from coordinator.py"""

# Read the original file
with open('coordinator.py', 'r', encoding='utf-8') as f:
    content = f.read()

# Find the AI_Planner class
start_marker = 'class AI_Planner(PathPlanner):'
end_marker = 'class OutsmartingAIPlannerV1(PathPlanner):'

start_idx = content.find(start_marker)
end_idx = content.find(end_marker)

if start_idx != -1 and end_idx != -1:
    ai_planner_code = content[start_idx:end_idx].rstrip()
    
    # Create the ai_planner.py file
    header = '''"""
AI Path Planner - Advanced collision-aware planning with intelligent routing
"""

import math
import random
import numpy as np
import itertools
from typing import Dict, List, Optional
from .path_planner import PathPlanner
from .data_types import Piece, Position, ExecutionPlan, PieceCommandSequence, CommandType
from .constants import PIECE_START_POSITIONS, PIECE_RADIUS, BOARD_SQUARE_SIZE, BOARD_EXTRA_SIDE


'''
    
    with open('coordinator/ai_planner.py', 'w', encoding='utf-8') as f:
        f.write(header + ai_planner_code)
    
    print('AI Planner extracted successfully')
else:
    print('Could not find AI_Planner class boundaries')
