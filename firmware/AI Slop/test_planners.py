from coordinator.main import ChessRobotUI

ui = ChessRobotUI()
print('Available planners:')
for i, p in enumerate(ui.available_planners):
    print(f'  {i}: {p.get_name()}')
