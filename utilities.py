# Grid
nodes = []
rows = 20
cols = 20
s = 35
potential_neighbours = [(0, -1), (-1, 0), (1, 0), (0, 1)]
potential_neighbours_diag = [(-1, -1), (-1, 1), (1, -1), (1, 1)]

# UI
ui_height = 200

# Time
step_delay = 0.001

# Colours
colour_path = '#F5AF02'
colour_frontier = '#86B817'
colour_visited = "#4285F4"
colour_start = '#0F9D58'
colour_end = '#DB4437'
colour_wall = 'black'
colour_empty = 'white'

# Mouse button states
mouse_down = False
mouse_up = True