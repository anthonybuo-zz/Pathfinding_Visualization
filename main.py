from priority_queue import *
from Tkinter import *
from utilities import *
import time


class Node:
	def __init__(self, i, j, cost=1):
		self.i = i
		self.j = j
		self.x = i * s
		self.y = j * s
		self.neighbours = []  # top, down, left, right
		self.neighbours_diag = []  # neighbours + diagonals
		self.active_neighbours = []  # either self.neighbours or self.neighbours_diag
		self.cost = cost
		self.wall = False
		self.square = w.create_rectangle(self.x + 2,
		                                 self.y + 2,
		                                 self.x+s + 2,
		                                 self.y+s + 2)
		self.set_fill(colour_empty)
		# DEBUG
		# self.id_text = w.create_text(self.x + s/4, self.y + s/6, text="(%r,%r)" % (self.i, self.j))
		# self.cost_text = w.create_text(self.x + s*3/4, self.y + s/6, text=str(self.cost))


	def set_fill(self, colour):
		w.itemconfig(self.square, fill=colour)


def setup_nodes():

	# setup list of nodes
	for j in range(rows):
		nodes.append([])
		for i in range(cols):
			nodes[-1].append(Node(i, j))
			
	# setup orthogonal neighbours for each node
	for j in range(rows):
		for i in range(cols):
			for location in potential_neighbours:
				if 0 <= j + location[1] < rows and 0 <= i + location[0] < cols:
					nodes[j][i].neighbours.append(nodes[j + location[1]][i + location[0]])
					nodes[j][i].neighbours_diag.append(nodes[j + location[1]][i + location[0]])

	# setup diagonal neighbours for each node
	for j in range(rows):
		for i in range(cols):
			for location in potential_neighbours_diag:
				if 0 <= j + location[1] < rows and 0 <= i + location[0] < cols:
					nodes[j][i].neighbours_diag.append(nodes[j + location[1]][i + location[0]])

	# set start and end colours
	nodes[0][0].set_fill(colour_start)
	nodes[-1][-1].set_fill(colour_end)

					
def breadth_first_search(start, end):

	# setup BFS data structures
	frontier = []
	frontier.append(start)
	came_from = {}
	came_from[start] = None
	dont_retrace = False

	while frontier:
		# look at current node
		current = frontier.pop(0)
		if current != start and current != end:
			current.set_fill(colour_visited)

		# quit if at our goal
		if current == end:
			break

		# add neighbours to frontier and keep track of how to get to them
		for next in current.active_neighbours:
			if next.wall:
				continue
			if next not in came_from:
				frontier.append(next)
				came_from[next] = current
				if next != end:
					next.set_fill(colour_frontier)
				time.sleep(step_delay)
				w.update()

	# rebuild path to start
	if end not in came_from.keys():
		dont_retrace = True

	if not dont_retrace:
		path = []
		current = end
		while current != start:
			path.append(current)
			current = came_from[current]
		path.append(start)
		for node in path[1:-1]:  # keep start and end colours
			node.set_fill(colour_path)
			time.sleep(step_delay)
			w.update()
		
		
def dijkstra(start, end):
	frontier = PriorityQueue()
	frontier.put(start, start.cost)
	cost_so_far = {}
	cost_so_far[start] = start.cost
	came_from = {}
	came_from[start] = None
	dont_retrace = False

	while frontier:
		try:
			current = frontier.get()
		except:
			dont_retrace = True
			break
		if current != start and current != end:
			current.set_fill(colour_visited)

		if current == end:
			break

		for next in current.active_neighbours:
			if next.wall:
				continue
			new_cost = cost_so_far[current] + next.cost
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost
				frontier.put(next, new_cost)
				came_from[next] = current
				if next != end:
					next.set_fill(colour_frontier)
				time.sleep(step_delay)
				w.update()

	if not dont_retrace:
		# rebuild path to start
		path = []
		current = end
		while current != start:
			path.append(current)
			current = came_from[current]
		path.append(start)
		for node in path[1:-1]:  # keep start and end colours
			node.set_fill(colour_path)
			time.sleep(step_delay)
			w.update()


def a_star(start, end):
	frontier = PriorityQueue()
	frontier.put(start, start.cost)
	cost_so_far = {}
	cost_so_far[start] = start.cost
	came_from = {}
	came_from[start] = None
	dont_retrace = False

	while frontier:
		try:
			current = frontier.get()
		except:
			dont_retrace = True
			break
		if current != start and current != end:
			current.set_fill(colour_visited)

		if current == end:
			break

		for next in current.active_neighbours:
			if next.wall:
				continue
			new_cost = cost_so_far[current] + next.cost
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost
				frontier.put(next, new_cost + heuristic(next, end))
				came_from[next] = current
				if next != end:
					next.set_fill(colour_frontier)
				time.sleep(step_delay)
				w.update()

	if not dont_retrace:
		# rebuild path to start
		path = []
		current = end
		while current != start:
			path.append(current)
			current = came_from[current]
		path.append(start)
		for node in path[1:-1]:  # keep start and end colours
			node.set_fill(colour_path)
			time.sleep(step_delay)
			w.update()


def heuristic(a, b):
	# Manhattan distance on a square grid
	return abs(a.i - b.i) + abs(a.j - b.j)


def set_buttons(state):
	buttons = [run_button, walls_button, path_button, maze_button]
	for button in buttons:
		button.config(state=state)


def run_simulation():
	set_buttons('disabled')
	clear_path()
	# set initial node colour and active neighbours
	for row in nodes:
		for node in row:
			if not node.wall:
				node.set_fill(colour_empty)
			if check_diag.get() == 1:
				node.active_neighbours = node.neighbours_diag
			else:
				node.active_neighbours = node.neighbours

	# Simulation speed
	global step_delay
	# scale [0,10] to [0, 0.1]
	step_delay = slider.get()/200.0 + 0.001

	start = nodes[0][0]
	end = nodes[-1][-1]
	start.set_fill(colour_start)
	end.set_fill(colour_end)

	# run algorithm depending on user input
	if check_alg.get() == 1:
		breadth_first_search(start, end)
	elif check_alg.get() == 2:
		dijkstra(start, end)
	elif check_alg.get() == 3:
		a_star(start, end)

	set_buttons('normal')


def clear_walls():
	for row in nodes:
		for node in row:
			if node.wall:
				node.wall = False
				node.set_fill(colour_empty)


def clear_path():
	for row in nodes:
		for node in row:
			if not node.wall and node != nodes[0][0] and node != nodes[-1][-1]:
				node.set_fill(colour_empty)


def mouse_click(event):
	global mouse_down, mouse_up
	mouse_down, mouse_up = mouse_up, mouse_down
	selected_node = nodes[event.y/s][event.x/s]
	if not selected_node.wall:
		selected_node.wall = True
		nodes[event.y / s][event.x / s].set_fill(colour_wall)
	else:
		selected_node.wall = False
		nodes[event.y / s][event.x / s].set_fill(colour_empty)


def mouse_drag(event):
	selected_node = nodes[event.y / s][event.x / s]
	if mouse_down:
		if not selected_node.wall:
			selected_node.wall = True
			nodes[event.y / s][event.x / s].set_fill(colour_wall)


def mouse_release(event):
	global mouse_down, mouse_up
	mouse_down, mouse_up = mouse_up, mouse_down


def load_maze():


	clear_walls()
	clear_path()
	# maze automatically generated by commenting this function and uncommenting the
	# function immediately below
	maze = ([0, 7],
	        [1, 1], [1, 2], [1, 3], [1, 4], [1, 5], [1, 7], [1, 9], [1, 10], [1, 11], [1, 12], [1, 13], [1, 14], [1, 15], [1, 16], [1, 17], [1, 18],
	        [2, 1], [2, 3], [2, 5], [2, 7], [2, 9],
	        [3, 1], [3, 3], [3, 9], [3, 11], [3, 12], [3, 13], [3, 14], [3, 15], [3, 16], [3, 17], [3, 18], [3, 19],
	        [4, 1], [4, 3], [4, 4], [4, 5], [4, 6], [4, 8], [4, 9],
	        [5, 6], [5, 8], [5, 11], [5, 12], [5, 13], [5, 14], [5, 15], [5, 16], [5, 17], [5, 18],
	        [6, 1], [6, 2], [6, 3], [6, 4], [6, 5], [6, 6], [6, 8], [6, 10], [6, 11],
	        [7, 1], [7, 10], [7, 13], [7, 14], [7, 15], [7, 16], [7, 17], [7, 18],
	        [8, 1], [8, 3], [8, 6], [8, 7], [8, 8], [8, 9], [8, 10], [8, 12], [8, 13],
	        [9, 1], [9, 3], [9, 4], [9, 6], [9, 12], [9, 15], [9, 16], [9, 17], [9, 18], [9, 19],
	        [10, 4], [10, 6], [10, 8], [10, 9], [10, 10], [10, 11], [10, 12], [10, 13],
	        [11, 0], [11, 1], [11, 2], [11, 4], [11, 6], [11, 8], [11, 13], [11, 15], [11, 16], [11, 17], [11, 18], [11, 19],
	        [12, 4], [12, 6], [12, 8], [12, 10], [12, 11], [12, 13], [12, 16], [12, 17], [12, 18], [12, 19],
	        [13, 1], [13, 2], [13, 3], [13, 4], [13, 6], [13, 8], [13, 10], [13, 11], [13, 13], [13, 19],
	        [14, 1], [14, 8], [14, 10], [14, 11], [14, 13], [14, 14], [14, 15], [14, 16], [14, 17], [14, 19],
	        [15, 1], [15, 3], [15, 4], [15, 5], [15, 6], [15, 7], [15, 8], [15, 10], [15, 17],
	        [16, 1], [16, 3], [16, 10], [16, 11], [16, 12], [16, 13], [16, 14], [16, 15], [16, 17], [16, 18],
	        [17, 1], [17, 3], [17, 5], [17, 6], [17, 7], [17, 8], [17, 15],
	        [18, 1], [18, 3], [18, 5], [18, 6], [18, 7], [18, 8], [18, 9], [18, 10], [18, 11], [18, 12], [18, 13], [18, 14], [18, 15], [18, 16], [18, 17], [18, 18], [18, 19],
	        [19, 1], [19, 3])

	for space in maze:
			nodes[space[0]][space[1]].wall = True
			nodes[space[0]][space[1]].set_fill(colour_wall)


def generate_maze_array():
	# Used in development to generate a maze array
	for row in nodes:
		for node in row:
			if node.wall:
				print '[%r, %r],' % (node.j, node.i),


if __name__ == '__main__':
	master = Tk()
	w = Canvas(master, width=(rows * s + 2), height=(cols * s + 2))
	w.grid(row=0, column=0)

	# Containers
	alg_container = Frame(master)
	action_container = Frame(master)
	time_container = Frame(master)
	ui_container = Frame(master)

	# Algorithm Buttons
	check_alg = IntVar()
	check_go = IntVar()
	check_diag = IntVar()
	bfs_button = Radiobutton(master, text="Breadth-First Search", variable=check_alg, value=1)
	dij_button = Radiobutton(master, text="Dijkstra's Algorithm", variable=check_alg, value=2)
	ast_button = Radiobutton(master, text="A*                                ", variable=check_alg, value=3)
	dia_button = Checkbutton(master, text="Diagonal Movement", variable=check_diag)
	bfs_button.pack(in_=alg_container, side='top')
	dij_button.pack(in_=alg_container, side='top')
	ast_button.pack(in_=alg_container, side='top')
	dia_button.pack(in_=alg_container, side='top')

	# Action buttons
	run_button = Button(master, text="Run", command=run_simulation)
	walls_button = Button(master, text="Clear Walls", command=clear_walls)
	path_button = Button(master, text="Clear Path", command=clear_path)
	maze_button = Button(master, text="Load Maze", command=load_maze)
	run_button.pack(in_=action_container, side='left')
	walls_button.pack(in_=action_container, side='left')
	path_button.pack(in_=action_container, side='left')
	maze_button.pack(in_=action_container, side='left')

	# Slider and Label
	slider_label = Label(text='Time Step')
	slider = Scale(master, from_=0, to=10, orient=HORIZONTAL)
	slider_label.pack(in_=time_container, side='top')
	slider.pack(in_=time_container, side='top')

	# Put containers below the graphics window
	alg_container.grid(row=1, column=0, sticky=W)
	action_container.grid(row=1, column=0)
	time_container.grid(row=1, sticky=E)

	# Run the simulator
	setup_nodes()
	w.bind("<Button-1>", mouse_click)
	w.bind("<B1-Motion>", mouse_drag)
	w.bind("<ButtonRelease-1>", mouse_release)
	Button(master).wait_variable(IntVar())
