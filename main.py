from graphics import *
from priority_queue import *

# Grid
nodes = []
rows = 10
cols = 10
s = 70
potential_neighbours = [(0, -1), (-1, 0), (1, 0), (0, 1)]

# Graphics Window
window = GraphWin("Pathfinding Visualization", rows * s, cols * s)

# Colours
colour_frontier = color_rgb(66, 134, 244)
colour_visited = color_rgb(244, 66, 134)
colour_path = color_rgb(66, 244, 134)

# Time
step_delay = 0.1


class Node:
	def __init__(self, i, j):
		self.i = i
		self.j = j
		self.x = i * s
		self.y = j * s
		self.neighbours = []
		self.cost = 1
		self.cumulative_cost_tag = Text(Point(0, 0), '')
		self.square = Polygon(Point(self.x,     self.y),  # top-left
		                      Point(self.x + s, self.y),  # top-right
		                      Point(self.x + s, self.y + s),  # bottom-right
		                      Point(self.x,     self.y + s))  # bottom-left
		self.title = Text(Point(self.x + s/4, self.y + s/6),
		                  "(%r,%r)" % (self.i, self.j))
		self.square.draw(window)
		self.title.draw(window)
		self.update_cumulative_cost('-')

	def show_cost(self):
		self.cost_tag = Text(Point(self.x + s*3/4, self.y + s/6),
		                     str(self.cost))
		self.cost_tag.draw(window)

	def update_cumulative_cost(self, cost):
		self.cumulative_cost_tag.undraw()
		self.cumulative_cost_tag = Text(Point(self.x + s/2, self.y + s/2),
		                                str(cost))
		self.cumulative_cost_tag.draw(window)


def get_click():
	click = window.getMouse()
	print int(click.x/s), int(click.y/s)


def setup_nodes():
	# setup list of nodes
	for j in range(rows):
		nodes.append([])
		for i in range(cols):
			nodes[-1].append(Node(i, j))

	# setup neighbours for each node
	for j in range(rows):
		for i in range(cols):
			for location in potential_neighbours:
				if 0 <= j + location[1] < rows and 0 <= i + location[0] < cols:
					nodes[j][i].neighbours.append(nodes[j + location[1]][i + location[0]])

	# setup high cost tiles
	forest = [[1,0], [2,0],
			  [1,1], [2,1], [3,1],
	          [1,2], [2,2],
	          [1,3],
	          [1,4],

	          [1,6], [1,7], [1,8],
	          [1,7],
	          [1,8],
	          [1,9]]
	for tile in forest:
		nodes[tile[1]][tile[0]].cost = 3
	for row in nodes:
		for node in row:
			node.show_cost()


def breadth_first_search(start, end):

	# setup BFS data structures
	frontier = []
	frontier.append(start)
	came_from = {}
	came_from[start] = None

	while frontier:
		# look at current node
		current = frontier.pop(0)
		current.square.setFill(colour_visited)

		# quit if at our goal
		if current == end:
			break

		# add neighbours to frontier and keep track of how to get to them
		for next in current.neighbours:
			if next not in came_from:
				frontier.append(next)
				came_from[next] = current
				next.square.setFill(colour_frontier)

	# rebuild path to start
	path = []
	current = end
	while current != start:
		path.append(current)
		current = came_from[current]
	path.append(start)
	for node in path:
		node.square.setFill('green')


def dijkstra(start, end):
	frontier = PriorityQueue()
	frontier.put(start, start.cost)
	cost_so_far = {}
	cost_so_far[start] = start.cost
	came_from = {}
	came_from[start] = None

	while frontier:
		current = frontier.get()
		current.square.setFill(colour_visited)

		if current == end:
			break

		for next in current.neighbours:
			new_cost = cost_so_far[current] + next.cost
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost
				frontier.put(next, new_cost)
				came_from[next] = current
				next.square.setFill(colour_frontier)

	# rebuild path to start
	path = []
	current = end
	while current != start:
		path.append(current)
		current = came_from[current]
	path.append(start)
	for node in path:
		node.square.setFill(colour_path)


def heuristic(a, b):
	# Manhattan distance on a square grid
	return abs(a.i - b.i) + abs(a.j - b.j)


def a_star(start, end):
	frontier = PriorityQueue()
	frontier.put(start, start.cost)
	cost_so_far = {}
	cost_so_far[start] = start.cost
	came_from = {}
	came_from[start] = None

	while frontier:
		current = frontier.get()
		current.square.setFill(colour_visited)

		if current == end:
			break

		for next in current.neighbours:
			new_cost = cost_so_far[current] + next.cost
			if next not in cost_so_far or new_cost < cost_so_far[next]:
				cost_so_far[next] = new_cost
				frontier.put(next, new_cost + heuristic(next, end))
				next.update_cumulative_cost(new_cost + heuristic(next, end))
				came_from[next] = current
				next.square.setFill(colour_frontier)
				time.sleep(step_delay)

	# rebuild path to start
	path = []
	current = end
	while current != start:
		path.append(current)
		current = came_from[current]
	path.append(start)
	for node in path:
		node.square.setFill(colour_path)
		time.sleep(step_delay/2)


setup_nodes()
a_star(nodes[0][0], nodes[-1][-1])
window.getKey()
window.close()