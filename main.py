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
colour_visisted = color_rgb(244, 66, 134)
colour_path = color_rgb(66, 244, 134)


class Node:
	def __init__(self, i, j):
		self.i = i
		self.j = j
		self.x = i * s
		self.y = j * s
		self.neighbours = []
		self.cost = 1
		self.square = Polygon(Point(self.x,     self.y),  # top-left
		                      Point(self.x + s, self.y),  # top-right
		                      Point(self.x + s, self.y + s),  # bottom-right
		                      Point(self.x,     self.y + s))  # bottom-left
		self.title = Text(Point(self.x + s/4, self.y + s/6),
		                  "(%r,%r)" % (self.i, self.j))
		self.square.draw(window)
		self.title.draw(window)
		self.show_cost()

	def show_cost(self):
		self.cost_tag = Text(Point(self.x + s/2, self.y + s/2),
		                     str(self.cost))
		self.cost_tag.draw(window)


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


def breadth_first_search(start, end):

	# setup BFS data structures
	frontier = []
	frontier.append(start)
	came_from = {}
	came_from[start] = None

	while frontier:
		# look at current node
		current = frontier.pop(0)
		current.square.setFill('red')

		# quit if at our goal
		if current == end:
			break

		# add neighbours to frontier and keep track of how to get to them
		for next in current.neighbours:
			if next not in came_from:
				frontier.append(next)
				came_from[next] = current
				next.square.setFill('blue')

	# rebuild path to start
	path = []
	current = end
	while current != start:
		path.append(current)
		current = came_from[current]
	path.append(start)
	for node in path:
		node.square.setFill('green')


setup_nodes()
res = breadth_first_search(nodes[0][0], nodes[-1][-1])

window.getKey()
window.close()