import math, io
from Graph import Graph, Node, Point


step_x = 35.0
step_y = 30.0
left_x = 50.0
right_x = 750.0
top_y = 50.0 
bottom_y = 550.0
		
class Polygon:
	
	def __init__(self, points):
		self.points = points
	
def getBorders(polygons):
	if (len(polygons) == 0):
		return {'left_x': 0, 'right_x': 0, 'top_y': 0, 'bottom_y': 0}
	if (len(polygons[0].points) < 4):
		return {'left_x': 0, 'right_x': 0, 'top_y': 0, 'bottom_y': 0}
	left_x = polygons[0].points[0].x
	right_x = polygons[0].points[0].x
	bottom_y = polygons[0].points[0].y
	top_y = polygons[0].points[0].y
	for poly in polygons:
		for point in poly.points:
			if (point.x < left_x):
				left_x = point.x
			if (point.x > right_x):
				right_x = point.x
			if (point.y > bottom_y):
				bottom_y = point.y
			if (point.y < top_y):
				top_y = point.y
	return {'left_x': (left_x), 'right_x': (right_x), 'top_y': (top_y), 'bottom_y': (bottom_y)}
	
def getCenter(polygon):
	xSum = 0.0
	ySum = 0.0		
	for point in polygon.points:
		xSum += point.x	
		ySum += point.y
	xSum /= 4.0
	ySum /= 4.0
	return  Point(xSum, ySum)
	
def nullifyOccupied(polygons, borders, grid):
	for poly in polygons:
		left_cell = int((min(poly.points, key=lambda point: point.x).x - borders['left_x'])/ step_x)  
		right_cell = int((max(poly.points, key=lambda point: point.x).x  - borders['left_x'])/ step_x)
		bottom_cell = int((max(poly.points, key=lambda point: point.y).y  - borders['top_y'])/ step_y)
		top_cell = int((min(poly.points, key=lambda point: point.y).y  - borders['top_y'])/ step_y)
		for i in range(top_cell, bottom_cell + 1):
			for j in range(left_cell, right_cell + 1):
				grid[i][j] = None
	n_id = 0
	for i in range(0, len(grid)):
		for j in range(0, len(grid[i])):
			if (grid[i][j] != None):
				grid[i][j].n_id = n_id
				n_id += 1

def createGrid(polygons, borders):	
	grid = []
	i = borders['top_y']
	c = 0
	while i <= borders['bottom_y']:
		row = []
		j = borders['left_x']
		while j <= borders['right_x']:
			c += 1 
			point = Point(j + step_x/2.0, i + step_y/2.0)
			row.append(Node(c, point, 0))
			j += step_x
		grid.append(row)
		i += step_y
	
	return grid					

def findNeighbors(i, j, grid):
	neighbors = []
	if (i + 1 < len(grid) and grid[i+1][j] != None):
		neighbors.append(grid[i+1][j].n_id)
	if (i + 1 < len(grid) and j + 1 < len(grid[i]) and grid[i+1][j+1] != None):
		neighbors.append(grid[i+1][j+1].n_id)
	if (i + 1 < len(grid) and j - 1 >= 0 and grid[i+1][j-1] != None):
		neighbors.append(grid[i+1][j-1].n_id)
	if (i - 1 >= 0 and grid[i-1][j] != None):
		neighbors.append(grid[i-1][j].n_id)
	if (i - 1 >= 0 and j + 1 < len(grid[i]) and grid[i-1][j+1] != None):
		neighbors.append(grid[i-1][j+1].n_id)
	if (i - 1 >= 0 and j - 1 >= 0 and grid[i-1][j-1] != None):
		neighbors.append(grid[i-1][j-1].n_id)
	if (j + 1 < len(grid[i]) and grid[i][j+1] != None):
		neighbors.append(grid[i][j+1].n_id)
	if (j - 1 >= 0 and grid[i][j-1] != None):
		neighbors.append(grid[i][j-1].n_id)
			
	return neighbors
	
def getGoalNode(grid, goal):
	for i in range (0, len(grid)-1):	
		for j in range(0, len(grid[0])-1):
			if (grid[i][j].center.x <= goal.x and grid[i+1][j+1].center.x > goal.x and
			grid[i][j].center.y <= goal.y and grid[i+1][j+1].center.y > goal.y):
				print "goal assigned"
				return grid[i][j].n_id
	return -1

# start - Point (x, y)				
def translateToGraph(polygons):
	graph = Graph()
	X = []
	Y = []

	if (len(polygons) != 1):
		print "no goal"
		return graph
	goal = getCenter(polygons[0])
	
	#borders = getBorders(polygons)
	borders = {'left_x': (left_x), 'right_x': (right_x), 'top_y': (top_y), 'bottom_y': (bottom_y)}
	grid = createGrid(polygons, borders)
	
	for i in range(0, len(grid)):
		for j in range(0, len(grid[i])):
			if (grid[i][j]):
				graph.nodes.append(grid[i][j])
				neighbors = findNeighbors(i, j, grid)
				grid[i][j].neighbors = neighbors
				X.append(grid[i][j].center.x)
				Y.append(grid[i][j].center.y)

	graph.goal = getGoalNode(grid, goal)
				
	return graph

####for testing				
def getPolygonsFromFile(filename):
	polygons = []
	f = open(filename, 'r')	
	for line in f:
		points = []
		coordinates = line.split(' ')
		i = 0
		while i + 1 < len(coordinates):
			points.append(Point(float(coordinates[i]), float(coordinates[i+1])))
			i += 2
		polygons.append(Polygon(points))
	return polygons
