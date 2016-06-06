import matplotlib.pyplot as plt
import matplotlib, math

def visualizeGraph(graph, poly):
	#draw nodes
	nx = []
	ny = []
	start = graph.getNodeById(graph.start).center
	goal = graph.getNodeById(graph.goal).center
	for node in graph.nodes:
		if (node.n_id == graph.start):
			start = node.center
		elif (node.n_id == graph.goal):
			goal = node.center
		else:
			nx.append(node.center.x)
			ny.append(node.center.y)
	plt.scatter(nx, ny, c=['g']*len(graph.nodes))
	
	#draw edges
	for node in graph.nodes:
		for n in node.neighbors:
			n_center = next((x for x in graph.nodes if x.n_id == n), None).center
			plt.plot([node.center.x, n_center.x], [node.center.y, n_center.y], c='g')
	
	#draw plolygons
	px = []
	py = []
	for p in poly:
		sum_x = 0
		sum_y = 0
		for point in p.points:
			px.append(point.x)
			py.append(point.y)
			sum_x += point.x
			sum_y += point.y
		px.append(sum_x/4.0)
		py.append(sum_y/4.0)
	plt.scatter(px, py, c=['b']*len(px))
	
	#draw start & goal
	plt.scatter([start.x], [start.y], c=['r'])
	plt.scatter([goal.x], [goal.y], c=['r'])

	plt.show()
	return 

####for testing		
def visualizeResult(graph, path, poly):
	#draw nodes
	nx = []
	ny = []
	start = None
	goal = None
	for node in graph.nodes:
		if (node.n_id == graph.start):
			start = node.center
		elif (node.n_id == graph.goal):
			goal = node.center
		else:
			nx.append(node.center.x)
			ny.append(node.center.y)
	plt.scatter(nx, ny, c=['g']*len(graph.nodes))
	
	#draw edges
	for node in graph.nodes:
		for n in node.neighbors:
			n_center = next((x for x in graph.nodes if x.n_id == n), None).center
			plt.plot([node.center.x, n_center.x], [node.center.y, n_center.y], c='g')
	
	#draw plolygons
	px = []
	py = []
	for p in poly:
		sum_x = 0
		sum_y = 0
		for point in p.points:
			px.append(point.x)
			py.append(point.y)
			sum_x += point.x
			sum_y += point.y
		px.append(sum_x/4.0)
		py.append(sum_y/4.0)
	plt.scatter(px, py, c=['b']*len(px))
	
	#draw start & goal
	plt.scatter([start.x], [start.y], c=['r'])
	plt.scatter([goal.x], [goal.y], c=['r'])

	#draw path
	path_x = []
	path_y = []
	for node in path:
		path_x.append(node.center.x)
		path_y.append(node.center.y)

	plt.plot(path_x , path_y, c='r')

	plt.show()
	return 

#for RRT
def visualizeTree(graph, poly):
	#draw nodes
	nx = []
	ny = []
	for node in graph.nodes:
			nx.append(node.center.x)
			ny.append(node.center.y)
	plt.scatter(nx, ny, c=['g']*len(graph.nodes))
	
	#draw edges
	for node in graph.edges:
		plt.plot([node[0].center.x, node[1].center.x], [node[0].center.y, node[1].center.y], c='g')
	
	plt.show()
	return

def visualizeResultFromNodes(graph, path, poly):
	#draw nodes
	nx = []
	ny = []
	start = None
	goal = None
	for node in graph.nodes:
		if (node.n_id == graph.start):
			start = node.center
		elif (node.n_id == graph.goal):
			goal = node.center
		else:
			nx.append(node.center.x)
			ny.append(node.center.y)
	plt.scatter(nx, ny, c=['g']*len(graph.nodes))
	
	#draw edges
	for node in graph.nodes:
		for n in node.neighbors:
			n_center = next((x for x in graph.nodes if x.n_id == n), None).center
			plt.plot([node.center.x, n_center.x], [node.center.y, n_center.y], c='g')
	
	#draw plolygons
	px = []
	py = []
	for p in poly:
		sum_x = 0
		sum_y = 0
		for point in p.points:
			px.append(point.x)
			py.append(point.y)
			sum_x += point.x
			sum_y += point.y
		px.append(sum_x/4.0)
		py.append(sum_y/4.0)
	plt.scatter(px, py, c=['b']*len(px))
	
	#draw start & goal
	plt.scatter([start.x], [start.y], c=['r'])
	plt.scatter([goal.x], [goal.y], c=['r'])

	#draw path
	prev_node = None
	for node in path:
		if (prev_node):
			plt.plot([prev_node.center.x, node.center.x], [prev_node.center.y, node.center.y], c='r')
		prev_node = node
	plt.show()
	return