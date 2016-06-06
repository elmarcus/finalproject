import pickle, math
from cStringIO import StringIO

class Graph:
	
	def __init__(self):
		self.nodes = []
		self.start = -1
		self.goal = -1
		
	def importFromFile(self, filepath):
		output = open(filepath, 'wb')
		pickle.dump(self, output)
		output.close()
		
	def getNodeById(self, n_id):
		return next((x for x in self.nodes if x.n_id == n_id), None)
		
	def setNodeType(self, n_id, n_type) :
		self.nodes.n_id = n_type
		
	def getNodeNearPt(self, point):
		if (len(self.nodes) < 2 or len(self.nodes[0].neighbors) < 1):
			return -1
		edgeLength = self.nodes[0].center.getDistanceTo(self.getNodeById(self.nodes[0].neighbors[0]).center)
		for node in self.nodes:
			if (node.center.getDistanceTo(point) <= edgeLength):
				print "start assigned"
				return node.n_id
		return -1

	def setStartNode(self, point):
		self.start = self.getNodeNearPt(point)		
		
	def saveToFile(self, path):
		f = open(path, 'w')
		picklestring = pickle.dumps(self)
		f.write(picklestring)
		
		
		

class Point:
	
	def __init__(self, x, y):
		self.x = x
		self.y = y
	
	def getDistanceTo(self, point):
		return math.sqrt(math.pow(self.x - point.x, 2) + math.pow(self.y - point.y, 2) )
		
class Node:
	
	def __init__(self, n_id, point, n_type):
		self.n_id = n_id
		self.center = point
		self.neighbors = []
		self.n_type = n_type
		
