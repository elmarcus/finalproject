import pickle

def saveGraphToFile(path, graph):
	f = open(path, 'w')
	picklestring = pickle.dumps(graph)
	f.write(picklestring)
	f.close()
	
def getGraphFromFile(path):
	output = open(path, 'r')
	graph = pickle.load(output)
	output.close()
	return graph
