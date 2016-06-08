import polygonsToGraph
from Graph import Graph, Point, Node
import AStar
import visualizationTool
import serializer

points = [Point(600, 500), Point(600, 510), Point(610, 500), Point(610, 510)]
polygons = [polygonsToGraph.Polygon(points)]
start = Point(51,55)
graph = polygonsToGraph.translateToGraph(polygons)
graph.setStartNode(start)
graph.setNodeType(24, -1)
graph.setNodeType(46, -1)
graph.setNodeType(68, -1)
visualizationTool.visualizeGraph(graph, polygons)
astar = AStar.AStar(graph)
path = astar.run()
visualizationTool.visualizeResultFromNodes(graph, path, polygons)
serializer.saveGraphToFile("test.txt", graph)
newgraph = serializer.getGraphFromFile("test.txt")
visualizationTool.visualizeGraph(newgraph, polygons)

