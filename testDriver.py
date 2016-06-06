import polygonsToGraph
from Graph import Graph, Point, Node
import AStar
import visualizationTool

points = [Point(600, 500), Point(600, 510), Point(610, 500), Point(610, 510)]
polygons = [polygonsToGraph.Polygon(points)]
start = Point(51,55)
graph = polygonsToGraph.translateToGraph(start, polygons)
visualizationTool.visualizeGraph(graph, polygons)
astar = AStar.AStar(graph)
path = astar.run()
visualizationTool.visualizeResultFromNodes(graph, path, polygons)
graph.saveToFile("test.txt")
newgraph = Graph()
newgraph.importFromFile("test.txt")
newgraph.saveToFile("test2.txt")
#TODO: visualize