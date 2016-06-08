#!/usr/bin/python
import polygonsToGraph, AStar
import sys, rospy, math, datetime 
from PyQt4 import QtGui, QtCore
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import ColorRGBA, Float32, Bool
from apriltags_intrude_detector.srv import apriltags_intrude
from apriltags_intrude_detector.srv import apriltags_info
import math
from sphero_node.msg import SpheroCollision
import visualizationTool

class PotentialField:
    
	def __init__(self, points):
		self.angle=0.0
		self.defaultAngle=0.0
		xSum = 0.0
		ySum = 0.0		
		for point in points:
			xSum += point.x	
			ySum += point.y
		xSum /= 4.0
		ySum /= 4.0
		self.centerPoint = {'x': xSum, 'y': ySum}
		self.power = 0
		self.radius = math.sqrt(math.pow(points[0].x - self.centerPoint['x'], 2) + math.pow(points[0].y - self.centerPoint['y'], 2) )
        

	def __init__(self, center_pt, radius):
		self.angle=0.0
		self.defaultAngle=0.0
		self.centerPoint = {'x': center_pt.x, 'y': center_pt.y}
		self.power = 0
		self.radius = radius

	def getDistanceTo(self, robotLocation):
		return math.sqrt( math.pow(robotLocation.x - self.centerPoint['x'], 2) + math.pow(robotLocation.y - self.centerPoint['y'], 2) )

	def getAngleTo(self, robotLocation):
		return math.atan2(float(-1*(self.centerPoint['y'] - robotLocation.y)), float(self.centerPoint['x'] - robotLocation.x))

	def isInsideGoal(self, robotLocation):
		d = self.getDistanceTo(robotLocation)
		return (d < self.radius*2)
        
class AttractiveField(PotentialField):

	def __init__(self, points):
		PotentialField.__init__(self, points)
		self.alpha = 1.5

	def __init__(self, center_pt, radius):
		PotentialField.__init__(self, center_pt, radius)
		self.alpha = 1.5
		
    
	def getVelocityChange(self, robotLocation):
		d = PotentialField.getDistanceTo(self, robotLocation)
		if (d < self.radius):
			print "inside goal"
			
			return {'x': 0, 'y': 0}
		else:
			self.angle = PotentialField.getAngleTo(self, robotLocation) 
			#print self.angle*180/math.pi
			deltaX = min(max(self.alpha*(d - self.radius)*math.cos(self.angle), -80), 80)
			deltaY = min(max(self.alpha*(d - self.radius)*math.sin(self.angle), -80), 80)
            
			return {'x': deltaX, 'y': deltaY}
            

# You implement this class
class Controller:
    stop = True # This is set to true when the stop button is pressed
    field = None
    
	
    def __init__(self):
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback)
        self.collisionSub = rospy.Subscriber("collision", SpheroCollision, self.collisionCallback)
        self.close_distance = 20.0   
        self.current_node_index = 0
        self.path = []
        self.field = None
        self.graph = None
        self.astar = None
        self.location = None
        self.collision_number = -1
        self.startPosition= None
        self.generations = 1
        self.nodes_per_generation = 0
        self.stat_results = []


    def isCloseTo(self, point1, point2):
        return math.sqrt(math.pow((point1.x - point2.x),2) + math.pow((point1.y - point2.y),2)) < self.close_distance       
     
    def getVelocityChange(self, robotLocation):
        # if we already calculated the path
        if (self.field):
            # if the robot reached the field
            if (self.field.isInsideGoal(robotLocation)):
                # if this is the goal, stop
                if (self.current_node_index == len(self.path) - 1):       
                    return {'x': 0, 'y': 0}
                #otherwise, assign next node to be the field
                self.current_node_index += 1
                self.field = AttractiveField(self.path[self.current_node_index].center, self.close_distance)
            #in any case, calculate velocity based on the field position
            delta = self.field.getVelocityChange(robotLocation)
            return delta   
        else:
            return {'x': 0, 'y': 0}

    def collisionEvent(self, robotLocation):
        self.stop = True
        
        dist = float("inf")
        closest_node = self.path[self.current_node_index].n_id
        

        #KILL THE NODE
        self.graph.getNodeById(closest_node).n_type = -1
        #for n in self.graph.getNodeById(closest_node).neighbors:
        #    self.graph.setNodeType(n, -1)
        #if (self.current_node_index >= 2): 
        #    self.graph.start = self.path[self.current_node_index - 2].n_id
        self.current_node_index = 0
        self.astar = AStar.AStar(self.graph)
        self.path = self.astar.run()
        #self.path = self.path[0:self.current_node_index+1]
        #self.path.reverse()
        self.stop = False
        self.field = AttractiveField(self.path[0].center, self.close_distance)
        self.generations += 1       
    
    def distance(self, point1, node):
        return math.sqrt(math.pow(point1.x - node.center.x, 2) + math.pow(point1.y - node.center.y, 2) )

    def trackposCallback(self, msg):
        # This function is continuously 
        self.startPosition= msg

        if not self.stop:
            twist = Twist()
            vel = self.getVelocityChange(msg)
            # Change twist.linear.x to be your desired x velocity
            twist.linear.x = vel['x'] * 0.6 #modify speed based on sphero's charge
            # Change twist.linear.y to be your desired y velocity
            twist.linear.y = vel['y'] * 0.6
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0

            self.cmdVelPub.publish(twist)


    def collisionCallback(self, msg):
        #this function is continuously called
        print "COLLISION FUNCTION"
        self.collision_number += 1
        if not self.stop:
            print "HIT WALL"
            print msg
            self.collisionEvent(msg)

        print "END CALLBACK"

    def start(self):
        rospy.wait_for_service("apriltags_info")

        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()

            for i in range(len(resp.polygons)):
                # A polygon (access points using poly.points)
                poly = resp.polygons[i]
                # The polygon's id (just an integer, 0 is self.goal, all else is bad)
                t_id = resp.ids[i]


        except Exception, e:
            print "Exception: " + str(e)
        finally:
            self.stop = False
        
        self.graph = polygonsToGraph.translateToGraph(resp.polygons)
        print 'Start Position is: '+ str(self.startPosition)

        self.graph.setStartNode(self.startPosition)
       # visualizationTool.visualizeGraph(self.graph, resp.polygons)
        #polygonsToGraph.visualizeResultFromNodes(graph, self.path, resp.polygons)
        

        # calculate shortest path using A* around obstacle
        self.astar = AStar.AStar(self.graph)
        self.path = self.astar.run()

        #visualizationTool.visualizeResultFromNodes(self.graph, self.path, resp.polygons)
        # while AGENT IS NOT IN GOAL
        #   move along path
        #   if BUMP_EVENT
        #       stop sphero
        #       update graph with wall enum
        #       wall hit -- re calculate with wall as obstacle
        #   if RAMP_EVENT
        #       update graph with ramp enum
        #       increase velocity
        # END WHILE
        #
        # assign first node on the path to be the attractive field -- run path
        #

        self.field = AttractiveField(self.path[0].center, self.close_distance)



    def stop(self):
        self.stop = True



class SpheroIntrudeForm(QtGui.QWidget):
    controller = Controller()
    
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        self.resize(600, 480) 
        self.initUI()

        rospy.init_node('sphero_intrude', anonymous=True)
        self.cmdVelPub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.cmdVelSub = rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback) 

        self.trackposSub = rospy.Subscriber("tracked_pos", Pose2D, self.trackposCallback) 
       
    def initUI(self):

        self.stateLabel = QtGui.QLabel("Position")
        self.stateTextbox = QtGui.QTextEdit()
        self.stateTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), self.updateStateTextbot)     
        
        key_instruct_label = """
	Control Your Sphero!
	---------------------------
	Moving around:
	   u    i    o
	   j    k    l
	   m    ,    .
	"""
        self.keyInstructLabel = QtGui.QLabel(key_instruct_label)
        self.cmdVelLabel = QtGui.QLabel("cmd_vel")
        self.cmdVelTextbox = QtGui.QTextEdit()
        self.cmdVelTextbox.setReadOnly(True)  
        self.connect(self, QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), self.updateCmdVelTextbox)

        self.aprilTagsInfoLabel = QtGui.QLabel("april tags info")
        self.aprilTagsInfoBtn = QtGui.QPushButton("Query")
        self.aprilTagsInfoBtn.clicked.connect(self.queryAprilTagsInfo)
        self.aprilTagsTextbox = QtGui.QTextEdit()
        self.aprilTagsTextbox.setReadOnly(True)
        self.connect(self, QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), self.updateAprilTagsTextbot)

        self.aprilTagsStartBtn = QtGui.QPushButton("Start")
        self.aprilTagsStartBtn.clicked.connect(self.controller.start)

        self.aprilTagsStopBtn = QtGui.QPushButton("Stop")
        self.aprilTagsStopBtn.clicked.connect(self.controller.stop)


        self.layout =  QtGui.QVBoxLayout()
        self.layout.addWidget(self.stateLabel)
        self.layout.addWidget(self.stateTextbox)
        self.layout.addWidget(self.keyInstructLabel)
        self.layout.addWidget(self.cmdVelLabel)
        self.layout.addWidget(self.cmdVelTextbox)
        hlayout = QtGui.QHBoxLayout()
        hlayout.addWidget(self.aprilTagsInfoLabel)
        hlayout.addWidget(self.aprilTagsInfoBtn)
        hlayout.addWidget(self.aprilTagsStartBtn)
        hlayout.addWidget(self.aprilTagsStopBtn)
        self.layout.addLayout(hlayout)
        self.layout.addWidget(self.aprilTagsTextbox)
        self.setLayout(self.layout)

        self.setWindowTitle("Sphero Intrude")
        self.show()

    def keyPressEvent(self, e): 
        twist = None       
        if e.key() == QtCore.Qt.Key_U:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_I:
            twist = Twist()  
            twist.linear.x = 0; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0     
        elif e.key() == QtCore.Qt.Key_O:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_J:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_K:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_L:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = 0; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_M:
            twist = Twist()
            twist.linear.x = -80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Comma:
            twist = Twist()
            twist.linear.x = 0; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        elif e.key() == QtCore.Qt.Key_Period:
            twist = Twist()
            twist.linear.x = 80; twist.linear.y = -80; twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0 
        if twist != None:
            self.cmdVelPub.publish(twist)

    def cmdVelCallback(self, msg):
        cmd_vel_text = "x=" + str(msg.linear.x) + " y=" + str(msg.linear.y)
        self.emit(QtCore.SIGNAL("sendCmdVelText(PyQt_PyObject)"), cmd_vel_text) 

    def updateCmdVelTextbox(self, value):
        self.cmdVelTextbox.moveCursor(QtGui.QTextCursor.End)
        self.cmdVelTextbox.ensureCursorVisible()
        self.cmdVelTextbox.append(str(value))
        self.cmdVelTextbox.update()

    def trackposCallback(self, msg):
        rospy.wait_for_service("apriltags_intrude")
        try:
            intrude_query = rospy.ServiceProxy("apriltags_intrude", apriltags_intrude)
            resp = intrude_query(int(msg.x), int(msg.y))
            pos_id_text = "["+str(int(msg.x))+"," +str(int(msg.y))+"]" + "(" + str(resp.id) + ")"
            self.emit(QtCore.SIGNAL("sendPosIDText(PyQt_PyObject)"), pos_id_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def updateStateTextbot(self, value):
        self.stateTextbox.moveCursor(QtGui.QTextCursor.End)
        self.stateTextbox.ensureCursorVisible()
        self.stateTextbox.append(str(value))
        self.stateTextbox.update()

    def queryAprilTagsInfo(self):
        #print "clicked"
        rospy.wait_for_service("apriltags_info")
        try:
            info_query = rospy.ServiceProxy("apriltags_info", apriltags_info)
            resp = info_query()
               
            #print str(resp)

            info_text = "" 
            for i in range(len(resp.polygons)):
                poly = resp.polygons[i]
                t_id = resp.ids[i]

                #print(str(poly))
                #print(str(t_id))
                info_text += "["+str(t_id)+"] "
                for p in poly.points:
                    info_text += "(" + str(int(p.x)) + "," + str(int(p.y)) + ")"
                info_text += "\n" 

            self.emit(QtCore.SIGNAL("sendTagInfoText(PyQt_PyObject)"), info_text)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def updateAprilTagsTextbot(self, value):
        self.aprilTagsTextbox.clear()
        self.aprilTagsTextbox.moveCursor(QtGui.QTextCursor.End)
        self.aprilTagsTextbox.ensureCursorVisible()
        self.aprilTagsTextbox.append(str(value))
        self.aprilTagsTextbox.update()        


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    w = SpheroIntrudeForm()
    w.show()
    sys.exit(app.exec_())
  
        
