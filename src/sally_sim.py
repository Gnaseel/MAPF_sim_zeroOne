#!/usr/bin/python3
--testtesttest
import rospy
import json
from enum import Enum
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point,  Pose
import PATHDATA
# from ..data.path import move_data_light, move_data
# from ..data.path import MoveData 
# from ..data/path 
import copy



def publishRobot(robot_list):


    marker_list=MarkerArray()
    marker_list=MarkerArray()
    id_list=MarkerArray()
    for idx, robot in enumerate(robot_list):
        marker = Marker()
        marker.id = idx
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        # marker.lifetime = rospy.Duration(0.3)
        marker.type = Marker.CUBE
        marker.pose.position.x = robot.x
        marker.pose.position.y = robot.y
        marker.scale.x = 1.5
        marker.scale.y = 0.7
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker_list.markers.append(marker)

        id_marker = Marker()
        id_marker.id = idx
        id_marker.header.frame_id = "map"
        id_marker.header.stamp = rospy.Time.now()
        # id_marker.lifetime = rospy.Duration(0.3)
        id_marker.type = Marker.TEXT_VIEW_FACING
        id_marker.pose.position.x = robot.x
        id_marker.pose.position.y = robot.y
        id_marker.pose.position.z = 5.5
        id_marker.scale.x = 1.2
        id_marker.scale.y = 1.2
        id_marker.scale.z = 1.2
        id_marker.color.a = 1.0
        id_marker.color.r = 1.0
        id_marker.color.g = 1.0
        id_marker.color.b = 0.0
        id_marker.text = str(idx)
        id_list.markers.append(id_marker)

    marker_list_pub.publish(marker_list)
    id_list_pub.publish(id_list)

class JParser:
    def __init__(self):
        return
    
    def parse_json_file(self, file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data
    
json_parser = JParser()
json_data = json_parser.parse_json_file("/home/hgnaseel/zeroone_ws/src/sal_sim/src/map_meta.json")

time_scale = 0.01

def getNodePose(target_name):
    global json_data
    # print(f"TARGET NAME {target_name}")
    # print(f"TARGET NAME {target_name}")
    # print(f"TARGET NAME {target_name}")
    for idx, node in enumerate(json_data["nodes"]):
        name = node["name"]
        if name == target_name:
            # print(f"NAME {name}")
            x = json_data["nodes"][idx]["position"]["x"]
            y = json_data["nodes"][idx]["position"]["y"]
            return x, y
    return None, None

class PathParser:
    def __init__(self):
        return
    
    def parse_the_file(data):
        node_list_list = []
        arrive_list_list = []
        start_list_list = []
        for line in data:
            node_list = []
            arrive_list = []
            start_list = []
            line = line[line.find(":")+3:]
            while True:
                c1 = line.find(",")
                if c1 == -1:
                    break
                c2 = line.find(")")
                c3 = line.find(">")
                time_arrive = float(line[0:c1])
                time_start = float(line[c1+1:c2-1])
                node_num = f"{int(line[c2+1:c3-1]):04d}"

                node_list.append(node_num)
                arrive_list.append(time_arrive)
                start_list.append(time_start)
                line = line[c3+2:]
            node_list_list.append(node_list)
            arrive_list_list.append(arrive_list)
            start_list_list.append(start_list)

        return node_list_list, arrive_list_list, start_list_list

class Schedule:
    def __init__(self):
        self.node_list = []
        self.time_arrive_list = []
        self.time_start_list = []

class Status(Enum):
    IDLE = 0
    MOVE = 1
    FINISHIED = 2
class Robot:
    def __init__(self):
        self.id = -1
        self.cur_node = None
        self.next_node = None
        self.time_cur_arrive = 0
        self.time_cur_start = 0
        self.time_next_arrive = 0
        self.node_idx = 0
        # self.time_next_end = 0

        self.status = Status.IDLE
        self.schedule = Schedule()
        self.x = 0
        self.y = 0

        self.dx = 0
        self.dy = 0
        return
    
    def setNodeData(self):
        self.cur_node = self.schedule.node_list[self.node_idx]
        self.next_node = self.schedule.node_list[self.node_idx+1]
        self.time_cur_arrive = self.schedule.time_arrive_list[self.node_idx]
        self.time_cur_start = self.schedule.time_start_list[self.node_idx]
        self.time_next_arrive = self.schedule.time_arrive_list[self.node_idx+1]

        self.x, self.y = getNodePose(self.cur_node)
        
    def shouldIStart(self, time):
        # print(f"Time {self.time_cur_start} cur {time}")
        if self.time_cur_start <= time:
            return True
        return False
    
    def didIArrive(self):

        goal_x, goal_y = getNodePose(self.next_node)

        dist_x = abs(goal_x - self.x)
        dist_y = abs(goal_y - self.y)
        # print(f"            DIST {dist_x+dist_y}")
        if (dist_x + dist_y) < 0.1:
            self.x = goal_x
            self.y = goal_y
            return True
        return False
    
    def culDxDy(self, time):

        goal_x, goal_y = getNodePose(self.next_node)
        dist_time = self.time_next_arrive - self.time_cur_start

        # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        # print(f"Cur Node {self.cur_node}  Next {self.next_node}")
        # print(f"start Time {self.time_cur_start} Arrive Time {self.time_next_arrive}")
        # print(f"start Pose {self.x} {self.y} Arrive Pose {goal_x} {goal_y}")
        dist_x = (goal_x - self.x) / dist_time
        dist_y = (goal_y - self.y) / dist_time
        self.dx = dist_x*time_scale
        self.dy = dist_y*time_scale
        # print(f"    DX Pose {self.dx} {self.dy} Arrive Pose {dist_x} {dist_y}")

    
    def move(self):
        self.x += self.dx
        self.y += self.dy
        return
    def updateRobot(self, time):
        # print(f"Im robot {self.id}  Status {self.status}")
        # print(f"    Idx {self.node_idx} coord  {self.x} {self.y}")
        if self.status == Status.IDLE:
            if self.shouldIStart(time):
                # print(" I Will start!!")
                self.status = Status.MOVE
                self.culDxDy(time)
            else:
                return

        elif self.status == Status.MOVE:
            if self.didIArrive():
                self.node_idx +=1
                # print
                if len(self.schedule.node_list) == self.node_idx+1:
                    self.status = Status.FINISHIED
                    return
                self.status = Status.IDLE
                self.setNodeData()
            else:
                self.move()
            return
        
        elif self.status == Status.FINISHIED:
            return
        
def addNodeMarker(id, x, y):
    marker = Marker()
    marker.id = id
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    # marker.lifetime = rospy.Duration(0.3)
    marker.type = Marker.SPHERE
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 2.0
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    return marker

def getEdgeData():
    global json_data
    edges_markers = MarkerArray()
    for idx, edge in enumerate(json_data["links"]):
        start_node_id = edge["connected"]["from"]
        end_node_id = edge["connected"]["to"]
        start_node_position = None
        end_node_position = None

        for value in json_data["nodes"]:
            if value["id"] == start_node_id:
                start_node_position = value["position"]
            elif value["id"] == end_node_id:
                end_node_position = value["position"]

        marker = Marker()
        marker.id = idx
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        # marker.lifetime = rospy.Duration(0.3)
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.06
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        p1 = Point()
        p1.x = float(start_node_position["x"])
        p1.y = float(start_node_position["y"])
        p2 = Point()
        p2.x = float(end_node_position["x"])
        p2.y = float(end_node_position["y"])
        marker.points = [p1, p2]
        edges_markers.markers.append(marker)

    return edges_markers

    
if __name__ =="__main__":
    rospy.init_node("sally_sim")
    # sim = Sim()
    # sim.json_data = json_parser.parse_json_file("/home/hgnaseel/zeroone_ws/src/sal_sim/src/map_meta.json")
    node, arrive, start = PathParser.parse_the_file(PATHDATA.move_data)
    # node, arrive, start = PathParser.parse_the_file(PATHDATA.move_data_light)
    # node, arrive, start = PathParser.parse_the_file(move_data_light)
    marker_list_pub = rospy.Publisher('/robot_markers', MarkerArray, queue_size=1)
    id_list_pub = rospy.Publisher('/robot_id_markers', MarkerArray, queue_size=1)

    node_list_pub = rospy.Publisher('/node_markers', MarkerArray, queue_size=100)
    link_list_pub = rospy.Publisher('/link_markers', MarkerArray, queue_size=100)

    # --------------- Init robot
    robot_list = []
    for idx in range(len(node)):
        robot = Robot()
        robot.id = idx
        robot.schedule.node_list = node[idx]
        robot.schedule.time_arrive_list = arrive[idx]
        robot.schedule.time_start_list = start[idx]
        robot.setNodeData()
        robot_list.append(robot)
    # print(node)


    time = 0.0
    r = rospy.Rate(50)

    nodes_markers = MarkerArray()
    edges_markers = getEdgeData()
    for idx, node in enumerate(json_data["nodes"]):
        nodes_markers.markers.append(addNodeMarker(idx, float(node["position"]["x"]), float(node["position"]["y"])))


    # print(nodes_markers)
    # print(edges_markers)

    for i in range(50):
        node_list_pub.publish(nodes_markers)
        link_list_pub.publish(edges_markers)
        r.sleep()

    input()
    while not rospy.is_shutdown():
        r.sleep()
        # print("HEllo world")
        publishRobot(robot_list)
        for robot in robot_list:
            robot.updateRobot(time)
        time += time_scale
        # input()
