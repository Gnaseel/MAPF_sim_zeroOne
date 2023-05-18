#!/usr/bin/python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point,  Pose
import PATHDATA

from robot import Robot
from myParsers import JParser, PathParser
import copy



def publishRobot(robot_list):
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

json_parser = JParser()
json_data = json_parser.parse_json_file("/home/hgnaseel/zeroone_ws/src/sal_sim/src/map_meta.json")

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
    node, arrive, start = PathParser.parse_the_file(PATHDATA.move_data)
    # node, arrive, start = PathParser.parse_the_file(PATHDATA.move_data_light)
    marker_list_pub = rospy.Publisher('/robot_markers', MarkerArray, queue_size=1)
    id_list_pub = rospy.Publisher('/robot_id_markers', MarkerArray, queue_size=1)

    node_list_pub = rospy.Publisher('/node_markers', MarkerArray, queue_size=100)
    link_list_pub = rospy.Publisher('/link_markers', MarkerArray, queue_size=100)


    time_scale = 0.01

    # --------------- Init robot
    robot_list = []
    for idx in range(len(node)):
        robot = Robot()
        robot.id = idx
        robot.schedule.node_list = node[idx]
        robot.schedule.time_arrive_list = arrive[idx]
        robot.schedule.time_start_list = start[idx]
        robot.time_scale = time_scale
        robot.json_data = json_data
        robot.setNodeData()
        robot_list.append(robot)


    time = 0.0
    r = rospy.Rate(50)

    nodes_markers = MarkerArray()
    edges_markers = getEdgeData()
    for idx, node in enumerate(json_data["nodes"]):
        nodes_markers.markers.append(addNodeMarker(idx, float(node["position"]["x"]), float(node["position"]["y"])))


    for i in range(50):
        node_list_pub.publish(nodes_markers)
        link_list_pub.publish(edges_markers)
        r.sleep()
    publishRobot(robot_list)

    input()
    while not rospy.is_shutdown():
        r.sleep()
        publishRobot(robot_list)
        for robot in robot_list:
            robot.updateRobot(time)
        time += time_scale
        # input()
