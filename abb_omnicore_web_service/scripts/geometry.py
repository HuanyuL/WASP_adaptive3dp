#!/usr/bin/env python3

import rospy
import json
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header
import numpy as np


def load_points_from_json(file_path):
    with open(file_path, "r") as file:
        points = json.load(file)
    return points


def generate_gradient_color(index, total_points):
    ratio = float(index) / float(total_points)
    r = 1.0
    g = ratio * 0.9
    b = 0
    return ColorRGBA(r, g, b, 1.0)


def create_marker(points):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "points"
    marker.id = 0
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.005  # Size of the points
    # marker.scale.y = 0.01

    total_points = len(points)
    for i, pt in enumerate(points):
        point = Point(pt["x"] / 1000, pt["y"] / 1000, pt["z"] / 1000)
        color = generate_gradient_color(i, total_points)
        marker.points.append(point)
        marker.colors.append(color)

    return marker


def clear_markers(self):
    marker_array = MarkerArray()
    marker = Marker()
    marker.header = Header()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.action = Marker.DELETEALL
    marker_array.markers.append(marker)

    for i in range(10):
        self.marker_pub.publish(marker_array)
        rospy.sleep(0.1)


def main():
    rospy.init_node("points_visualizer")

    pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    rospy.sleep(1)

    points = load_points_from_json("/home/huanyu/Downloads/test1.json")
    marker = create_marker(points)

    while not rospy.is_shutdown():
        pub.publish(marker)
        rospy.sleep(1)


if __name__ == "__main__":
    main()
