#!/usr/bin/env python3
import rospy
import json
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header, Int8, Bool
import tf


class VisualizeGeometry:
    def __init__(self):
        self.polyline_marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.error_marker_pub = rospy.Publisher("/error_marker", Marker, queue_size=10)
        self.tf_listener = tf.TransformListener()
        self.error_sub = rospy.Subscriber("/iaac_monitoring/pixel_space/deviation", Bool, self.error_callback)
        self.traj_path = rospy.get_param("~json_path", default="/dev_ws/src/iaac_monitoring/q3d.json")
        self.error_positions = []
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)

    def run(self):
        rospy.loginfo("Visualize geometry node is running")
        self.clear_markers()
        if self.traj_path is not None:
            points = self.load_points_from_json()
            polyline_marker = self.create_polyline_marker(points)
            self.polyline_marker_pub.publish(polyline_marker)
            print("Published marker")
        while not rospy.is_shutdown():
            if self.error_positions is not None:
                error_marker = self.create_poit_marker(self.error_positions)
                self.error_marker_pub.publish(error_marker)

                self.rate.sleep()

    def load_points_from_json(self):
        with open(self.traj_path, "r") as file:
            points = json.load(file)
        return points

    def generate_gradient_color(self, index, total_points):
        ratio = float(index) / float(total_points)
        r = 0
        g = ratio * 0.9
        b = 1.0
        return ColorRGBA(r, g, b, 1.0)

    def create_polyline_marker(self, points):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.005  # Size of the points

        total_points = len(points)
        for i, pt in enumerate(points):
            point = Point(pt["x"] / 1000, pt["y"] / 1000, pt["z"] / 1000)
            color = self.generate_gradient_color(i, total_points)
            marker.points.append(point)
            marker.colors.append(color)

        return marker

    def create_poit_marker(self, points):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "point"
        marker.id = 1
        marker.type = Marker.POINTS
        marker.points = points
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02  # Sphere diameter in meters
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Red color

        return marker

    def error_callback(self, data):
        if data.data == True:  
            rospy.loginfo("Error detected")
            self.error_pos = self.get_tool_position()
            if self.error_pos is not None:
                self.error_positions.append(self.error_pos)

    def clear_markers(self):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.action = Marker.DELETEALL

        for i in range(10):
            self.polyline_marker_pub.publish(marker)
            self.error_marker_pub.publish(marker)
            rospy.sleep(0.1)

    def get_tool_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform("/base_link", "/tool0", rospy.Time(0))
            return Point(trans[0], trans[1], trans[2])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            return None

    def shutdown(self):
        self.clear_markers()
        rospy.loginfo("Visualize geometry node shutting down")


if __name__ == "__main__":
    rospy.init_node("points_visualizer", anonymous=True)
    visualizer = VisualizeGeometry()
    visualizer.run()
