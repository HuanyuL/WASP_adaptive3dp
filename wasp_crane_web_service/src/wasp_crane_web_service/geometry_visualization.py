#!/usr/bin/env python3

import rospy
import json
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header, Int8, Bool


class GeometryVisualization:
    def __init__(self):
        self.polyline_marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.error_marker_pub = rospy.Publisher("/error_marker", Marker, queue_size=10)
        # self.error_sub = rospy.Subscriber("/iaac_monitoring/pixel_space/deviation", Bool, self.error_callback)
        self.traj_path = rospy.get_param(
            "~json_path", default="/dev_ws/src/wasp_crane_webservice/wasp_crane_web_service/wasp_onsite.json"
        )
        # self.error_positions = []
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown)

    def load_data_from_json(self):
        with open(self.traj_path, "r") as file:
            data = json.load(file)
        return data

    def run(self):
        data = self.load_data_from_json()
        while not rospy.is_shutdown():
            marker_id = 0
            for polyline in data["polylines"]:
                points = polyline["points"]
                layer_id = polyline["layer_id"][0]
                self.polyline_marker_pub.publish(self.create_polyline_marker(points, marker_id, layer_id))
                marker_id += 1

            self.rate.sleep()

    def generate_gradient_color(self, normalized_z):
        r = 0
        g = normalized_z * 0.9
        b = 1.0
        return ColorRGBA(1, 0, b, 1.0)

    def create_polyline_marker(self, points, marker_id, layer_id):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tool_path_layer_" + str(layer_id)
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01  # Size of the points

        z_values = [pt["z"] for pt in points]

        min_z = min(z_values)
        max_z = max(z_values)

        # print(min_z, max_z)

        for pt in points:
            point = Point(pt["x"] / 1000, pt["y"] / 1000, pt["z"] / 1000)
            marker.points.append(point)
            normalized_z = self.normalize_value(pt["z"], min_z, max_z)
            color = self.generate_gradient_color(normalized_z)
            marker.colors.append(color)

        return marker

    def shutdown(self):
        self.clear_markers()
        rospy.loginfo("[IAAC GEOMETRY VISUALIZATION]: Shutting down...")

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

    @staticmethod
    def normalize_value(value, min_value, max_value):
        if max_value - min_value == 0:
            return 0.0
        return (value - min_value) / (max_value - min_value)


if __name__ == "__main__":
    rospy.init_node("geometry_visualization")
    try:
        geometry_visualization = GeometryVisualization()
        geometry_visualization.run()
    except rospy.ROSInterruptException:
        pass
