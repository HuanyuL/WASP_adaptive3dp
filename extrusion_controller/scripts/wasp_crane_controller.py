#!/usr/bin/env python3
import rospy
import requests
import json
from std_msgs.msg import Float64


class CraneController:
    def __init__(self) -> None:

        # Initialize the header for webserver, information can be found in octoprint API documentation
        self.headers = {
            "Host": "iaac",
            "Content-Type": "application/json",
            "Accept": "application/json",
            "X-Api-Key": "F4B968EDF7524DE4BD47E30063D17EE6",
        }

        self.width_sub = rospy.Subscriber("/width", Float64, self.width_callback)
        self.rate = rospy.Rate(10)

        self.width = None
        self.default_motion_speed = rospy.get_param("/default_motion_speed")
        self.slow_motion_speed = rospy.get_param("/slow_motion_speed")
        self.fast_motion_speed = rospy.get_param("/fast_motion_speed")
        self.default_extrusion_speed = rospy.get_param("/default_extrusion_speed")
        self.slow_extrusion_speed = rospy.get_param("/slow_extrusion_speed")
        self.fast_extrusion_speed = rospy.get_param("/fast_extrusion_speed")
        self.default_width = rospy.get_param("/default_width")
        self.width_threshold = rospy.get_param("/width_threshold")
        self.printing_state = None
        self.previous_state = None

    def check_parameters(self) -> None:
        if any(
            param is None
            for param in [
                self.default_motion_speed,
                self.slow_motion_speed,
                self.fast_motion_speed,
                self.default_extrusion_speed,
                self.slow_extrusion_speed,
                self.fast_extrusion_speed,
                self.default_width,
                self.width_threshold,
            ]
        ):
            rospy.logerr("Failed to get parameters")
            rospy.signal_shutdown("Failed to get parameters")

    def run(self) -> None:
        if self.width is not None:
            self.retreive_printer_status()
            self.printing_state = "maintain"
            while not rospy.is_shutdown():
                self.printing_state = self.check_state(self.width)
                if self.previous_state != self.printing_state:
                    print("State changed")
                    if self.printing_state == "increase":
                        self.increase_width()

                    elif self.printing_state == "decrease":
                        self.decrease_width()

                else:
                    rospy.loginfo("Maintaining changes")

                self.previous_state = self.printing_state
                self.rate.sleep()

    def width_callback(self, msg) -> None:
        self.width = msg.data
        if self.width is not None:
            rospy.loginfo("Received width: {}".format(self.width))

    def retreive_printer_status(self) -> dict:
        # Get the status of the printer
        response = requests.get(
            "http://192.168.0.159/api/printer",
            headers=self.headers,
            params={"exclude": "temperature,sd"},
        )

        if response.status_code != 200:
            rospy.logerr("Failed to get printer status")
            return None

        return response.json()

    def adjust_width(self, motion_speed: int, extrusion_speed: int) -> None:
        try:
            response_fdr = requests.post(
                "http://192.168.0.159/api/printer/printhead",
                headers=self.headers,
                data=json.dumps({"command": "flowrate", "factor": motion_speed}),
            )

            response_flr = requests.post(
                "http://192.168.0.159/api/printer/tool",
                headers=self.headers,
                data=json.dumps({"command": "flowrate", "factor": extrusion_speed}),
            )

            if response_fdr.status_code == 204 and response_flr.status_code == 204:
                rospy.loginfo(
                    f"Width adjustment successful: motion_speed={motion_speed}, extrusion_speed={extrusion_speed}"
                )
            else:
                rospy.logerr("Failed to adjust width")

        except requests.RequestException as e:
            rospy.logerr(f"Failed to adjust width: {e}")

    def increase_width(self) -> None:
        self.adjust_width(self.slow_motion_speed, self.fast_extrusion_speed)

    def decrease_width(self) -> None:
        self.adjust_width(self.fast_motion_speed, self.slow_extrusion_speed)

    def check_state(self, data) -> str:
        if data > self.default_width + self.width_threshold:
            return "decrease"
        if data < self.default_width - self.width_threshold:
            return "increase"
        return "maintain"


def main():
    rospy.init_node("wasp_crane_controller", anonymous=True)
    crane_controller = CraneController()
    crane_controller.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
