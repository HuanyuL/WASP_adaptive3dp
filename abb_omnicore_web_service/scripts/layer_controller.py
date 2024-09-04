#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64, Int8, Bool
from abb_omnicore_web_service.srv import (
    SetSpeedRatio,
    GetMastership,
    ReleaseMastership,
)


class LayerController:
    def __init__(self):
        self.layer_sub = rospy.Subscriber("/iaac_monitoring/pixel_space/width_error", Float64, self.layer_callback)
        self.deviation_sub = rospy.Subscriber("/iaac_monitoring/pixel_space/deviation", Bool, self.deviation_callback)
        self.speed_ratio_sub = rospy.Subscriber("/iaac_robot/current_speed_ratio", Int8, self.speed_ratio_callback)

        self.local_speed_change_pub = rospy.Publisher("/iaac_robot/local_speed_change", Int8, queue_size=1)
        self.global_speed_pub = rospy.Publisher("/iaac_robot/global_speed_change", Int8, queue_size=1)

        self.default_speed_ratio = rospy.get_param("/iaac_robot/default_speed_ratio", 70)
        self.step = rospy.get_param(
            "/iaac_robot/layer_controller/threshold", 1
        )  # the changes of speed in rapid code in mm/s
        self.kp = rospy.get_param("/iaac_robot/layer_controller/kp", 12)
        self.max_speed = rospy.get_param("/iaac_robot/max_speed", 20)  # max speed in rapid code in mm/s

        self.width_error = None
        self.speed_ratio = None

        self.speed_upper_limit = 100
        self.speed_lower_limit = 30

        self.deviation = False

        self.previous_speed_ratio = None

        self.rate = rospy.Rate(2)
        rospy.on_shutdown(self.shutdown)

        self._init_service_proxies()

    def _init_service_proxies(self):
        rospy.wait_for_service("/iaac_robot/set_speed_ratio", timeout=5)
        self.set_speed_ratio_srv = rospy.ServiceProxy("/iaac_robot/set_speed_ratio", SetSpeedRatio)

        rospy.wait_for_service("/iaac_robot/get_mastership", timeout=5)
        self.get_mastership_srv = rospy.ServiceProxy("/iaac_robot/get_mastership", GetMastership)

        rospy.wait_for_service("/iaac_robot/release_mastership", timeout=5)
        self.release_mastership_srv = rospy.ServiceProxy("/iaac_robot/release_mastership", ReleaseMastership)

    def layer_callback(self, data):
        self.width_error = data.data

    def speed_ratio_callback(self, data):
        self.speed_ratio = data.data

    def run(self):
        rep = self.get_mastership_srv(True)
        if rep.success:
            rospy.loginfo("[IAAC LAYER CONTROLLER]: Mastership granted")
        while not rospy.is_shutdown():
            if self.width_error is not None and self.speed_ratio is not None:
                rospy.loginfo("[IAAC LAYER CONTROLLER]: Layer controller running")
                new_speed_ratio = self.speed_controller(
                    self.max_speed,
                    self.default_speed_ratio,
                    self.width_error,
                    self.kp,
                    self.step,
                    self.speed_upper_limit,
                    self.speed_lower_limit,
                    self.deviation
                )
                self.set_speed_ratio_srv(int(new_speed_ratio))
                self.global_speed_pub.publish(int(self.default_speed_ratio - new_speed_ratio))
                if self.previous_speed_ratio is not None:
                    self.local_speed_change_pub.publish(new_speed_ratio - self.previous_speed_ratio)
                    self.previous_speed_ratio = new_speed_ratio
                else:
                    self.previous_speed_ratio = self.default_speed_ratio
            else:
                rospy.loginfo("[IAAC LAYER CONTROLLER]: Waiting for data")
            self.rate.sleep()

    def deviation_callback(self,data):
        self.deviation = data.data

    def shutdown(self):
        resp = self.release_mastership_srv(True)
        if resp.success:
            rospy.loginfo("[IAAC LAYER CONTROLLER]: Mastership released")
        rospy.loginfo("[IAAC LAYER CONTROLLER]: Layer controller shutting down")

    @staticmethod
    def speed_controller(max_speed, default_speed_ratio, error, kp, step, upper_limit, lower_limit, deviation):
        default_speed = (default_speed_ratio / 100.0) * max_speed

        if deviation:

            if error > 0:
                new_speed = min(default_speed + kp * abs(error), upper_limit * max_speed / 100)
            elif error < 0:
                new_speed = max(default_speed - kp * abs(error), lower_limit * max_speed / 100)
        else:
            new_speed = default_speed
        rospy.loginfo(f"[IAAC LAYER CONTROLLER]: New speed = {new_speed}")

        # Ensure the change is above the threshold
        if abs(new_speed - default_speed) < step:
            new_speed = default_speed

        new_speed_ratio = (new_speed / max_speed) * 100
        return new_speed_ratio


if __name__ == "__main__":
    rospy.init_node("layer_controller", anonymous=True)
    layer_controller = LayerController()
    layer_controller.run()
