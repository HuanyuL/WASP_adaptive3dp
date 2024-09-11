#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Int8, Bool
from wasp_crane_web_service.srv import SetFeedrate, SetFlowrate


class LayerController:
    def __init__(self):
        self.layer_sub = rospy.Subscriber(
            "/iaac_monitoring/image_processing/results.width_error", Float64, self.layer_callback
        )
        self.deviation_sub = rospy.Subscriber(
            "/iaac_monitoring/image_processing/results.deviation", Bool, self.deviation_callback
        )
        self.feedrate_pub = rospy.Publisher("/iaac_crane/set_feedrate", Int8, queue_size=10)
        self.default_feedrate = rospy.get_param("/iaac_crane/default_feedrate", 100)
        self.default_feedrate = rospy.get_param("/iaac_crane/default_feedrate", 100)

        self.step = rospy.get_param(
            "/iaac_crane/layer_controller/threshold", 10
        )  # the changes of speed in rapid code in percetage
        self.kp = rospy.get_param("/iaac_crane/layer_controller/kp", 12)

        self.width_error = None

        self.speed_upper_limit = 40
        self.speed_lower_limit = 10

        self.deviation = False

        self.previous_feedrate = None
        self.previous_feedrate = None

        self.rate = rospy.Rate(2)
        rospy.on_shutdown(self.shutdown)

        self._init_service_proxies()

    def _init_service_proxies(self):
        rospy.wait_for_service("/iaac_crane/set_feedrate", timeout=5)
        self.set_feedrate_srv = rospy.ServiceProxy("/iaac_robot/set_feedrate", SetFeedrate)

        rospy.wait_for_service("/iaac_cranet/set_flowrate", timeout=5)
        self.set_flowrate_srv = rospy.ServiceProxy("/iaac_robot/set_feedrate", SetFlowrate)

    def layer_callback(self, data):
        self.width_error = data.data

    def run(self):
        while not rospy.is_shutdown():
            if self.width_error is not None:
                rospy.loginfo("[IAAC LAYER CONTROLLER]: Layer controller running")
                new_feedrate = self.speed_controller(
                    self.default_feedrate,
                    self.width_error,
                    self.kp,
                    self.step,
                    self.speed_upper_limit,
                    self.speed_lower_limit,
                    self.deviation,
                )
                self.set_feedrate_srv(int(new_feedrate))
                if self.previous_feedrate is not None:
                    self.previous_speed_ratio = new_feedrate
                    self.feedrate_pub.publish(int(new_feedrate))
                else:
                    self.previous_speed_ratio = self.default_feedrate
            else:
                rospy.loginfo("[IAAC LAYER CONTROLLER]: Waiting for data")
            self.rate.sleep()

    def deviation_callback(self, data):
        self.deviation = data.data

    def shutdown(self):
        rospy.loginfo("[IAAC LAYER CONTROLLER]: Layer controller shutting down")

    @staticmethod
    def speed_controller(default_feedrate, error, kp, step, upper_limit, lower_limit, deviation):

        if deviation:
            if error > 0:
                new_feedrate = min(default_feedrate + kp * abs(error), upper_limit)
            elif error < 0:
                new_feedrate = max(default_feedrate - kp * abs(error), lower_limit)
        else:
            new_feedrate = default_feedrate

        rospy.loginfo(f"[IAAC LAYER CONTROLLER]: New feedrate = {new_feedrate}")

        # Ensure the change is above the threshold
        if abs(new_feedrate - default_feedrate) < step:
            new_feedrate = default_feedrate

        return new_feedrate


if __name__ == "__main__":
    rospy.init_node("layer_controller", anonymous=True)
    layer_controller = LayerController()
    layer_controller.run()
