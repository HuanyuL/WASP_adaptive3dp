#!/usr/bin/env python3
import requests
from requests.auth import HTTPBasicAuth
from ws4py.client.threadedclient import WebSocketClient
import xml.etree.ElementTree as ET
import json
import math
import subprocess
import rospy
import urllib3
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from abb_omnicore_web_service.srv import (
    SetDO,
    SetDOResponse,
    SetSpeedRatio,
    SetSpeedRatioResponse,
    GetMastership,
    GetMastershipResponse,
    ReleaseMastership,
    ReleaseMastershipResponse,
)
from abb_omnicore_web_service.msg import OperationStatus

namespace = "{http://www.w3.org/1999/xhtml}"
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


ctrl_state = None
opmode = None
speed_ratio = None


def print_event(evt):
    global ctrl_state, opmode, speed_ratio
    root = ET.fromstring(evt)
    if root.findall(".//{0}li[@class='pnl-ctrlstate-ev']".format(namespace)):
        ctrl_state_data = root.find(".//{0}li[@class='pnl-ctrlstate-ev']/{0}span".format(namespace)).text
        ctrl_state = ctrl_state_data
        print("\tControl State : " + ctrl_state_data)

    if root.findall(".//{0}li[@class='pnl-opmode-ev']".format(namespace)):
        opmode_data = root.find(".//{0}li[@class='pnl-opmode-ev']/{0}span".format(namespace)).text
        opmode = opmode_data
        print("\tOperation Mode : " + opmode_data)

    if root.findall(".//{0}li[@class='pnl-speedratio-ev']".format(namespace)):
        speed_ratio_data = root.find(".//{0}li[@class='pnl-speedratio-ev']/{0}span".format(namespace)).text
        speed_ratio = speed_ratio_data
        print("\tSpeed Ratio : " + speed_ratio_data)

    ios_signalstate_evs = root.findall(".//{0}li[@class='ios-signalstate-ev']".format(namespace))
    if ios_signalstate_evs:
        for ios_signalstate_ev in ios_signalstate_evs:
            title = ios_signalstate_ev.attrib.get("title", "No Title")
            lvalue = ios_signalstate_ev.find(".//{0}span[@class='lvalue']".format(namespace)).text
            print(title.split("/")[-1] + ", Value: " + lvalue)


# This class encapsulates the Web Socket Callbacks functions.
class RobWebSocketClient(WebSocketClient):
    def opened(self):
        print("Web Sockect connection established")

    def closed(self, code, reason=None):
        print("Closed down", code, reason)

    def received_message(self, event_xml):
        if event_xml.is_text:
            print("Events : ")
            print_event(event_xml.data.decode("utf-8"))
        else:
            print("Received Illegal Event " + str(event_xml))


class RWSClient:

    def __init__(self) -> None:

        rospy.init_node("rws_client", anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.host = "192.168.125.1:443"
        self.username = "Admin"
        self.password = "robotics"
        self.basic_auth = HTTPBasicAuth(self.username, self.password)
        self.header = {
            "Accept": "application/hal+json;v=2.0",
            "Content-Type": "application/x-www-form-urlencoded;v=2.0",
        }
        self.subscription_url = "https://{0}/subscription".format(self.host)
        self.session = requests.Session()
        self.session_post = requests.Session()

        self.location = None
        self.cookie = None
        self.ws = None
        self.mastership_status = False
        self.rate = rospy.Rate(100)

        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self.speed_ratio_pub = rospy.Publisher("/iaac_robot/current_speed_ratio", Int8, queue_size=1)
        self.status_pub = rospy.Publisher("/iaac_monitoring/safety/operational_status", OperationStatus, queue_size=1)

        if self._test_connection():
            rospy.loginfo("[IAAC ROBOT CONTROLLER]: Connection to RWS established")
            self._init_services()
            self._init_websocket()

        else:
            rospy.logerr("[IAAC ROBOT CONTROLLER]: Unable to connect to RWS")
            rospy.signal_shutdown("[IAAC ROBOT CONTROLLER]: Unable to connect to RWS")

    def _test_connection(self) -> bool:
        try:
            subprocess.check_output(["ping", "-c", "1", self.host.split(":")[0]], timeout=5)
            return True
        except subprocess.TimeoutExpired:
            rospy.logerr(f"[IAAC ROBOT CONTROLLER]: Ping failed: Unable to reach {self.host}")
            return False
        except subprocess.CalledProcessError:
            rospy.logerr(f"[IAAC ROBOT CONTROLLER]: Ping failed: Unable to reach {self.host}")
            return False

    def _init_websocket(self) -> None:
        payload = {
            "resources": ["1", "2", "3", "4", "5", "6"],
            "1": "/rw/panel/speedratio",
            "1-p": "1",
            "2": "/rw/panel/ctrl-state",
            "2-p": "1",
            "3": "/rw/panel/opmode",
            "3-p": "1",
            "4": "/rw/iosystem/signals/192.168.125.100/ABB_Scalable_IO/ABB_Scalable_IO_0_DO15;state",
            "4-p": "1",
            "5": "/rw/iosystem/signals/192.168.125.100/ABB_Scalable_IO/ABB_Scalable_IO_0_DO16;state",
            "5-p": "1",
            "6": "/rw/motionsystem/mechunits;mechunitmodechangecount",
            "6-p": "1",
        }
        resp = self.session.post(
            self.subscription_url,
            auth=self.basic_auth,
            headers=self.header,
            data=payload,
            verify=False,
        )
        resp_post = self.session_post.get(
            url="https://{0}".format(self.host), auth=self.basic_auth, headers=self.header, verify=False
        )
        if resp_post.status_code != 200:
            rospy.logerr("[IAAC ROBOT CONTROLLER]: Failed to login")
        else:
            rospy.loginfo("[IAAC ROBOT CONTROLLER]: Login done")
            if resp.status_code != 201:
                rospy.logerr("[IAAC ROBOT CONTROLLER]: Failed to subscribe to RWS")
                rospy.signal_shutdown("[IAAC ROBOT CONTROLLER]: Failed to subscribe to RWS")
            else:
                print_event(resp.text)
                self.location = resp.headers["Location"]
                self.cookie = "-http-session-={0}; ABBCX={1}".format(
                    resp.cookies["-http-session-"], resp.cookies["ABBCX"]
                )
                header_login = [("Cookie", self.cookie)]
                self.ws = RobWebSocketClient(
                    self.location, protocols=["rws_subscription"], headers=header_login, heartbeat_freq=0
                )
                self.ws.connect()
                self.ws_thread = threading.Thread(target=self.ws.run_forever)
                self.ws_thread.daemon = True
                self.ws_thread.start()

    def run(self) -> None:
        while not rospy.is_shutdown():
            robot_state = self.session.get(
                "https://{0}/rw/motionsystem/mechunits/ROB_1/jointtarget".format(self.host),
                headers=self.header,
                params={},
                verify=False,
            )
            joint_state = self.parse_joint_states(robot_state.text)
            self.joint_state_pub.publish(joint_state)
            if speed_ratio is not None:
                self.speed_ratio_pub.publish(int(speed_ratio))
            if ctrl_state is not None and opmode is not None:
                self.status_pub.publish(OperationStatus(ctrl_state, opmode))

            self.rate.sleep()

    def _init_services(self) -> None:
        rospy.Service("/iaac_robot/set_speed_ratio", SetSpeedRatio, self.set_speed_ratio_cb)
        rospy.Service("/iaac_robot/set_DO", SetDO, self.set_DO_cb)
        rospy.Service("/iaac_robot/get_mastership", GetMastership, self.get_mastership_cb)
        rospy.Service("/iaac_robot/release_mastership", ReleaseMastership, self.release_mastership_cb)

    def shutdown(self) -> None:
        if self.ws():
            # close the thread
            self.ws_thread.join()
            self.ws.close()
            rospy.loginfo("[IAAC ROBOT CONTROLLER]: Shutting down RWS Client")

    def set_speed_ratio_cb(self, new_speed_ratio) -> SetSpeedRatioResponse:
        if opmode != "AUTO" or self.mastership_status is False:
            rospy.logerr(
                "[IAAC ROBOT CONTROLLER]: Operation Mode is not AUTO or Mastership is not acquired. Cannot set speed ratio."
            )
            return SetSpeedRatioResponse(success=False)
        else:
            payload = {"speed-ratio": new_speed_ratio.speed_ratio}
            resp = self.session_post.post(
                "https://{0}/rw/panel/speedratio".format(self.host),
                headers=self.header,
                data=payload,
                verify=False,
            )
            if resp.status_code == 204:
                rospy.loginfo("[IAAC ROBOT CONTROLLER]: Speed Ratio set to: " + str(new_speed_ratio))
                return SetSpeedRatioResponse(success=True)

    def set_DO_cb(self, req) -> SetDOResponse:
        pin = str(req.pin_id)
        value = str(req.value)
        resp = self.session_post.post(
            "https://{0}/rw/iosystem/signals/192.168.125.100/ABB_Scalable_IO/ABB_Scalable_IO_0_DO{pin}/set-value".format(
                self.host, pin=pin
            ),
            verify=False,
            data={"lvalue": value},
        )
        print(resp.status_code)
        print(resp.text)
        if resp.status_code == 204:
            rospy.loginfo(f"Pin {pin} set to {value}")
            return SetDOResponse(success=True)
        else:
            rospy.logerr(f"Failed to set pin {pin} to {value}")
            return SetDOResponse(success=False)

    def get_mastership_cb(self, req) -> GetMastershipResponse:
        if req:
            resp = self.session_post.post(
                "https://{0}/rw/mastership/edit/request".format(self.host),
                headers=self.header,
                data={},
                verify=False,
            )
            print(resp.status_code)
            if resp.status_code == 204:
                rospy.loginfo("[IAAC ROBOT CONTROLLER]: Mastership requested")
                self.mastership_status = True
                return GetMastershipResponse(success=True)
            else:
                rospy.logerr("[IAAC ROBOT CONTROLLER]: Failed to request mastership")
                return GetMastershipResponse(success=False)

    def release_mastership_cb(self, req) -> ReleaseMastershipResponse:
        if req:
            resp = self.session_post.post(
                "https://{0}/rw/mastership/release".format(self.host), headers=self.header, data={}, verify=False
            )
            if resp.status_code == 204:
                rospy.loginfo("[IAAC ROBOT CONTROLLER]: Mastership released")
                self.mastership_status = False
                return ReleaseMastershipResponse(success=True)
            else:
                rospy.logerr("[IAAC ROBOT CONTROLLER]: Failed to release mastership")
                return ReleaseMastershipResponse(success=False)

    @staticmethod
    def parse_joint_states(joint_state) -> None:
        data = json.loads(joint_state)
        state = data["state"][0]
        joint_state = JointState()
        joint_state.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        joint_state.position = [
            math.radians(float(state["rax_1"])),
            math.radians(float(state["rax_2"])),
            math.radians(float(state["rax_3"])),
            math.radians(float(state["rax_4"])),
            math.radians(float(state["rax_5"])),
            math.radians(float(state["rax_6"])),
        ]
        joint_state.header.stamp = rospy.Time.now()
        return joint_state


if __name__ == "__main__":
    try:
        RC = RWSClient()
        RC.run()
    except rospy.ROSInterruptException:
        WebSocketClient.closed(1000, "ROS Shutdown")
