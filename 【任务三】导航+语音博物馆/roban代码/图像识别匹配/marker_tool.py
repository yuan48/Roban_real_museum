#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, rospkg
from visualization_msgs.msg import Marker
import math
import json
import os, sys

sys.path.append(rospkg.RosPack().get_path("leju_lib_pkg") + "/src/lejufunc")
import motion.bodyhub_client as bodycli

NODE_NAME = "museum_docent"
CONTROL_ID = 6
ARTAG_POS_TOPIC = "/head/visualization_marker_head"
ARTAG_CONFIG_FILE_PATH = os.path.join(rospkg.RosPack().get_path("ros_museum_docent"), "config/config.json")

class Action(object):
    '''
    robot action
    '''
    def __init__(self, name, ctl_id):
        rospy.init_node(name, anonymous=False)
        rospy.sleep(0.2)
        rospy.on_shutdown(self.__ros_shutdown_hook)
        self.bodyhub = bodycli.BodyhubClient(ctl_id)

    def __ros_shutdown_hook(self):
        if self.bodyhub.reset() == True:
            rospy.loginfo('bodyhub reset, exit')
        else:
            rospy.loginfo('exit')

    def bodyhub_ready(self):
        if self.bodyhub.ready() == False:
            rospy.logerr('bodyhub to ready failed!')
            rospy.signal_shutdown('error')
            rospy.sleep(1)
            exit(1)

    def bodyhub_walk(self):
        if self.bodyhub.walk() == False:
            rospy.logerr('bodyhub to walk failed!')
            rospy.signal_shutdown('error')
            rospy.sleep(1)
            exit(1)

class MuseumDocent(Action):
    def __init__(self):
        super(MuseumDocent, self).__init__(NODE_NAME, CONTROL_ID)
        self.load_artag_config()
        self.is_current_pos_refresh = False
        rospy.Subscriber(ARTAG_POS_TOPIC, Marker, self.marker_callback, queue_size=1)
        self.current_pos = {"z": 0.0, "x": 0.0, "pitch": 0.0}
        # self.debug()
        rospy.spin()

    def quart_to_pyr(self, w, x, y, z):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return [r, p, y]

    def marker_callback(self, msg):
        pyr = self.quart_to_pyr(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        self.current_pos["z"] = msg.pose.position.z
        self.current_pos["x"] = -msg.pose.position.x
        self.current_pos["pitch"] = -pyr[1] * 180.0 / math.pi
        self.is_current_pos_refresh = True

    def load_json_file(self, file_path):
        with open(file_path, "r", encoding="utf-8") as file:
            data = file.read()
        return json.loads(data)

    def load_artag_config(self):
        data = self.load_json_file(ARTAG_CONFIG_FILE_PATH)
        self.tag_num = data["tag_num"]
        self.target_pos = data["target_pos"]
        self.threshold = data["threshold"]
        self.pid_gain = data["pid_gain"]

    def debug(self):
        self.bodyhub_walk()
        rospy.loginfo("将机器人放到目标位置，按下回车键将自动校准定位")
        while not rospy.is_shutdown():
            if input("input:") == "q":
                break
            self.is_current_pos_refresh = False
            rospy.sleep(0.5)
            if self.is_current_pos_refresh:
                data = self.load_json_file(ARTAG_CONFIG_FILE_PATH)
                original_target_pos_data = data["target_pos"]
                data["target_pos"] = {"z": self.current_pos["z"], "x": self.current_pos["x"], "pitch": self.current_pos["pitch"]}
                with open(ARTAG_CONFIG_FILE_PATH, "w", encoding="utf-8") as wf:
                    json.dump(data, wf, ensure_ascii=False, indent=-1)
                rospy.loginfo("目标位置已更新:\nz:{} ---> {}\nx:{} ---> {}\npitch:{} ---> {}\n程序已退出".format(original_target_pos_data["z"], data["target_pos"]["z"], original_target_pos_data["x"], data["target_pos"]["x"], original_target_pos_data["pitch"], data["target_pos"]["pitch"]))
                break
            else:
                rospy.logwarn("没有识别到 artag 码，请移动机器人或检查摄像头")
        rospy.signal_shutdown('exit')

if __name__ == "__main__":
    MuseumDocent()
