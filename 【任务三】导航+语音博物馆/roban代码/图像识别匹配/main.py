#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, rospkg
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from cv_bridge import *
import numpy as np
import math
import json
import os, sys
from feature_matching import FeatureMatch
import subprocess
import threading
from ros_AIUI_node.srv import textToSpeak
import cv2 as cv

sys.path.append(rospkg.RosPack().get_path("leju_lib_pkg") + "/src/lejufunc")
import motion.bodyhub_client as bodycli
import algorithm.pidAlgorithm as pidAlg
sys.path.append(os.path.join(rospkg.RosPack().get_path('ros_actions_node'), 'scripts'))
from lejulib import *

BOW_HEAD_ACTION_PATH = os.path.join(rospkg.RosPack().get_path("ros_museum_docent"), "scripts/bow_head.py")

NODE_NAME = "museum_docent"
CONTROL_ID = 6
ARTAG_POS_TOPIC = "/head/visualization_marker_head"
HEAD_IMAGE_TOPIC = '/camera/color/image_raw'
ACTION_FINISH_TOPIC = "/Finish"
AIUI_PLAY_END = "/aiui/play_end"
ARTAG_CONFIG_FILE_PATH = os.path.join(rospkg.RosPack().get_path("ros_museum_docent"), "config/config.json")
WAIT_ACTION_DONE = 0.5
MODLE_IMG = os.path.join(rospkg.RosPack().get_path("ros_museum_docent"), "scripts/model_image/model.png")
COUNT_MODEL_IMAGE = 2
POINT_X_INDEX = 0
SINGLE_MODEL_WIDTH = 640

class Action(object):

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
        self.__cv_bridge = CvBridge()
        self.__image_origin = np.zeros((640, 480, 3), np.uint8)
        rospy.Subscriber(ARTAG_POS_TOPIC, Marker, self.marker_callback, queue_size=1)
        rospy.Subscriber(HEAD_IMAGE_TOPIC, Image, self.image_callback, queue_size=1)
        self.is_finished = True
        self.is_play_end = True
        rospy.Subscriber(ACTION_FINISH_TOPIC, String, self.action_finish_callback, queue_size=1)
        rospy.Subscriber(AIUI_PLAY_END, Empty, self.aiui_play_end_callback, queue_size=1)
        # self.current_pos = {"z": 0.0, "x": 0.0, "pitch": 0.0}
        # self.pid_z = pidAlg.PositionPID(p=self.pid_gain["z_p"])
        # self.pid_x = pidAlg.PositionPID(p=self.pid_gain["x_p"])
        # self.pid_pitch = pidAlg.PositionPID(p=self.pid_gain["pitch_p"])
        self.start()
        # cv.namedwindow("img")
        # rospy.spin()

    def aiui_play_end_callback(self, msg):
        self.is_play_end = True

    def action_finish_callback(self, data):
        self.is_finished = True

    def image_callback(self, data):
        try:
            self.__image_origin = self.__cv_bridge.imgmsg_to_cv2(data, "bgr8")
            # print (self.__image_origin)
            # cv.imshow("img",self.__image_origin)
            # cv.waitKey(0)
        except CvBridgeError as err:
            rospy.logerr(err)

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
        with open(file_path, "r") as file:
            data = file.read()
        return json.loads(data)

    def load_artag_config(self):
        data = self.load_json_file(ARTAG_CONFIG_FILE_PATH)
        self.tag_num = data["tag_num"]
        self.target_pos = data["target_pos"]
        self.threshold = data["threshold"]
        self.pid_gain = data["pid_gain"]
        self.elements_and_results_of_matching = data["elements_and_results_of_matching"]

    def action_and_move_to(self, cultural_relic):
        action_and_move_to_list = self.elements_and_results_of_matching[cultural_relic]["action_and_move_to_list"]
        for event in action_and_move_to_list:
            if isinstance(event, list):
                self.walk_to(event)
            else:
                self.execute_action(event)

    def walk_to(self, data):
        self.bodyhub_walk()
        self.bodyhub.walking_the_distance(data[0], data[1], data[2])
        self.bodyhub.wait_walking_done()

    def execute_action(self, action_file_path):

        self.p = subprocess.Popen("python {}".format(action_file_path.encode("utf-8")), stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
        while self.is_finished == False:
            rospy.sleep(0.1)

    def calibrate_axis_or_angle(self, difference, threshold):
        difference = 0 if abs(difference) < threshold else difference

    def goto_position(self):
        self.bodyhub_walk()
        while not rospy.is_shutdown():
            self.is_current_pos_refresh = False
            rospy.sleep(0.5)
            if self.is_current_pos_refresh:
                z_axis_difference = self.current_pos["z"] - self.target_pos["z"]
                x_axis_difference = self.current_pos["x"] - self.target_pos["x"]
                pitch_angle_difference = self.current_pos["pitch"] - self.target_pos["pitch"]
                if (abs(z_axis_difference) < self.threshold["z"]) and (abs(x_axis_difference) < self.threshold["x"]) and (abs(pitch_angle_difference) < self.threshold["pitch"]):
                    rospy.loginfo("到达目标位置，当前位置：{}".format(self.current_pos))
                    break
                self.calibrate_axis_or_angle(z_axis_difference, self.threshold["z"])
                self.calibrate_axis_or_angle(x_axis_difference, self.threshold["x"])
                self.calibrate_axis_or_angle(pitch_angle_difference, self.threshold["pitch"])
                z_pid_distance = self.pid_z.run(z_axis_difference)
                x_pid_distance = self.pid_x.run(x_axis_difference)
                pitch_pid_angle = self.pid_pitch.run(pitch_angle_difference)
                self.bodyhub.walking_the_distance(z_pid_distance, x_pid_distance, pitch_pid_angle)
                self.bodyhub.wait_walking_done()
                rospy.loginfo("z_axis_difference: {}, x_axis_difference:{}, pitch_angle_difference:{}".format(z_axis_difference, x_axis_difference, pitch_angle_difference))
            else:
                rospy.logwarn("未能识别到 artag 码!")

    def get_match_result(self):
        """
        return model picture index from left to right which is match to origin image.
        """
        number_of_matching_to_single_template = []
        max_number_of_matching = 10
        origin_image = self.__image_origin
        # origin_image = cv.imread("model_image/Sanxingdui_left.jpg")
        fm = FeatureMatch(origin_image)
        is_template=-1
        for element, content_items in self.elements_and_results_of_matching.items():
            # print("element:",element)
            for template_image_path in content_items["model_image_path"]:
                number_of_matching_to_single_template.append(fm.feature_extraction(template_image_path))
            print("match:",number_of_matching_to_single_template)
            if max(number_of_matching_to_single_template) > max_number_of_matching:
                print("max(number_of_matching_to_single_template)",max(number_of_matching_to_single_template))
                max_number_of_matching = max(number_of_matching_to_single_template)
                is_template = element                
            number_of_matching_to_single_template = []
        # print("returning template = ",is_template)
        return is_template

    def text_to_speak(self, cultural_relic):
        rospy.wait_for_service("/aiui/text_to_speak", 2)
        tts_client = rospy.ServiceProxy("/aiui/text_to_speak", textToSpeak)
        if(cultural_relic==-1):
            museum_introduce_text=["停止运行"]
        else:
            museum_introduce_text = self.elements_and_results_of_matching[cultural_relic]["museum_introduce_text"]
        for items in museum_introduce_text:
            while self.is_play_end == False:
                rospy.sleep(0.1)
            if ".mp3" in items or ".wav" in items:
                player_process = subprocess.Popen("ffplay {} -nodisp -loglevel quiet -autoexit".format(items), stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
                while player_process.poll() != 0:
                    rospy.sleep(0.1)
            else:
                self.is_play_end = False
                tts_client(items)
        while self.is_play_end == False:
            rospy.sleep(0.1)

    def start(self):
        # self.goto_position()
        print("-----Detect start begin!---")
        # self.bodyhub_ready()
        # self.execute_action(BOW_HEAD_ACTION_PATH)
        rospy.sleep(WAIT_ACTION_DONE)
        cultural_relic = self.get_match_result()
        print("cultural_relic:",cultural_relic)
        # self.bodyhub.reset()
        if cultural_relic!=-1:
            # print("start threading! Give para to voice")
            exhitpub.publish(cultural_relic)

            # self.bodyhub.reset()

            # tts_thread = threading.Thread(target=self.text_to_speak, args=(cultural_relic, ))
            # action_thread = threading.Thread(target=self.action_and_move_to, args=(cultural_relic, ))
            # tts_thread.start()
            # action_thread.start()
            # tts_thread.join()
            # action_thread.join()
        # else:
        #     tts_thread = threading.Thread(target=self.text_to_speak, args=(cultural_relic, ))
        #     tts_thread.start()
        #     tts_thread.join() 

        # rospy.signal_shutdown('exit')

    def frame_action(self, key_frame):
        try:
            client_action.custom_action([], key_frame)
        except Exception as err:
            serror(err)
        finally:
            pass

if __name__ == "__main__":
    # rospy.init_node('detect',anonymous=False)
    exhitpub=rospy.Publisher("/detected_exhibition",String,queue_size=2)
    # ros_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        MuseumDocent()
        # ros_rate.sleep()
