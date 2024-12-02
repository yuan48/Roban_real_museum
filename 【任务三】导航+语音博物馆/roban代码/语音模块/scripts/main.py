#!/usr/bin/env python3
# coding=utf-8
import rospy
import rospkg
from ros_AIUI_node.srv import *
import pyaiui
from pyAIUIConstant import AIUIConstant
import json5
import os
import sys
from Player import Player
import random
import json
from threading import Thread
import time
from EventListener import EventListener
from AIui_node import AIUINode, read_json5, load_local_skills
ROS_PACKAGE_NAME = "ros_AIUI_node"
PKG_DIR = rospkg.RosPack().get_path(ROS_PACKAGE_NAME)

local_custom_skills_file_path = os.path.join(PKG_DIR, "config/VoiceControlConfig.json")
# local_custom_skills_dict = read_json5(local_custom_skills_file_path)
local_custom_skills_dict = load_local_skills(local_custom_skills_file_path)
if __name__ == "__main__":
    rospy.init_node("ros_aiui_node", anonymous=False)
    debug = True if len(sys.argv) > 1 and sys.argv[1] == "debug" else False
    eventlistener = EventListener(skills_dict = local_custom_skills_dict, debug = debug)
    aiuinode = AIUINode(eventlistener, debug)
    rospy.on_shutdown(aiuinode.rosShutdownHook)
    aiuinode.start()
