#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import threading
import numpy as np
import rospy
import rospkg
import bodyhub_action as bodyact
import navigation as nav
import image_manager
import image_detect

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *

class OpenDoor(bodyact.Action, nav.Navigation):
    """开门类 继承bodyhub_action的Action类和navigation的Navigation类

        slam路径引导到按钮前，调用动作击打开关实现开门

            runRace = OpenDoor(body_client)
            runRace.debug() # 调试模式
            runRace.start() # 比赛模式
    """
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        nav.Navigation.__init__(self, body_client, False)
        self.bodyhub = body_client
        self.marker_debug_print_on = 0b01
        self.slam_debug_print_on = 0b10

        self.slam_path_point = [[2.35, -0.95, -85]]

    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[1], head_rot[0]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_rot = [0, head_rot[0], head_rot[1]]

    def press_door(self):
        """击打开门开关

        """
        try:
            press_door_1_frames = {"1":[[[700,0],[0,0],[150,0]],[[1200,0],[-150,0],[90,0]],[[1500,0],[-90,0],[150,0]],[[2000,0],[-150,0],[90,0]],[[2300,0],[-90,0],[210,0]],[[3000,0],[-210,0],[0,0]]],"2":[[[700,-1.5],[0,0],[150,0]],[[1200,-1.5],[-150,0],[90,0]],[[1500,-1.5],[-90,0],[150,0]],[[2000,-1.5],[-150,0],[90,0]],[[2300,-1.5],[-90,0],[210,0]],[[3000,-1.5],[-210,0],[0,0]]],"3":[[[700,16.5],[0,0],[150,0]],[[1200,16.5],[-150,0],[90,0]],[[1500,16.5],[-90,0],[150,0]],[[2000,16.5],[-150,0],[90,0]],[[2300,16.5],[-90,0],[210,0]],[[3000,16.5],[-210,0],[0,0]]],"4":[[[700,-34.1],[0,0],[150,0]],[[1200,-34.1],[-150,0],[90,0]],[[1500,-34.1],[-90,0],[150,0]],[[2000,-34.1],[-150,0],[90,0]],[[2300,-34.1],[-90,0],[210,0]],[[3000,-34.1],[-210,0],[0,0]]],"5":[[[700,-17.6],[0,0],[150,0]],[[1200,-17.6],[-150,0],[90,0]],[[1500,-17.6],[-90,0],[150,0]],[[2000,-17.6],[-150,0],[90,0]],[[2300,-17.6],[-90,0],[210,0]],[[3000,-17.6],[-210,0],[0,0]]],"6":[[[700,-1.5],[0,0],[150,0]],[[1200,-1.5],[-150,0],[90,0]],[[1500,-1.5],[-90,0],[150,0]],[[2000,-1.5],[-150,0],[90,0]],[[2300,-1.5],[-90,0],[210,0]],[[3000,-1.5],[-210,0],[0,0]]],"7":[[[700,0],[0,0],[150,0]],[[1200,0],[-150,0],[90,0]],[[1500,0],[-90,0],[150,0]],[[2000,0],[-150,0],[90,0]],[[2300,0],[-90,0],[210,0]],[[3000,0],[-210,0],[0,0]]],"8":[[[700,1.5],[0,0],[150,0]],[[1200,1.5],[-150,0],[90,0]],[[1500,1.5],[-90,0],[150,0]],[[2000,1.5],[-150,0],[90,0]],[[2300,1.5],[-90,0],[210,0]],[[3000,1.5],[-210,0],[0,0]]],"9":[[[700,-16.5],[0,0],[150,0]],[[1200,-16.5],[-150,0],[90,0]],[[1500,-16.5],[-90,0],[150,0]],[[2000,-16.5],[-150,0],[90,0]],[[2300,-16.5],[-90,0],[210,0]],[[3000,-16.5],[-210,0],[0,0]]],"10":[[[700,34.1],[0,0],[150,0]],[[1200,34.1],[-150,0],[90,0]],[[1500,34.1],[-90,0],[150,0]],[[2000,34.1],[-150,0],[90,0]],[[2300,34.1],[-90,0],[210,0]],[[3000,34.1],[-210,0],[0,0]]],"11":[[[700,17.6],[0,0],[150,0]],[[1200,17.6],[-150,0],[90,0]],[[1500,17.6],[-90,0],[150,0]],[[2000,17.6],[-150,0],[90,0]],[[2300,17.6],[-90,0],[210,0]],[[3000,17.6],[-210,0],[0,0]]],"12":[[[700,1.5],[0,0],[150,0]],[[1200,1.5],[-150,0],[90,0]],[[1500,1.5],[-90,0],[150,0]],[[2000,1.5],[-150,0],[90,0]],[[2300,1.5],[-90,0],[210,0]],[[3000,1.5],[-210,0],[0,0]]],"13":[[[700,-45],[0,0],[150,0]],[[1200,-95],[-150,0],[90,0]],[[1500,-95],[-90,0],[150,0]],[[2000,-95],[-150,0],[90,0]],[[2300,-45],[-90,-8.549999999999999],[210,19.95]],[[3000,0],[-210,0],[0,0]]],"14":[[[700,25],[0,0],[150,0]],[[1200,25],[-150,0],[90,0]],[[1500,-15],[-90,0],[150,0]],[[2000,20],[-150,0],[90,0]],[[2300,20],[-90,0],[210,0]],[[3000,-70],[-210,0],[0,0]]],"15":[[[700,-15],[0,0],[150,0]],[[1200,-90],[-150,0],[90,0]],[[1500,-65],[-90,0],[150,0]],[[2000,-90],[-150,0],[90,0]],[[2300,-90],[-90,0],[210,0]],[[3000,-15],[-210,0],[0,0]]],"16":[[[700,0],[0,0],[150,0]],[[1200,0],[-150,0],[90,0]],[[1500,0],[-90,0],[150,0]],[[2000,0],[-150,0],[90,0]],[[2300,0],[-90,0],[210,0]],[[3000,0],[-210,0],[0,0]]],"17":[[[700,70],[0,0],[150,0]],[[1200,70],[-150,0],[90,0]],[[1500,70],[-90,0],[150,0]],[[2000,70],[-150,0],[90,0]],[[2300,70],[-90,0],[210,0]],[[3000,70],[-210,0],[0,0]]],"18":[[[700,15],[0,0],[150,0]],[[1200,15],[-150,0],[90,0]],[[1500,15],[-90,0],[150,0]],[[2000,15],[-150,0],[90,0]],[[2300,15],[-90,0],[210,0]],[[3000,15],[-210,0],[0,0]]],"19":[[[700,0],[0,0],[150,0]],[[1200,0],[-150,0],[90,0]],[[1500,0],[-90,0],[150,0]],[[2000,0],[-150,0],[90,0]],[[2300,0],[-90,0],[210,0]],[[3000,0],[-210,0],[0,0]]],"20":[[[700,0],[0,0],[150,0]],[[1200,0],[-150,0],[90,0]],[[1500,0],[-90,0],[150,0]],[[2000,0],[-150,0],[90,0]],[[2300,0],[-90,0],[210,0]],[[3000,0],[-210,0],[0,0]]],"21":[[[700,0],[0,0],[150,0]],[[1200,0],[-150,0],[90,0]],[[1500,0],[-90,0],[150,0]],[[2000,0],[-150,0],[90,0]],[[2300,0],[-90,0],[210,0]],[[3000,0],[-210,0],[0,0]]],"22":[[[700,0],[0,0],[150,0]],[[1200,0],[-150,0],[90,0]],[[1500,0],[-90,0],[150,0]],[[2000,0],[-150,0],[90,0]],[[2300,0],[-90,0],[210,0]],[[3000,0],[-210,0],[0,0]]]}
            press_door_1_musics = []
            client_action.custom_action(press_door_1_musics,press_door_1_frames)
        except Exception as err:
            serror(err)
        finally:
            pass


    def debug(self):
        self.bodyhub_ready()
        self.set_head_rot([15, 0])
        self.set_arm_mode(1)
        self.set_debug_print(self.slam_debug_print_on)
        while not rospy.is_shutdown():
            time.sleep(0.01)
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)

    def start(self):
        self.bodyhub_ready()
        self.set_head_rot([15, 0])
        self.set_arm_mode(0)

        self.bodyhub_walk()
        self.path_tracking(self.slam_path_point)
        self.bodyhub.wait_walking_done()
        
        self.bodyhub_ready()
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)

        self.bodyhub_ready()
        self.press_door()


if __name__ == '__main__':
    from motion import bodyhub_client as bodycli

    rospy.init_node('botech_open_door_node', anonymous=True)
    time.sleep(0.2)
    obj = OpenDoor(bodycli.BodyhubClient(2))
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        obj.debug()
    else:
        obj.start()
