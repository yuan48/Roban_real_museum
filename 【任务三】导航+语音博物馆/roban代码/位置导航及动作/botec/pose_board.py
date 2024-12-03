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

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *


class PoseBoard(bodyact.Action, nav.Navigation):
    """姿态门类 继承bodyhub_action的Action类和navigation的Navigation类

        实现机器人以合适姿势通过姿态门

            runRace = PoseBoard(body_client)
            runRace.debug() # 调试模式
            runRace.start() # 比赛模式
    """
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        nav.Navigation.__init__(self, body_client, False)
        self.bodyhub = body_client
        self.marker_debug_print_on = 0b01
        self.slam_debug_print_on = 0b10

        self.use_slam = False
        self.slam_path1_point = [[-0.5, -2.5, 0], [-0.3, -2.5, -180]]
        self.slam_path2_point = [[-0.8, -2.6, 0], [0.9, -2.6, -180]]

    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[1], head_rot[0]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_rot = [0, head_rot[0], head_rot[1]]

    def board_act(self):
        """设计姿态门对应的动作

        """
        try:
            board_frames = {"1": [[[1000, 0], [0, 0], [0, 0]]], "2": [[[1000, -1.5], [0, 0], [0, 0]]], "3": [[[1000, 0], [0, 0], [0, 0]]], "4": [[[1000, 0], [0, 0], [0, 0]]], "5": [[[1000, 0], [0, 0], [0, 0]]], "6": [[[1000, -1.5], [0, 0], [0, 0]]], "7": [[[1000, 0], [0, 0], [0, 0]]], "8": [[[1000, 1.5], [0, 0], [0, 0]]], "9": [[[1000, 0], [0, 0], [0, 0]]], "10": [[[1000, 0], [0, 0], [0, 0]]], "11": [[[1000, 0], [0, 0], [0, 0]]], "12": [[[1000, 1.5], [0, 0], [0, 0]]], "13": [[[1000, 0], [0, 0], [0, 0]]], "14": [[[1000, -35], [0, 0], [0, 0]]], "15": [[[1000, 0], [0, 0], [0, 0]]], "16": [[[1000, 0], [0, 0], [0, 0]]], "17": [[[1000, -5], [0, 0], [0, 0]]], "18": [[[1000, 85], [0, 0], [0, 0]]], "19": [[[1000, 0], [0, 0], [0, 0]]], "20": [[[1000, 0], [0, 0], [0, 0]]], "21": [[[1000, 0], [0, 0], [0, 0]]], "22": [[[1000, 0], [0, 0], [0, 0]]]}
            board_musics = []
            client_action.custom_action(board_musics, board_frames)
        except Exception as err:
            serror(err)
        finally:
            pass

    def debug(self):
        self.bodyhub_ready()
        self.set_debug_print(self.marker_debug_print_on + self.slam_debug_print_on)
        while not rospy.is_shutdown():
            time.sleep(0.01)

    def start(self):
        """前往姿态门，识别地上的artag信息，调整至可通过姿态门的角度，再做正确的通关动作
        
        """
        if self.use_slam:
            self.bodyhub_ready()
            self.set_head_rot([15, 0])
            self.set_arm_mode(0)
            self.bodyhub_walk()
            self.path_tracking(self.slam_path1_point)
            self.bodyhub.wait_walking_done()
            self.bodyhub_ready()
            self.set_head_rot([0, 0])
            self.set_arm_mode(1)
        else:
            self.bodyhub_walk()
            self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 6)
            self.bodyhub.wait_walking_done()

        self.bodyhub_walk()
        self.goto_pose(0, [0.14, -0.21, 0.0])

        self.bodyhub_ready()
        self.board_act()
        time.sleep(8)
        self.bodyhub.reset()

        if self.use_slam:
            self.bodyhub_ready()
            self.set_head_rot([15, 0])
            self.set_arm_mode(0)
            self.bodyhub_walk()
            self.path_tracking(self.slam_path2_point)
            self.bodyhub.wait_walking_done()
            self.bodyhub_ready()
            self.set_head_rot([0, 0])
            self.set_arm_mode(1)
        else:
            self.bodyhub_walk()
            self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 6)
            self.bodyhub.wait_walking_done()

        self.bodyhub_walk()
        self.goto_rot(1, 0.0)
        self.goto_pose(1, [0.13, -0.21, 0.0])


if __name__ == '__main__':
    from motion import bodyhub_client as bodycli

    rospy.init_node('botech_pose_board_node', anonymous=True)
    time.sleep(0.2)
    obj = PoseBoard(bodycli.BodyhubClient(2))
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        obj.debug()
    else:
        obj.start()
