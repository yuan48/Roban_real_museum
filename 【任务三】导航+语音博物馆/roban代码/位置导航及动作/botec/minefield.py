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

class Minefield(bodyact.Action, nav.Navigation):
    """地雷类 继承bodyhub_action的Action类和navigation的Navigation类

        规划路径点，机器人通过slam建图引导通过

            runRace = Minefield(body_client)
            runRace.debug() # 调试模式
            runRace.start() # 比赛模式
    """
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        nav.Navigation.__init__(self, body_client, False)
        self.bodyhub = body_client
        self.marker_debug_print_on = 0b01
        self.slam_debug_print_on = 0b10

        self.slam_path_point = [[0.45, -0.54, 0], [0.62, -1.25, 0], [0.46, -1.95, 0], [0.30, -2.45, 0], [0.04, -2.55, -180]]

    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[1], head_rot[0]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_rot = [0, head_rot[0], head_rot[1]]

    def debug(self):
        self.set_arm_mode(1)
        self.bodyhub_ready()
        self.set_head_rot([15, 0])
        self.set_debug_print(self.slam_debug_print_on)
        while not rospy.is_shutdown():
            time.sleep(0.01)
        self.bodyhub_ready()
        self.set_head_rot([0, 0])
        self.set_arm_mode(0)

    def start(self):
        """依次通过标点，避开地雷

        """
        self.bodyhub_ready()
        self.set_head_rot([15, 0])
        self.set_arm_mode(0)

        self.bodyhub_walk()
        self.path_tracking(self.slam_path_point)
        self.bodyhub.wait_walking_done()
        
        self.bodyhub_ready()
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)

    
if __name__ == '__main__':
    from motion import bodyhub_client as bodycli

    rospy.init_node('minefield_node', anonymous=True)
    time.sleep(0.2)
    obj = Minefield(bodycli.BodyhubClient(2))
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        obj.debug()
    else:
        obj.start()