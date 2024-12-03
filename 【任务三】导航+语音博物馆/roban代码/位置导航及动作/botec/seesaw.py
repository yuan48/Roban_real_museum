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


class Seesaw(bodyact.Action, nav.Navigation):
    """跷跷板类 继承bodyhub_action的Action类和navigation的Navigation类

        实现对比赛地图坐标信息的获取以及标定通过关卡的路径

            runRace = Seesaw(body_client)
            runRace.debug() # 调试模式
            runRace.start() # 比赛模式
    """
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        nav.Navigation.__init__(self, body_client, False)
        self.bodyhub = body_client
        self.marker_debug_print_on = 0b01
        self.slam_debug_print_on = 0b10

        self.slam_path_point = [[0.30, 0, 0], [1.7, 0, 0], [1.8, 0, 0]]

    def set_head_rot(self, head_rot):
        """设置头部舵机相对于标准状态下的角度

        @param head_rot: 设置头部舵机的角度，可以传入一个列表包含两个元素，第一个元素代表头部舵机上下旋转的角度，第二个值代表头部舵机左右旋转的角度
        """
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[1], head_rot[0]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_rot = [0, head_rot[0], head_rot[1]]

    def debug(self):
        """debug模式 用于机器人slam和artag定位等调试，输出当前机器人所在slam建图以及artag的坐标信息

        """
        self.bodyhub_ready()
        self.set_head_rot([15, 0])
        self.set_arm_mode(0)
        self.set_debug_print(self.slam_debug_print_on)
        while not rospy.is_shutdown():
            time.sleep(0.01)
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)

    def start(self):
        """start模式 机器人运行比赛程序
        
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

    rospy.init_node('botech_seesaw_node', anonymous=True)
    time.sleep(0.2)
    obj = Seesaw(bodycli.BodyhubClient(2))
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        obj.debug()
    else:
        obj.start()
