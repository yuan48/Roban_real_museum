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


class Turntable(bodyact.Action, nav.Navigation):
    """转盘类 继承bodyhub_action的Action类和navigation的Navigation类

        实现待转盘杆远离机器人后，机器人按照固定路径通关

            runRace = Turntable(body_client)
            runRace.debug() # 调试模式
            runRace.start() # 比赛模式
    """
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        nav.Navigation.__init__(self, body_client, False)
        self.bodyhub = body_client
        self.marker_debug_print_on = 0b01
        self.slam_debug_print_on = 0b10

        self.image = image_manager.ImageManager()
        hsv = image_detect.color_range(image_detect.rgb_to_hsv([30, 65, 90]), [0.1, 0.4, 0.7])
        self.rope = image_detect.ColorDetect(hsv)
        self.center = [0 for i in range(5)]
        
        self.slam_path_point = [[2.0, 0, 0], [2.23, 0.34, 0], [2.75, 0.32, 0], [2.74, -0.34, 0], [2.65, -0.75, 0]]

    def set_head_rot(self, head_rot):
        """设置头部舵机相对于标准状态下的角度

        @param head_rot: 设置头部舵机的角度，可以传入一个列表包含两个元素，第一个元素代表头部舵机上下旋转的角度，第二个值代表头部舵机左右旋转的角度
        """
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[1], head_rot[0]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_rot = [0, head_rot[0], head_rot[1]]

    def elude_pendulum_bob(self):
        """判断转盘杆是否转到可以安全通过的位置

            杆满足条件即退出循环
        """
        idx, v = 0, 0
        while not rospy.is_shutdown():
            result = self.rope.detect(self.image.chin_image, area_max=True)
            if len(result) > 0:
                self.center[idx] = result[0]['center'][0]
                idx = idx + 1
                if idx >= len(self.center):
                    idx = 0
                    for i in range(1, len(self.center)):
                        v = v + (self.center[i] - self.center[i-1])
                    v = float(v) / (len(self.center)-1)
                if v > 3 and result[0]['center'][0] > 540:      # 判断正转
                    break
                elif v < -3 and result[0]['center'][0] < 100:   # 判断反转
                    break
            time.sleep(0.1)

    def debug(self):
        import cv2 as cv
        self.bodyhub_ready()
        # self.set_head_rot([15, 0])
        # self.set_arm_mode(1)
        # self.set_debug_print(self.slam_debug_print_on)
        while not rospy.is_shutdown():
            result = self.rope.detect(self.image.chin_image, area_max=True)
            if len(result) > 0:
                print result
            cv.imshow("image detect", self.image.chin_image)
            cv.waitKey(1)
            time.sleep(0.1)
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)

    def start(self):
        """等待杆转到合适位置再走一段固定路径

        """
        print 'wait rope...'
        self.elude_pendulum_bob()
        print 'wait over'

        self.bodyhub_ready()
        self.set_head_rot([15, 0])
        self.set_arm_mode(0)

        self.bodyhub_walk()
        self.path_tracking(self.slam_path_point, mode=0)
        self.bodyhub.wait_walking_done()
        
        self.bodyhub_ready()
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)


if __name__ == '__main__':
    from motion import bodyhub_client as bodycli

    rospy.init_node('botech_turntable_node', anonymous=True)
    time.sleep(0.2)
    obj = Turntable(bodycli.BodyhubClient(2))
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        obj.debug()
    else:
        obj.start()
