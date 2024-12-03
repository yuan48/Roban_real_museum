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


class PendulumBob(bodyact.Action, nav.Navigation):
    """摆锤类 继承bodyhub_action的Action类和navigation的Navigation类

        实现待摆锤杆远离机器人后，机器人按照固定路径通关

            runRace = PendulumBob(body_client)
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
        hsv = image_detect.color_range(image_detect.rgb_to_hsv([5, 70, 90]), [0.1, 0.4, 0.7])
        self.pendulum = image_detect.ColorDetect(hsv)
        self.center = [0 for i in range(5)]

        self.slam_path1_point = [[0.32, 0.14, 0], [0.9, 0.14, 0], [1.0, 0.14, 0]]
        self.slam_path2_point = [[2.0, 0.1, 0], [2.1, 0.1, 0]]

    def set_head_rot(self, head_rot):
        keyframes = [
            ([0, -1, 16, -34, -17, -1, 0, 1, -16, 34, 17, 1, 0, -70, -15, 0, 70, 15, 0, 0, head_rot[1], head_rot[0]], 500, 0)
        ]
        self.movement.linearMove(keyframes)
        self.head_rot = [0, head_rot[0], head_rot[1]]

    def elude_pendulum_bob(self):
        """判断摆锤杆是否转到可以安全通过的位置

            杆满足条件即退出循环
        """
        idx, v = 0, 0
        while not rospy.is_shutdown():
            result = self.pendulum.detect(self.image.eye_image, area_max=True)
            if len(result) > 0:
                self.center[idx] = result[0]['center'][0]
                idx = idx + 1
                if idx >= len(self.center):
                    idx = 0
                    for i in range(1, len(self.center)):
                        v = v + (self.center[i] - self.center[i-1])
                    v = float(v) / (len(self.center)-1)
                if v > 3 and result[0]['center'][0] > 540:  # 判断摆锤从右往左移动
                    break
                elif v < -3 and result[0]['center'][0] < 100:   # 判断摆锤从左往右移动
                    break
            time.sleep(0.1)

    def debug(self):
        import cv2 as cv
        self.bodyhub_ready()
        # self.set_debug_print(self.slam_debug_print_on)
        while not rospy.is_shutdown():
            result = self.pendulum.detect(self.image.eye_image, area_max=True)
            if len(result) > 0:
                print result
            cv.imshow("image detect", self.image.eye_image)
            cv.waitKey(1)
            time.sleep(0.1)

    def start(self):
        """等待杆移动到合适位置再走一段固定路径

        """
        self.bodyhub_ready()
        self.set_head_rot([15, 0])
        self.set_arm_mode(0)

        self.bodyhub_walk()
        self.path_tracking(self.slam_path1_point)
        self.bodyhub.wait_walking_done()
        
        self.bodyhub_ready()
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)

        time.sleep(1)
        print 'wait pendulum bob...'
        self.elude_pendulum_bob()
        print 'wait over'
        
        self.bodyhub_ready()
        self.set_head_rot([15, 0])
        self.set_arm_mode(0)

        self.bodyhub_walk()
        self.bodyhub.walking_n_steps([0.1, 0.0, 0.0], 4)
        self.bodyhub.wait_walking_done()
        self.path_tracking(self.slam_path2_point)
        self.bodyhub.wait_walking_done()
        
        self.bodyhub_ready()
        self.set_head_rot([0, 0])
        self.set_arm_mode(1)


if __name__ == '__main__':
    from motion import bodyhub_client as bodycli

    rospy.init_node('botech_pendulum_node', anonymous=True)
    time.sleep(0.2)
    obj = PendulumBob(bodycli.BodyhubClient(2))
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        obj.debug()
    else:
        obj.start()
