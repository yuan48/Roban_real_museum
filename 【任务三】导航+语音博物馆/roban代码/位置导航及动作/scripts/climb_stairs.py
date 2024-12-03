#!/usr/bin/python
# -*- coding: utf-8 -*-
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import threading
import time
import math
from lejufunc import client_walk
from motion.motionControl import SetBodyhubStatus, SetBodyhubTo_walking
from ik_module_climbstairs import ClimbStairs

WIDTH = 320
HEIGHT = 240
GAIT_RANGE = 0.05
ROTATION_RANGE = 10.0
Control_ID = 6
SLOP_ANGLE = 6


color_range = {
    'red': [(160, 51, 51), (179, 255, 255)],
    'red2': [(0, 51, 51), (10, 255, 255)],
    'green': [(35, 43, 46), (77, 255, 255)],
}


class NearStairs(threading.Thread):
    def __init__(self):
        super(NearStairs, self).__init__()
        self.bridge = CvBridge()
        self.walker = client_walk.WalkTransfer()
        self.climber = ClimbStairs()
        rospy.Subscriber("/chin_camera/image", Image, self.img_callback)

    def get_img(self):
        img = rospy.wait_for_message("/chin_camera/image", Image, 1)
        try:
            cv_img = self.bridge.imgmsg_to_cv2(img, 'bgr8')
            cv_image = cv2.resize(cv_img, (0, 0), fx=0.5, fy=0.5)
        except CvBridgeError as err:
            print(err)
        else:
            return cv_image[:110, :]

    def slow_walk(self, direction, gait=None, stepnum=None, angle=None):
        """
        :param direction: "forward" ,"backward", "left", "right"  or "rotation"
        :param gait: max:0.07
        :param stepnum: int num
        :param angle: degree
        :return:
        """
        array = [0.0, 0.0, 0.0]
        if direction == "forward":
            array[0] = gait
        elif direction == "backward":
            array[0] = -1 * gait
        elif direction == "left":
            array[1] = 0.8 * gait
        elif direction == "right":
            array[1] = -0.8 * gait
        elif direction == "rotation":
            array[2] = ROTATION_RANGE if angle > 0 else -1 * ROTATION_RANGE
            stepnum = int(abs(angle) / ROTATION_RANGE)
        else:
            rospy.logerr("error walk direction")
        self.walker.walkingPublish(array, stepnum)
        self.walker.waitForWalkingDone()

    def get_line(self, position, img, color):
        """
        :param position: 'bottom' or 'top'
        :param img: color_image
        :param color: hsv color range such as: [(167, 59, 152), (178, 210, 255)]
        :return: bottom_line points or top_line points
        """
        img = cv2.GaussianBlur(img, (5, 5), 0)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, np.array(color_range[color][0]), np.array(color_range[color][1]))
        if color == 'red':
            mask2 = cv2.inRange(hsv_img, np.array(color_range['red2'][0]), np.array(color_range['red2'][1]))
            mask = mask | mask2
        mask = cv2.medianBlur(mask, 7)
        image, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            max_area = max(contours, key=cv2.contourArea)
            epsilon = 0.05 * cv2.arcLength(max_area, True)
            approx = cv2.approxPolyDP(max_area, epsilon, True)
            cv2.drawContours(img, [approx], 0, (255, 0, 0), 2)
            approx_list = list(approx)
            approx_after = []
            for i in range(len(approx_list)):
                approx_after.append(approx_list[i][0])
            approx_after = sorted(approx_after, key=lambda x: x[0])
            if position == 'top':
                approx_after = sorted(approx_after, key=lambda x: x[1])
            elif position == 'bottom':
                approx_after = sorted(approx_after, key=lambda x: x[1], reverse=True)
            if approx_after[0][0] > approx_after[1][0]:
                approx_after[0], approx_after[1] = approx_after[1], approx_after[0]
            return approx_after[0], approx_after[1]
        else:
            return None

    def get_slope(self, line_list):
        x1, y1 = line_list[0]
        x2, y2 = line_list[1]
        if x1 == x2:
            k = -100
        else:
            k = float(y2 - y1) / (x2 - x1)
        return k

    def line_distance(self, line_list):
        x1, y1 = line_list[0]
        x2, y2 = line_list[1]
        dist = float(y1 + y2) / 2
        return dist

    def goto_stairs(self, color, distance, gait, line):
        """
        :param color: such as 'red', 'green', 'blue' ...
        :param distance: distance to line
        :param gait: step size
        :param line: 'bottom' or 'top'
        :return:
        """
        SetBodyhubTo_walking(Control_ID)
        while not rospy.is_shutdown():
            while not rospy.is_shutdown():
                img = self.get_img()
                borderline = self.get_line(line, img, color)
                if not borderline:
                    self.slow_walk("forward", 0.05, 2)
                    continue
                k = self.get_slope(borderline)
                if k > math.tan(-(SLOP_ANGLE - 5) * math.pi / 180):
                    self.slow_walk("rotation", angle=-10)
                elif k < math.tan(-(SLOP_ANGLE + 5) * math.pi / 180):
                    self.slow_walk("rotation", angle=10)
                else:
                    break
            img = self.get_img()
            borderline = self.get_line(line, img, color)
            dist = self.line_distance(borderline)
            if dist < distance:
                self.slow_walk("forward", gait, 1)
            else:
                SetBodyhubStatus(Control_ID, 'stop')
                break

    def run(self):
        self.climb_stairs()

    def climb_stairs(self):
        self.goto_stairs('red', distance=100, gait=0.03, line='bottom')
        self.goto_stairs('green', distance=13, gait=0.02, line='bottom')
        self.climber.toInitPoses()
        self.climber.getposes()
        self.climber.PosPara_wF.Torso_z = -self.climber.leftLegZ
        self.climber.PosPara_wF.Lfoot_y = self.climber.leftLegY
        self.climber.PosPara_wF.Rfoot_y = self.climber.rightLegY
        self.climber.climb_stairs('up')
        self.goto_stairs('green', distance=13, gait=0.02, line='top')
        self.climber.climb_stairs('up')
        self.goto_stairs('red', distance=50, gait=0.02, line='top')
        self.climber.climb_stairs('down')
        self.climber.reset()

    def img_callback(self, img):
        cv_img = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        cv_image = cv2.resize(cv_img, (0, 0), fx=0.5, fy=0.5)
        cv_image = cv_image[:110, :]
        cv2.imshow("cv_image", cv_image)
        bottom_line1 = self.get_line('bottom', cv_image, 'red')
        bottom_line2 = self.get_line('bottom', cv_image, 'green')
        top_line1 = self.get_line('top', cv_image, 'green')
        top_line2 = self.get_line('top', cv_image, 'red')
        if bottom_line1:
            x1, y1 = bottom_line1[0]
            x2, y2 = bottom_line1[1]
            cv2.line(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        if bottom_line2:
            x3, y3 = bottom_line2[0]
            x4, y4 = bottom_line2[1]
            cv2.line(cv_image, (x3, y3), (x4, y4), (255, 0, 0), 2)
        if top_line1:
            x5, y5 = top_line1[0]
            x6, y6 = top_line1[1]
            cv2.line(cv_image, (x5, y5), (x6, y6), (0, 0, 255), 2)
        if top_line2:
            x7, y7 = top_line2[0]
            x8, y8 = top_line2[1]
            cv2.line(cv_image, (x7, y7), (x8, y8), (255, 255, 0), 2)
        cv2.imshow("line_image", cv_image)
        if cv2.waitKey(1) == ord("q"):
            pass


if __name__ == '__main__':
    rospy.init_node("climb_stairs")
    near_stairs = NearStairs()
    near_stairs.start()
    rospy.spin()
