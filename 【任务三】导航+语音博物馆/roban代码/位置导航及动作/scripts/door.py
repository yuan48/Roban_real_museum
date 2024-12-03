#!/usr/bin/python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray 
import cv2 as cv
import numpy as np 
from motion.motionControl import SendGaitCommand, WaitForWalkingDone, WalkTheDistance, SetBodyhubTo_walking
import math
import time

BLUE_HSV = ((78, 43, 46), (124, 255, 255))
BLACK_HSV = ((0, 0, 0), (180, 255, 46))

# 障碍物可以出现在的最低点，即最大 Y 值
LAND_MAX_Y = 180

# 可跨越宽度像素值
GO_WIDTH = 350
CENTER_DIFF = 40

# 机器人中点到边线的最大距离 cm
ROBOT_CENTER_DIFF = 15

# 地板的宽度 cm
FLOOR_WIDTH = 80

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480


def get_chin_image():
    chin_topic = "/chin_camera/image"
    image = rospy.wait_for_message(chin_topic, Image)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image, "bgr8")
    src_point = np.float32(((218, 0), (200, 240), (484, 0), (494, 240)))
    dst_point = np.float32(((200, 0), (200, 240), (494, 0), (494, 240)))
    matrix = cv.getPerspectiveTransform(src_point, dst_point)
    image = cv.warpPerspective(image, matrix, image.shape[1::-1])
    # cv.imshow("src", img)
    return image[:240]


def get_eye_image():
    topic = "/camera/color/image_raw"
    image = rospy.wait_for_message(topic, Image)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image, "bgr8")
    return image

def get_eye_depth_image():
    topic = "/camera/depth/image_rect_raw"
    image = rospy.wait_for_message(topic, Image)
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image, "16UC1")
    return image


class Door():
    def __init__(self, get_chin_image, get_eye_image, get_eye_depth_image):
        self.get_chin_image = get_chin_image
        self.get_eye_image = get_eye_image
        self.get_eye_depth_image = get_eye_depth_image
        self.start()

    def image_to_blue(self):
        image = self.get_eye_image()
        binary = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        binary = cv.inRange(binary, BLUE_HSV[0], BLUE_HSV[1])
        return image, binary

    def image_to_black(self):
        image = self.get_chin_image()
        binary = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        binary = cv.inRange(binary, BLACK_HSV[0], BLACK_HSV[1])
        return image, binary

    def get_distance(self):
        image, binary = self.image_to_blue()
        depth_image = self.get_eye_depth_image()
        print(depth_image.shape)
        cv.imshow("image", image)
        cv.imshow("binary", binary)
        cv.waitKey(1)
        image = depth_image[binary == 255]
        image = image[image != 0]
        image = image[image < 800]
        return np.sum(image) / image.size

    def find_contours(self):
        _, binary = self.image_to_black()
        _, contours, _ = cv.findContours(binary, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        contours = [ cv.approxPolyDP(contour, 10, True) for contour in contours if len(cv.approxPolyDP(contour, 10, True)) == 4 ]
        return contours

    def get_wall_scope_x(self):
        contours = self.find_contours()
        if len(contours) == 0:
            return
        contour = contours[0]
        points = [ point[0] for point in contour ]
        p1 = points[0] - points[1]
        width = math.hypot(p1[0], p1[1])
        p2 = points[0] - points[3]
        height = math.hypot(p2[0], p2[1])
        if width > height:
            scope = math.atan(p1[1] * 1.0 / p1[0] ) / math.pi * 180
        else:
            scope = math.atan(p2[1] * 1.0 / p2[0] ) / math.pi * 180

        max_x = max(points, key=lambda n: n[0])[0]
        min_x = min(points, key=lambda n: n[0])[0]
        return scope, (max_x + min_x) / 2




    def start(self):
        SetBodyhubTo_walking(2)
        center_x = 400
        is_center = False
        # WalkTheDistance(0.50, 0, 0)
        import time
        while not rospy.is_shutdown():
            # 获取边线 如果没有，则往边上挪挪
            time.sleep(2)
            if is_center == False:
                wall_par = self.get_wall_scope_x()
                if wall_par == None:
                    WalkTheDistance(0, 0.10, 0)
                    continue
                scope, mid_x = wall_par
                center_x = 320 - mid_x
                angle = scope
                symbol = 1 if angle > 0 else -1
                angle = (90 - angle * symbol) * symbol
                if angle < -5 or angle > 5:
                    print("", angle)
                    WalkTheDistance(0, 0, angle)
                    continue
            
                distance = self.get_distance()
                print(distance)
                if distance == 0:
                    WalkTheDistance(0.10, 0, 0)
                elif distance > 250:
                    WalkTheDistance((distance - 250) / 1000.0, 0, 0)
                else:
                    WalkTheDistance(0, -(400 - center_x) / 1000.0, 0)
                    WalkTheDistance(0.40, 0, 0)
                    break



            print self.get_distance()

            # print("exit")
            # return


        
if __name__ == "__main__":
    rospy.init_node("landmine_node")

    a = np.array([1,2,3,4,5])
    # print a[a==2].size
    Door(get_chin_image, get_eye_image, get_eye_depth_image)
    rospy.spin()
    