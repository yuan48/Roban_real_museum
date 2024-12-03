#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import math
import time


class ImageManager:
    """订阅摄像头消息，转化成opencv格式的图像

            manager = ImageManager()
            manager.chin_image
            manager.eye_image
            manager.eye_depth

    """
    def __init__(self):
        self.__bridge = CvBridge()
        self.__chin_image = None
        self.__eye_image = None
        self.__eye_depth = None

        rospy.Subscriber("/chin_camera/image", Image, self.__chin_callback)
        rospy.Subscriber("/camera/color/image_raw", Image, self.__eye_callback)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.__eye_depth_callback)

    @property
    def chin_image(self):
        """返回下巴摄像头图像
        
        """
        while not rospy.is_shutdown() and self.__chin_image is None:
            continue
        return self.__chin_image

    @property
    def eye_image(self):
        """返回头部摄像头图像
        
        """
        while not rospy.is_shutdown() and self.__eye_image is None:
            continue
        return self.__eye_image

    @property
    def eye_depth(self):
        """返回深度摄像头图像
        
        """
        while not rospy.is_shutdown() and self.__eye_depth is None:
            continue
        return self.__eye_depth

    def __chin_callback(self, image):
        """处理下巴摄像头消息，转换成opencv格式的图像信息

            @param image: 下巴摄像头订阅的消息
        """
        try:
            image = self.__bridge.imgmsg_to_cv2(image, "bgr8")
            self.__chin_image = image
            # src_point = np.float32(((190, 0), (160, 300), (522, 0), (575, 300)))
            # dst_point = np.float32(((160, 0), (160, 300), (575, 0), (575, 300)))
            # matrix = cv.getPerspectiveTransform(src_point, dst_point)
            # image = cv.warpPerspective(image, matrix, image.shape[1::-1])
            # self.__chin_image = image[:300]
        except CvBridgeError as err:
            rospy.logerr(err)

    def __eye_callback(self, image):
        """处理头部摄像头消息，转换成opencv格式的图像信息

            @param image: 头部摄像头订阅的消息
        """
        try:
            image = self.__bridge.imgmsg_to_cv2(image, "bgr8")
            self.__eye_image = image
        except CvBridgeError as err:
            rospy.logerr(err)

    def __eye_depth_callback(self, image):
        """处理深度摄像头消息，转换成opencv格式的图像信息

            @param image: 深度摄像头订阅的消息
        """
        try:
            image = self.__bridge.imgmsg_to_cv2(image, "16UC1")
            self.__eye_depth = image
        except CvBridgeError as err:
            rospy.logerr(err)


if __name__ == "__main__":
    rospy.init_node("image_manager")
    manager = ImageManager()
    while not rospy.is_shutdown():
        cv.imshow("chin image", manager.chin_image)
        cv.imshow("eye image", manager.eye_image)
        cv.imshow("eye depth", manager.eye_depth)
        cv.waitKey(1)
