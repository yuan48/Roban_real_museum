#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import cv2
import Queue
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import threading
from std_msgs.msg import *
from bodyhub.srv import *  # for SrvState.srv
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
from lejulib import *
import rospkg


QUEUE_IMG = Queue.Queue(maxsize=2)# 用于传递图像数据的队列
bridge = CvBridge()
faceadd = rospkg.RosPack().get_path("ros_actions_node") + "/scripts/tracking/haarcascade_frontalface_alt2.xml"
face_detector = cv2.CascadeClassifier(faceadd)# 加载识别人脸的级联分类器

class FaceConfig:# 储存人脸识别参数的类
    def __init__(self):
        self.size = 0.5 # 识别的缩放比例，缩小分辨率可以加快识别速度
        self.face = 0, 0, 0, 0 # 人脸坐标
        self.face_roi = 0, 0, 0, 0 # 感兴趣区域的坐标，用于在上一次查找的人脸附近进行跟踪
        self.face_template = None # 人脸模板，用于人脸短暂丢失时的查找
        self.found_face = False
        self.template_matching_running = False
        self.template_matching_start_time = 0
        self.template_matching_current_time = 0
        self.center_x = 160
        self.center_y = 120

class Face_detecter(threading.Thread):
    def __init__(self, debug = False):
        super(Face_detecter, self).__init__()
        self.__cv_bridge = CvBridge()
        self.debug = True
        self.Face = FaceConfig() 
        rospy.Subscriber('/camera/color/image_raw', Image, self.__image_callback)# 订阅摄像头图片获取数据
        self.face_tem = None

    def __image_callback(self, msg):
        """摄像头图像的回调函数"""
        try:
            cv2_img = self.__cv_bridge.imgmsg_to_cv2(msg, "bgr8")# 通过CvBridge将ros消息的图片数据转化为opencv中的bgr8格式
        except CvBridgeError as err:
            print(err)
        else:
            # 缩小分辨率，加快检测速度
            cv2_img = cv2.resize(cv2_img, (0, 0), fx=self.Face.size, fy=self.Face.size)
            
            # 根据获取到的人脸信息绘制结果
            face = self.Face.face
            cv2.rectangle(cv2_img, (face[0], face[1]), (face[0]+face[2],face[1]+face[3]), (0, 0, 255), 2)
            if self.debug:
                cv2.imshow("result",cv2_img)# 如果在debug模式则显示人脸识别结果
                cv2.imshow("ROI",self.face_tem) if self.face_tem is not None else None# 如果在debug模式则显示人脸识别结果 
                cv2.waitKey(1)
            if QUEUE_IMG.full():
                QUEUE_IMG.get()
            QUEUE_IMG.put(cv2_img, block=True)# 将图像推入队列中

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.core.is_shutdown_requested() and not self.Face.found_face:
            # 外循环用于检测人脸并锁定最大的人脸
            time.sleep(0.01)
            if not QUEUE_IMG.empty():
                frame = QUEUE_IMG.get()
            else:
                continue
            self.detectFaceAllSizes(frame)# 检测人脸并找到并锁定最大的人脸

            while not rospy.core.is_shutdown_requested() and self.Face.found_face:
                # 如果找到人脸并锁定之后，进入内循环
                rate.sleep()
                if not self.Face.face_template.any():
                    continue
                if not QUEUE_IMG.empty():
                    frame = QUEUE_IMG.get()
                else:
                    continue
                self.detectFaceAroundRoi(frame)# 在感兴趣区域即上一次检测到人脸的区域附近进行检测
                # 如果没有检测到人脸，则使用模板匹配查找
                if self.Face.template_matching_running:
                    self.detectFacesTemplateMatching(frame)
    
    def get_face_pos(self):
        """获取人脸中心位置"""
        face_x = self.Face.face[0] + self.Face.face[2] / 2
        face_y = self.Face.face[1] + self.Face.face[3] / 2
        if face_x == 0 and face_y == 0:
            face_x = self.Face.center_x
            face_y = self.Face.center_y
        return [face_x/self.Face.size,face_y/self.Face.size]

    def detectFaceAllSizes(self, frame):
        """Detect using cascades over whole image

        :param frame:
        :return:
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
        face_locations = face_detector.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=3, minSize=(frame.shape[1] / 12, frame.shape[0] / 12),
            maxSize=(2 * frame.shape[1] / 3, 2 * frame.shape[1] / 3))
        if len(face_locations) <= 0:
            self.Face.face = 0, 0, 0, 0
            return
        self.Face.found_face = True
        self.Face.face = face_filter(face_locations)# 筛选出最大的人脸

        #储存人脸模板图片
        self.Face.face_template = frame[self.Face.face[1]:(self.Face.face[1] + self.Face.face[3]),
                            self.Face.face[0]:(self.Face.face[0] + self.Face.face[2])].copy()
        self.Face.face_roi = doubleRectSize(self.Face.face, (0, 0, frame.shape[1], frame.shape[0]))
        
    def detectFaceAroundRoi(self, frame):
        """Detect using cascades only in ROI
        在感兴趣区域附近检测人脸

        :param frame:
        :return:
        """
        face_tem = frame[self.Face.face_roi[1]:self.Face.face_roi[1] + self.Face.face_roi[3],
                self.Face.face_roi[0]:self.Face.face_roi[0] + self.Face.face_roi[2]]
        gray = cv2.cvtColor(face_tem, cv2.COLOR_BGR2GRAY)
        self.face_tem = face_tem
        face_locations = face_detector.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=3, minSize=(frame.shape[1] / 12, frame.shape[0] / 12),
            maxSize=(2 * frame.shape[1] / 3, 2 * frame.shape[1] / 3))
        if len(face_locations) <= 0:# 如果没有检测到人脸则准备使用模板匹配进行查找
            self.Face.template_matching_running = True
            if self.Face.template_matching_start_time == 0:
                self.Face.template_matching_start_time = cv2.getTickCount()# 记录开始模板匹配的时间
            return
        self.Face.template_matching_running = False
        self.Face.template_matching_current_time = 0
        self.Face.template_matching_start_time = 0
        self.Face.face = face_filter(face_locations)# 在ROI中查找最大人脸区域
        # 人脸在ROI中的坐标加上ROI的全局坐标才是人脸的全局坐标
        self.Face.face[0] += self.Face.face_roi[0]
        self.Face.face[1] += self.Face.face_roi[1]

        # 储存最新的人脸模板
        self.Face.face_template = frame[self.Face.face[1]:self.Face.face[1] + self.Face.face[3],
                            self.Face.face[0]:self.Face.face[0] + self.Face.face[2]].copy()
        
        # 将新的人脸区域放大两倍作为感兴趣区域(ROI)
        self.Face.face_roi = doubleRectSize(self.Face.face, (0, 0, frame.shape[1], frame.shape[0]))


    def detectFacesTemplateMatching(self, frame):
        """Detect using template matching

        :param frame:
        :return:
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.Face.template_matching_current_time = cv2.getTickCount()
        duration = (self.Face.template_matching_current_time - self.Face.template_matching_start_time) / cv2.getTickFrequency()
        if duration > 1:# 如果1s内没有检测到人脸(使用模板匹配1s后)，则退出内循环进入外循环查找其他人脸
            self.Face.found_face = False
            self.Face.template_matching_running = False
            self.Face.template_matching_start_time = 0
            self.Face.template_matching_current_time = 0

        # 使用模板图片在感兴趣区域进行模板匹配
        target = gray[self.Face.face_roi[1]:self.Face.face_roi[1] + self.Face.face_roi[3],
                self.Face.face_roi[0]:self.Face.face_roi[0] + self.Face.face_roi[2]]
        self.Face.face_template = cv2.cvtColor(self.Face.face_template, cv2.COLOR_BGR2GRAY)
        res = cv2.matchTemplate(target, self.Face.face_template, cv2.TM_CCOEFF)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
        max_x = max_loc[0] + self.Face.face_roi[0]
        max_y = max_loc[1] + self.Face.face_roi[1]

        # 使用模板匹配的结果作为人脸检测的结果
        self.Face.face = max_x, max_y, self.Face.face[2], self.Face.face[3]
        self.Face.face_template = frame[self.Face.face[1]:self.Face.face[1] + self.Face.face[3],
                            self.Face.face[0]:self.Face.face[0] + self.Face.face[2]].copy()
        # 将人脸区域围绕中心放大两倍作为感兴趣区域
        self.Face.face_roi = doubleRectSize(self.Face.face, (0, 0, frame.shape[1], frame.shape[0]))

    
def doubleRectSize(input_rect, keep_inside):
    """将input_rect围绕其中心放大2倍，同时确保不超出keep_inside的范围"""
    xi, yi, wi, hi = input_rect
    xk, yk, wk, hk = keep_inside
    wo = wi * 2
    ho = hi * 2
    xo = xi - wi // 2
    yo = yi - hi // 2
    if wo > wk:
        wo = wk
    if ho > hk:
        ho = hk
    if xo < xk:
        xo = xk
    if yo < yk:
        yo = yk
    if xo + wo > wk:
        xo = wk - wo
    if yo + ho > hk:
        yo = hk - ho
    return xo, yo, wo, ho

def face_size(face):
    """计算人脸面积"""
    x, y, w, h = face
    return w * h

def face_filter(face_list):
    """查找返回最大的人脸所在坐标"""
    face_size_list = map(face_size, face_list)
    target_index = face_size_list.index(max(face_size_list))
    return face_list[target_index]

