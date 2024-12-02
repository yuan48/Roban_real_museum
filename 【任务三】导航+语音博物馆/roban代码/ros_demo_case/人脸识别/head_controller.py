#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import rospy
import rospkg
from std_msgs.msg import *
from bodyhub.srv import *  # for SrvState.srv
from bodyhub.msg import JointControlPoint
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
from lejulib import *
import motion.bodyhub_client as bodycli
import algorithm.pidAlgorithm as pidAlg
# Horizontal and vertical limit value
H_limit=75
V_limit=25

def limit_head_angle(actuator_pos):
    if actuator_pos[0] >= H_limit:
        actuator_pos[0] = H_limit
    elif actuator_pos[0] <= -H_limit:
        actuator_pos[0] = -H_limit

    if actuator_pos[1] >= V_limit:
        actuator_pos[1] = V_limit
    elif actuator_pos[1] <= -V_limit:
        actuator_pos[1] = -V_limit
    return actuator_pos
class Head_Joint_Controller(object):
    def __init__(self, debug = False):
        super(Head_Joint_Controller, self).__init__()
        self.debug = debug
        self.pid_x = pidAlg.PositionPID(p=0.005,d=0.002) # 实例化x(水平)方向的pid控制器
        self.pid_y = pidAlg.PositionPID(p=0.005,d=0.002)
        self.err_threshold = [20.0, 20.0] # 执行阈值
        self.controlID= 2 # 用于控制bodyhub的controlID值

        # 实例化一个Publisher用于给bodyhub节点发布头部关节动作
        self.HeadJointPub = rospy.Publisher('MediumSize/BodyHub/HeadPosition', JointControlPoint, queue_size=100)
        bodycli.BodyhubClient(2).ready()# 状态跳转到ready
        self.update_ctlid() # 更新一下controlID的值
        self.set_head_servo([0,0]) # 设置初始头部位置为(0，0)

    def set_head_servo(self, angles):
        if self.debug:
            rospy.logwarn("pub angles:{}".format(angles))
        if not rospy.is_shutdown():
            self.HeadJointPub.publish(positions=angles, mainControlID=self.controlID) # 发布头部关节动作
        
    def update_ctlid(self): # 获取bodyhub中当前的contorlID值
        try:
            rospy.wait_for_service('MediumSize/BodyHub/GetMasterID',1)
        except:
            print ('error: wait_for_service GetMasterID!',sys.exc_info())
            return 0
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetMasterID', SrvTLSstring)
        response = client('get') # 调用MediumSize/BodyHub/GetMasterID服务获取当前的controlID
        self.controlID= response.data
