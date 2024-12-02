#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os
import time
import numpy as np
import rospy
import rospkg
import yaml
from std_msgs.msg import *
from geometry_msgs.msg import *

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from public import PublicNode
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]

NODE_NAME = 'my_cmd_node'
CONTROL_ID = 6
with open(os.path.join(SCRIPTS_PATH,"my_points.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)

# 搬运动作开始后需要使用调用set_arm_mode(0)函数取消掉手部的行走动作

class Run_node(PublicNode):
    def __init__(self,nodename=None,control_id=None):
        super(Run_node, self).__init__(nodename,control_id)

    def A_pos(self, num):
        '''
        pos_wait_mode=1:等待机器人站稳后才进行定位,精度高但慢
        ignore_angle模式:用于回程运动,忽略标记点的方向
        wait_time:等待机器人定位稳定的时间
        pos_exit_high_acc:确保终点精度高,调节时间会增加
        '''
        self.set_arm_mode(1)
        self.bodyhub_walk()
        self.path_tracking(SLAM_POINT["pos{}".format(num)],pos_wait_mode=1,wait_time=0.1,pos_exit_high_acc=False)
        self.bodyhub.wait_walking_done()

if __name__ == '__main__':
    Run= Run_node()
    Run.A_pos(1)

