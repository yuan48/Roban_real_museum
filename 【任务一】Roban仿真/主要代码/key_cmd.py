#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import math
import time
import tty
import termios
import select

import rospy
import rospkg

from std_msgs.msg import *
# from track_points import *

sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.bodyhub_client as bodycli

#!/usr/bin/env python

import os
import numpy as np
import yaml
from stage_0330 import *
from geometry_msgs.msg import *
import motion.bodyhub_client as bodycli
from ik_lib.ikmodulesim import IkModuleSim
from ik_lib.ikmodulesim.CtrlType import CtrlType as C
import algorithm.pidAlgorithm as pidAlg
from motion.motionControl import ResetBodyhub, GetBodyhubStatus, SendJointCommand

from bodyhub.srv import SrvState

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *
from public import PublicNode
SCRIPTS_PATH=os.path.split(sys.argv[0])[0]\

from test_all320 import *

NODE_NAME = 'my_cmd_node'
CONTROL_ID = 6
with open(os.path.join(SCRIPTS_PATH,"my_points.yaml"),"r")as f:
    SLAM_POINT=yaml.load(f)
    print(SLAM_POINT)


STEP_LEN = [0.1, 0.1, 10.0]
STEP_NUM=4

class Action(object):
    '''
    robot action
    '''

    def __init__(self, name, ctl_id):
        self.bodyhub = bodycli.BodyhubClient(6)
        # rospy.init_node(name, anonymous=True)
        time.sleep(0.2)
        rospy.on_shutdown(self.__ros_shutdown_hook)
        # bodycli.BodyhubClient.__init__(self,6)

    def __ros_shutdown_hook(self):
        if self.bodyhub.reset() == True:
            rospy.loginfo('bodyhub reset, exit')
        else:
            rospy.loginfo('exit')

    def bodyhub_ready(self):
        if self.bodyhub.ready() == False:
            rospy.logerr('bodyhub to ready failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def bodyhub_walk(self):
        if self.bodyhub.walk() == False:
            rospy.logerr('bodyhub to walk failed!')
            rospy.signal_shutdown('error')
            time.sleep(1)
            exit(1)

    def start(self):
        print('action start')


class Walking(Action):
    def __init__(self):
        # super(Walking, self).__init__()
        super(Walking, self).__init__('walk_telecontrol', 6)
        # self.bodyhub = bodycli.BodyhubClient(2)
        # bodycli.BodyhubClient.__init__(self,4)
        # IkModuleSim.__init__(self) 
        ResetBodyhub()     ##这个很重要
        while GetBodyhubStatus().data != "preReady":
            time.sleep(0.1)
            ResetBodyhub()
            continue
        self.step_len = STEP_LEN
        self.step_num = STEP_NUM
        self.timeout = 100
        self.gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=2)
        self.action = actions()


    def printTeleInfo(self):
        print('\n%-15s%s' % (' ', 'w--forward'))
        print('%-15s%-15s%-15s' % ('a--left', 's--backward', 'd--right'))
        print('%-15s%-15s%-15s' % ('z--trun left', 'x--in situ', 'c--trun right'))
        print('%-15s%s\n' % (' ', 'q--quit'))

    def walking(self, delta_x, delta_y, theta):
        rospy.wait_for_message('/requestGaitCommand', Bool, self.timeout)
        self.gait_cmd_pub.publish(data=[delta_x, delta_y, theta])

    def walking_n_steps(self, x, y, a, n):
        for i in range(0, n):
            self.walking(x, y, a)
            print('%s %-10s%-10s%-10s%-10s'%('step', i+1, x, y, a))
        self.printTeleInfo()

    def getch(self, str=''):
        print(str,)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print(ch)
        return ch

    def getKey(self, key_timeout):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ' '
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def keyboard_poll(self):
        while not rospy.is_shutdown():
            cmd = self.getch('key:')
            if cmd == 'w':
                self.walking_n_steps(self.step_len[0], 0.0, 0.0, self.step_num)
            elif cmd == 's':
                self.walking_n_steps(-self.step_len[0]*0.8, 0.0, 0.0, self.step_num)
            elif cmd == 'a':
                self.walking_n_steps(0, self.step_len[1], 0.0, self.step_num)
            elif cmd == 'd':
                self.walking_n_steps(0, -self.step_len[1], 0.0, self.step_num)
            elif cmd == 'z':
                self.walking_n_steps(0.0, 0.0, self.step_len[2], self.step_num)
            elif cmd == 'c':
                self.walking_n_steps(0.0, 0.0, -self.step_len[2], self.step_num)
            elif cmd == 'x':
                self.walking_n_steps(0.0, 0.0, 0.0, self.step_num)
            elif cmd == 'x':
                self.walking_n_steps(0.0, 0.0, 0.0, self.step_num)
            elif cmd == 'q':
                return

    def start(self):
        self.bodyhub_walk()
        self.printTeleInfo()
        self.keyboard_poll()
        rospy.signal_shutdown('exit')

    def printMyTele(self):
        print('%-15s%-15s%-15s' % ('a--left', 'w--forward', 'd--right'))
        print('%-15s%-15s%-15s' % ('1--position 1', '2--position 2', '3--position 3'))
        print('%-15s%s\n' % (' ', 'q--quit'))

    def keyboard_museum(self):
        while not rospy.is_shutdown():
            cmd = self.getch('key:')
            if cmd == 'w':
                self.walking_n_steps(self.step_len[0], 0.0, 0.0, self.step_num)
                # action.reset()
            elif cmd == 's':
                self.walking_n_steps(-self.step_len[0]*0.8, 0.0, 0.0, self.step_num)
                # action.reset()
            elif cmd == 'a':
                self.walking_n_steps(0, self.step_len[1], 0.0, self.step_num)
                # action.reset()
            elif cmd == 'd':
                self.walking_n_steps(0, -self.step_len[1], 0.0, self.step_num)
                # action.reset()
            elif cmd == 'z':
                self.walking_n_steps(0.0, 0.0, self.step_len[2], self.step_num)
                # action.reset()
            elif cmd == 'c':
                self.walking_n_steps(0.0, 0.0, -self.step_len[2], self.step_num)
                # action.reset()
            elif cmd == 'x':
                self.walking_n_steps(0.0, 0.0, 0.0, self.step_num)
                # action.reset()
            elif cmd == 'q':
                return
            elif cmd== '0':
                # action.setStatusClient.call(6, "reset")
                # self.setStatusClient.call(10, "setStatus")
                # self.setStatusClient = rospy.ServiceProxy('MediumSize/IKmodule/SetStatus', SrvState)

                # rosservice call /MediumSize/BodyHub/StateJump "masterID: 6 stateReq: 'setStatus'"
                Poses.A_pos(1)
                time.sleep(0.1)
                action.reset()
                print("in actions")
                if action.toInitPoses():
                    action.body_motion([C.RArm_z, C.RArm_y], [0.05, -0.05], 50)
                    action.body_motion([C.RArm_z, C.RArm_x], [0.11, 0.04], 50)
                    action.body_motion([C.RArm_x, C.RArm_y], [0.09, 0.03], 10)
                action.reset()
                # ResetBodyhub()
            elif cmd== '1':
                print("heading position 1!")
                Poses.A_pos(1)
                time.sleep(0.1)
                action.reset()
                self.colordetect()
                if action.toInitPoses():
                    action.body_motion([C.RArm_z, C.RArm_y], [0.05, -0.05], 50)
                    action.body_motion([C.RArm_z, C.RArm_x], [0.11, 0.04], 50)
                    action.body_motion([C.RArm_x, C.RArm_y], [0.09, 0.03], 10)
                action.reset()
                time.sleep(0.5)
                    # ResetBodyhub()

            elif cmd== '2':
                self.bodyhub_ready()
                print("heading position 2!")
                Poses.A_pos(1.2)
                time.sleep(0.5)
                action.reset()
                self.bodyhub_ready()
                Poses.A_pos(2)
                self.colordetect()
                if action.toInitPoses():
                    action.body_motion([C.RArm_z, C.RArm_y], [0.05, -0.05], 50)
                    action.body_motion([C.RArm_z, C.RArm_x], [0.11, 0.04], 50)
                    action.body_motion([C.RArm_x, C.RArm_y], [0.09, 0.03], 10)
                action.reset()
            elif cmd== '3':
                self.bodyhub_ready()
                print("heading position 3!")
                Poses.A_pos(3)
                self.colordetect()
                if action.toInitPoses():
                    action.body_motion([C.RArm_z, C.RArm_y], [0.05, -0.05], 50)
                    action.body_motion([C.RArm_z, C.RArm_x], [0.11, 0.04], 50)
                    action.body_motion([C.RArm_x, C.RArm_y], [0.09, 0.03], 10)
                    action.reset()
            elif cmd== '4':
                self.bodyhub_ready()
                print("heading position 4!")
                Poses.A_pos(4)
                self.colordetect()
                if action.toInitPoses():
                    action.body_motion([C.RArm_z, C.RArm_y], [0.05, -0.05], 50)
                    action.body_motion([C.RArm_z, C.RArm_x], [0.11, 0.04], 50)
                    action.body_motion([C.RArm_x, C.RArm_y], [0.09, 0.03], 10)
                    action.reset()
            elif cmd== '5':
                self.bodyhub_ready()
                print("heading position 5!")
                Poses.A_pos(5)
                colordetect()
                if action.toInitPoses():
                    action.body_motion([C.RArm_z, C.RArm_y], [0.05, -0.05], 50)
                    action.body_motion([C.RArm_z, C.RArm_x], [0.11, 0.04], 50)
                    action.body_motion([C.RArm_x, C.RArm_y], [0.09, 0.03], 10)
                    action.reset()


    
    def start_my(self):
        print("start in")
        self.bodyhub_walk()
        print("ready walk")
        self.printMyTele()
        self.keyboard_museum()
        rospy.signal_shutdown('exit')

class actions(IkModuleSim):
    def __init__(self):
        # super(actions, self).__init__()
        IkModuleSim.__init__(self)
    def arms(self):
        print("in actions")
        self.body_motion([C.RArm_z, C.RArm_y], [0.05, -0.05], 50)
        self.reset()


    
class Run_node(PublicNode):
    def __init__(self,nodename=NODE_NAME,control_id=6):
        super(Run_node, self).__init__(nodename,6)

    def A_pos(self, num):
        '''
        pos_wait_mode=1:等待机器人站稳后才进行定位,精度高但慢
        ignore_angle模式:用于回程运动,忽略标记点的方向
        wait_time:等待机器人定位稳定的时间
        pos_exit_high_acc:确保终点精度高,调节时间会增加
        '''
        # self.set_arm_mode(1)
        self.bodyhub_walk()
        self.path_tracking(SLAM_POINT["pos{}".format(num)],pos_wait_mode=1,wait_time=0.1,pos_exit_high_acc=False)
        self.bodyhub.wait_walking_done()
    def set_arm(self):
        self.set_arm_mode(0)
    

Poses=Run_node()
action=actions()
test=test_movement()
# Moves=test_movement(debug= True)

if __name__ == '__main__':
    # rospy.init_node('action', anonymous=True)
    Walking().start_my()
