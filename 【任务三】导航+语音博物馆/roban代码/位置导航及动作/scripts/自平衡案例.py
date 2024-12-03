#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import sys, tty, termios

import rospy
import rospkg

from std_msgs.msg import *
from lejulib import terminate
from mediumsize_msgs.srv import *
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
import motion.motionControl as mCtrl

CONTROL_ID = 2

def setBalanceBoard():
    try:
        rospy.wait_for_service('/MediumSize/BodyHub/standType', 15)
    except:
        print 'error: wait_for_service action!'
        return False
    client = rospy.ServiceProxy('/MediumSize/BodyHub/standType', SetAction)
    response = client(1, 'balanceBoard')
    if response.result == 1:
        return True
    return False

def setWalkStand():
    try:
        rospy.wait_for_service('/MediumSize/BodyHub/standType', 15)
    except:
        print 'error: wait_for_service action!'
        return False
    client = rospy.ServiceProxy('/MediumSize/BodyHub/standType', SetAction)
    response = client(0, 'setWalkStand')
    if response.result == 0:
        return True
    return False

def rosShutdownHook():
    setWalkStand()
    mCtrl.ResetBodyhub()

if __name__ == '__main__':
    rospy.init_node('balance_board', anonymous=True)
    rospy.Subscriber('terminate_current_process', String, terminate)
    time.sleep(0.2)
    rospy.on_shutdown(rosShutdownHook)
    rospy.loginfo('node runing...')

    if mCtrl.SetBodyhubTo_setStatus(CONTROL_ID) == False:
        rospy.logerr('bodyhub to ready failed!!')
        rospy.signal_shutdown('error')
        exit(1)
    if setBalanceBoard() == False:
        rospy.logerr('setBalanceBoard error!')
        rospy.signal_shutdown('error')
        exit(1)
    if mCtrl.SetBodyhubTo_walking(CONTROL_ID) == False:
        rospy.logerr('bodyhub to walk failed!')
        rospy.signal_shutdown('error')
        exit(1)

    while not rospy.is_shutdown():
        time.sleep(0.01)
        pass
