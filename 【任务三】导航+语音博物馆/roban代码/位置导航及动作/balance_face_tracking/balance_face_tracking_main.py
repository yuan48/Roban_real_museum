#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import sys, tty, termios
import rospkg
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))

from std_msgs.msg import *
from mediumsize_msgs.srv import *
import motion.motionControl as mCtrl
import os
import cv2
import signal
import Queue
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import array
import threading
from bodyhub.srv import *  # for SrvState.srv
from bodyhub.msg import JointControlPoint
from lejulib import *

from balance_board import balance_board_start,balance_board_stop
from face_tracking import face_tracking_start,face_tracking_stop

def on_shutdown():
    face_tracking_stop()
    balance_board_stop()
if __name__ == "__main__":
    debug=True if len(sys.argv)>1 else False
    rospy.init_node("balance_face_tracking", anonymous=True)
    rospy.sleep(0.2)
    balance_board_start()
    face_tracking_start(debug=debug)
    rospy.on_shutdown(on_shutdown)
    rospy.spin()

