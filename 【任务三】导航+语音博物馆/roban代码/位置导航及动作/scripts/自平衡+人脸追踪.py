#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy,sys,os

sys.path.append(os.path.split(sys.argv[0])[0])
from balance_face_tracking.balance_board import balance_board_start,balance_board_stop
from balance_face_tracking.face_tracking import face_tracking_start,face_tracking_stop
from lejulib import *


def on_shutdown(date=None):
    face_tracking_stop()
    balance_board_stop()
    
if __name__ == "__main__":
    rospy.init_node("balance_face_tracking", anonymous=True)
    rospy.Subscriber('terminate_current_process', String, terminate)
    balance_board_start()
    face_tracking_start()
    rospy.on_shutdown(on_shutdown)
    rospy.spin()

