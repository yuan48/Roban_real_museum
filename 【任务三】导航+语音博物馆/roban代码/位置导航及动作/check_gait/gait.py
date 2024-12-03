#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import rospy
import rospkg
import bodyhub_action as bodyact
from std_msgs.msg import String

sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts')
from lejulib import *


class PoseBoard(bodyact.Action):
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        self.bodyhub = body_client
        
    def start(self):
    
        self.bodyhub_walk()
        # self.bodyhub.walking_n_steps([0.00, 0.0, 0.0], 3)
        self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 9)
        self.bodyhub.wait_walking_done()

def terminate(data):
    rospy.loginfo(data.data)
    rospy.signal_shutdown("kill")

if __name__ == '__main__':
    from motion import bodyhub_client as bodycli

    rospy.init_node('botech_pose_board_node', anonymous=True)
    rospy.Subscriber('terminate_current_process', String, terminate)
    time.sleep(0.2)
    obj = PoseBoard(bodycli.BodyhubClient(2))
    obj.start()
