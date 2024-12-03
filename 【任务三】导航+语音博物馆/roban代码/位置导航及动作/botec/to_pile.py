#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import threading
import numpy as np
import rospy
import rospkg
import bodyhub_action as bodyact
import navigation as nav


class ToPile(bodyact.Action, nav.Navigation):
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        nav.Navigation.__init__(self, body_client, False)
        self.bodyhub = body_client
        self.marker_debug_print_on = 0b01
        self.slam_debug_print_on = 0b10

    def debug(self):
        self.bodyhub_ready()
        self.set_debug_print(self.marker_debug_print_on)
        while not rospy.is_shutdown():
            time.sleep(0.01)

    def start(self):
        self.bodyhub_walk()
        self.bodyhub.walking_n_steps([0.08, 0.0, 0.0], 1)
        self.bodyhub.wait_walking_done()
        self.goto_pose(0, [0.22, -0.18, 0.0])

    
if __name__ == '__main__':
    from motion import bodyhub_client as bodycli

    rospy.init_node('botech_to_pile_node', anonymous=True)
    time.sleep(0.2)
    obj = ToPile(bodycli.BodyhubClient(2))
    if len(sys.argv) >= 2 and (sys.argv[1] == 'debug'):
        obj.debug()
    else:
        obj.start()