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

from motion import bodyhub_client as bodycli

import seesaw
import turntable
import open_door
import pendulum_bob
import minefield
import pose_board
import to_pile


class Botec():
    """botec比赛前三关启动程序:

            runRace = Botec(body_client)
            runRace.start() 

    """
    def __init__(self, body_client):
        self.body_client = body_client

    def start(self):
        """调用三个关卡的程序，依次执行
        
        """
        seesaw.Seesaw(self.body_client).start()
        turntable.Turntable(self.body_client).start()
        open_door.OpenDoor(self.body_client).start()
        # pendulum_bob.PendulumBob(self.body_client).start()
        # minefield.Minefield(self.body_client).start()
        # pose_board.PoseBoard(self.body_client).start()


if __name__ == '__main__':
    rospy.init_node('botec_node', anonymous=True)
    time.sleep(0.2)
    Botec(bodycli.BodyhubClient(2)).start()
