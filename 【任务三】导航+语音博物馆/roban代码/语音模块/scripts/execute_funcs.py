#!/usr/bin/env python3
# coding=utf-8
import os
import queue as Queue
import sys
import rospkg
import rospy
import time
import threading
from cn_num import cn2dig
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))

from motion.motionControl import ResetBodyhub,SetBodyhubTo_walking,WalkingNSteps,WaitForWalkingDone

CONTROL_ID = 2
STEP_LEN = [0.06, 0.04, 10]


class Exec_func(threading.Thread):
    def __init__(self, ):
        super().__init__()
        self.__actQueue = Queue.Queue()

    def run(self) -> None:
        while not rospy.is_shutdown():
            if self.__actQueue.empty():
                time.sleep(0.1)
            else:
                action_cmd = self.__actQueue.get()
                print("get actioncmd", action_cmd)
                if "func" in action_cmd:
                    try:
                        action = getattr(self, action_cmd["func"])
                        action(action_cmd["argv"])
                    except Exception as e:
                        rospy.logwarn("action failed! {}".format(e))

    def exec(self, action_str):
        self.__actQueue.put(action_str)

    def parse_steps(self, cmd_str):
        step = None
        S = ["步", "度", "°"]
        P = ["走", "转", "退", "进"]
        for suffix in S:
            for prefix in P:
                if suffix in cmd_str and prefix in cmd_str:
                    id_p = cmd_str.index(prefix)
                    id_s = cmd_str.index(suffix)
                    try:
                        num_str = cmd_str[id_p+1:id_s]
                        c = int(num_str) if num_str.isdigit(
                        ) else cn2dig(num_str)
                    except Exception as e:
                        print(sys.exc_info(), __file__)
                    else:
                        step = c
                        print("count:", c)
                    break
        return step

    def walk_forward(self, cmd_str=None):
        step = self.parse_steps(cmd_str)
        SetBodyhubTo_walking(CONTROL_ID)
        WalkingNSteps([STEP_LEN[0], 0, 0], step if step is not None else 1)
        WaitForWalkingDone()
        ResetBodyhub()

    def walk_back(self, cmd_str=None):
        step = self.parse_steps(cmd_str)
        SetBodyhubTo_walking(CONTROL_ID)
        WalkingNSteps([-STEP_LEN[0], 0, 0], step if step is not None else 1)
        WaitForWalkingDone()
        ResetBodyhub()

    def walk_left(self, cmd_str=None):
        step = self.parse_steps(cmd_str)
        SetBodyhubTo_walking(CONTROL_ID)
        WalkingNSteps([0, STEP_LEN[1], 0], step if step is not None else 1)
        WaitForWalkingDone()
        ResetBodyhub()

    def walk_right(self, cmd_str=None):
        step = self.parse_steps(cmd_str)
        SetBodyhubTo_walking(CONTROL_ID)
        WalkingNSteps([0, -STEP_LEN[1], 0], step if step is not None else 1)
        WaitForWalkingDone()
        ResetBodyhub()

    def turn_left(self, cmd_str=None):
        angle = self.parse_steps(cmd_str)
        step = angle//STEP_LEN[2] if angle is not None and angle > STEP_LEN[2] else 1
        SetBodyhubTo_walking(CONTROL_ID)
        WalkingNSteps([0, 0, STEP_LEN[2]], step)
        WaitForWalkingDone()
        ResetBodyhub()

    def turn_right(self, cmd_str=None):
        angle = self.parse_steps(cmd_str)
        step = angle//STEP_LEN[2] if angle is not None and angle > STEP_LEN[2] else 1
        SetBodyhubTo_walking(CONTROL_ID)
        WalkingNSteps([0, 0, -STEP_LEN[2]], step)
        WaitForWalkingDone()
        ResetBodyhub()
