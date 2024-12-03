#!/usr/bin/env python
# coding=utf-8
import time
import math
import rospy
from sensor_msgs.msg import MultiDOFJointState
from geometry_msgs.msg import Transform
from ik_lib.paramofposture import ParameterOfPosture
from ik_lib.ikmodulesim import IkModuleSim
from ik_module.msg import PoseArray
from bodyhub.srv import SrvString
from ik_lib.ikmodulesim.CtrlType import CtrlType as C

Shoulder2_X = 0.0
Shoulder2_Y = 0.08138
Shoulder2_Z = 0.18392

Shoulder1_X = 0.00
Shoulder1_Y = 0.00
Shoulder1_Z = 0.00

Elbow1_X = 0.0
Elbow1_Y = 0.07581
Elbow1_Z = 0.0

Wrist1_X = 0.00
Wrist1_Y = 0.08179
Wrist1_Z = 0.0

count = 100

toPosesPub = rospy.Publisher('MediumSize/IKmodule/toPoses', MultiDOFJointState, queue_size=10)


class ClimbStairs(IkModuleSim):
    def __init__(self):
        super(ClimbStairs, self).__init__()

    def waitJointTrajDone(self):
        rospy.wait_for_service('MediumSize/BodyHub/GetStatus', 30)
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetStatus', SrvString)
        while not rospy.is_shutdown():
            response = client.call('get')
            if response.jointQueueSize > 0 or response.poseQueueSize > 0:
                break
        while not rospy.is_shutdown():
            response = client.call('get')
            if response.jointQueueSize == 0 and response.poseQueueSize == 0:
                break
            time.sleep(0.1)

    def toInitPoses(self):
        if self.setstatus() and self.getposes():
            return True
        return False

    def waitPostureDone(self):
        rospy.wait_for_service('MediumSize/BodyHub/GetStatus', 30)
        jointTraj = rospy.ServiceProxy('MediumSize/BodyHub/GetStatus', SrvString)
        rospy.wait_for_service('MediumSize/IKmodule/GetStatus', 30)
        posture = rospy.ServiceProxy('MediumSize/IKmodule/GetStatus', SrvString)
        while not rospy.is_shutdown():
            response1 = jointTraj.call('get')
            response2 = posture.call('get')
            if (response1.jointQueueSize == 0 and response1.poseQueueSize == 0) and response2.poseQueueSize == 0:
                break
            time.sleep(0.1)

    def body_motion(self, type, value, count=300):
        """
        Args:
            - type: Ctrl_Type, or [Ctrl_Type]
            - value: list, or two dim list. RPY, xyz.
            - count: int. times of division

        """
        if not (isinstance(type, list) or isinstance(type, tuple)):
            type = [type]
            value = [value]

        d = {}

        for t, diff in zip(type, value):
            if isinstance(t.key(), str):
                d[t.key()] = diff
            else:
                for key, dif in zip(t.key(), diff):
                    d[key] = dif

        for index in range(count):
            for k, v in d.items():
                setattr(self.PosPara_wF, k, getattr(self.PosPara_wF, k) + v * 1.0 / count)

            poseArrayMsg = PoseArray()
            poseArrayMsg.poses.append(self.getleftlegPosMsg())
            poseArrayMsg.poses.append(self.getrightlegPosMsg())
            poseArrayMsg.poses.append(self.getleftarmPosMsg())
            poseArrayMsg.poses.append(self.getrightarmPosMsg())
            poseArrayMsg.controlId = 6
            self.targetPosesPub.publish(poseArrayMsg)

        rospy.loginfo("waitPostureDone...")
        self.waitPostureDone()

    def climb_stairs(self, direction):
        """
        Args:
            - direction: 'up' or 'down'
        """
        if self.toInitPoses():
            self.getposes()
            if direction == 'up':
                self.body_motion(C.Torso, [0, 0, 0, 0, 0.0, -0.04])  # 重心下降 4cm
                self.body_motion(C.Torso_y, 0.12)  # 重心向左腿偏移 12cm
                self.body_motion(C.Rfoot, [0, 0, 0, 0, 0.0, 0.05])  # 右腿向上移动 5cm
                self.body_motion([C.Torso, C.Rfoot],
                                 [[0, -math.pi / 8, 0, 0.0, 0, 0], [0, 0, 0, 0.20, 0.0, 0.0]])  # 右腿向前20cm
                self.body_motion([C.Torso, C.Rfoot],
                                 [[0, math.pi / 8, 0, 0.08, -0.12, 0.0], [0, 0, 0, 0, 0.0, -0.02]])  # 右腿下2cm

                self.body_motion([C.Torso, C.Lfoot], [[math.pi / 8, math.pi / 6, 0, 0.16, -0.10, 0.05],
                                                      [0, -math.pi / 6, 0, 0.06, 0.0, 0.14]])  # 抬左腿1
                self.body_motion([C.Torso, C.Lfoot], [[0.0, -math.pi / 9, 0, -0.04, 0.0, 0.0],
                                                      [0, math.pi / 6, 0, 0.14, 0.0, -0.05]])  # 左腿向前20cm
                self.body_motion([C.Torso, C.Lfoot],
                                 [[-math.pi / 8, -math.pi / 18, 0, 0, 0.10, 0.02], [0, 0, 0, 0.0, 0.0, -0.06]])  # #复原

            elif direction == 'down':
                self.body_motion(C.Torso_z, -0.05)  # 重心下降 5cm
                self.body_motion(C.Torso_y, 0.12)  # 重心向左腿偏移 12cm
                self.body_motion(C.Rfoot, [0, 0, 0, 0, 0.0, 0.01])  # 右腿向上移动 1cm
                self.body_motion([C.Torso, C.Rfoot],
                                 [[0, -math.pi / 8, 0, 0.0, 0, 0], [0, 0, 0, 0.14, 0.0, 0.0]])  # 右腿向前14cm
                self.body_motion([C.Torso, C.Rfoot],
                                 [[0, 0, 0, 0.0, 0.0, 0.0], [0, -math.pi / 8, 0, 0, 0.0, 0.0]])  # 右脚向上旋转
                self.body_motion([C.Torso, C.Rfoot], [[0, math.pi / 8, 0, 0.12, -0.22, -0.02],
                                                      [0, math.pi / 8, 0, 0.0, 0.0, -0.04]])  # 重心移右腿
                self.body_motion([C.Torso, C.Lfoot], [[0, 0, 0, 0.02, 0.0, 0], [0, 0, 0, 0.14, 0.0, 0.01]])  # 左腿向前14cm
                self.body_motion([C.Torso, C.Lfoot], [[0, 0, 0, 0.0, 0.10, 0.04], [0, 0, 0, 0.0, 0.0, -0.04]])  # 复原


if __name__ == '__main__':
    rospy.init_node('climb_stairs', anonymous=True)
    time.sleep(0.2)
    climb_stairs = ClimbStairs()


    def shut_down():
        climb_stairs.reset()

    rospy.on_shutdown(shut_down)

    # start the simulation once
    climb_stairs.climb_stairs('up')
