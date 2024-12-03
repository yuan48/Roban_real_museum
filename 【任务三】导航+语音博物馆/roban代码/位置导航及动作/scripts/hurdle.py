#!/usr/bin/env python
# coding=utf-8
import time
import math
import rospy
from sensor_msgs.msg import MultiDOFJointState

from ik_lib.ikmodulesim import IkModuleSim
from bodyhub.srv import SrvString

from ik_lib.ikmodulesim.CtrlType import CtrlType as C



toPosesPub = rospy.Publisher(
    'MediumSize/IKmodule/toPoses', MultiDOFJointState, queue_size=10)

class Hurdle(IkModuleSim):
    def __init__(self):
        super(Hurdle, self).__init__()

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
        jointTraj = rospy.ServiceProxy(
            'MediumSize/BodyHub/GetStatus', SrvString)
        rospy.wait_for_service('MediumSize/IKmodule/GetStatus', 30)
        posture = rospy.ServiceProxy(
            'MediumSize/IKmodule/GetStatus', SrvString)
        while not rospy.is_shutdown():
            response1 = jointTraj.call('get')
            response2 = posture.call('get')
            if (response1.jointQueueSize == 0 and response1.poseQueueSize == 0) and response2.poseQueueSize == 0:
                break
            time.sleep(0.1)

    def main(self):
        if self.setstatus() and self.getposes():
            self.PosPara_wF.Torso_z = -self.leftLegZ
            self.PosPara_wF.Lfoot_y = self.leftLegY
            self.PosPara_wF.Rfoot_y = self.rightLegY

            self.body_motion(C.Torso, [math.pi / 12, -math.pi / 12, 0 ,0, -0.08, -0.05])
            self.body_motion(C.Lfoot, [0, 0, 0, 0, 0.05, 0.09])
            self.body_motion(C.Lfoot, [0, 0, 0, 0.20, -0.03, 0])
            self.body_motion([C.Lfoot, C.Torso], [[0, -math.pi / 6, 0, 0, 0, -0.045], [-math.pi / 12, 0, math.pi / 12, 0.04, 0.02, 0.03]])
            self.body_motion([C.Lfoot, C.Torso], [[0, math.pi / 12, 0, 0, 0, -0.025], [0, 0, 0, 0.02, 0.02, -0.02]])
            self.body_motion([C.Lfoot, C.Torso], [[0, math.pi / 12, 0, 0, 0, -0.02], [0, 0, 0, 0.04, 0.04, -0.01]])
            self.body_motion(C.Torso, [0, math.pi / 6, 0, 0.03, 0.03, 0])
            self.body_motion([C.Rfoot, C.Torso], [[0, math.pi / 12, 0, 0, 0, 0.03], [0, 0, 0, 0.03, 0.03, 0.0]])
            self.body_motion([C.Rfoot, C.Torso], [[0, math.pi / 12, 0, 0, 0, 0.03], [0, 0, 0, 0.03, 0.03, 0.0]])
            self.body_motion([C.Rfoot, C.Torso], [[0, -math.pi / 12, 0, 0, 0, 0.05], [0, 0, math.pi / 12, 0, 0, 0.03]])
            self.body_motion(C.Rfoot, [0, 0, 0, 0.20, -0.04, 0.0], 300)
            self.body_motion(C.Rfoot, [0, -math.pi / 12, 0, 0, 0.04, -0.09])
            self.reset()



if __name__ == '__main__':

    hurdle = Hurdle()
    def rosShutdownHook():
        hurdle.reset()
        rospy.signal_shutdown('node_close')

    rospy.init_node('hurdles', anonymous=True)

    rospy.on_shutdown(rosShutdownHook)

    hurdle.main()
