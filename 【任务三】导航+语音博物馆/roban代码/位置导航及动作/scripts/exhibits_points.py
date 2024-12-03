#!/usr/bin/env python
# coding=utf-8

# from lejulib import *
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import subprocess
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from bodyhub.msg import *
from bodyhub.srv import * 
from actexecpackage.srv import *


GOAL = PoseStamped() 
 


def callback(num):
    global point
    point = num.uintReq
    print("changing num to: ",num)
    return 6


if __name__ == '__main__':
    rospy.init_node("poses_pub",anonymous=True)
    goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
    s = rospy.Service("/pointType",SrvTLSuint,callback)
    point = 0
    while not rospy.is_shutdown():
        # print("point is :",point)
        GOAL.header.frame_id = 'map'
        GOAL.header.stamp = rospy.Time().now()
        if point != 0 :
            if point == 1:
                # print(1)
                GOAL.pose.position.x = 0.2
                GOAL.pose.position.y = -1
                GOAL.pose.position.z = 0
                GOAL.pose.orientation.x = 0
                GOAL.pose.orientation.y = 0
                GOAL.pose.orientation.z = -0.38
                GOAL.pose.orientation.w = 0.92
            elif point == 2:
                # print(2)
                GOAL.pose.position.x = 0.4
                GOAL.pose.position.y = 0.0
                GOAL.pose.position.z = 0
                GOAL.pose.orientation.x = 0
                GOAL.pose.orientation.y = 0
                GOAL.pose.orientation.z = 0
                GOAL.pose.orientation.w = 1
            elif point ==3:
                # print(3)
                GOAL.pose.position.x = 0.4
                GOAL.pose.position.y = 0.2
                GOAL.pose.position.z = 0
                GOAL.pose.orientation.x = 0
                GOAL.pose.orientation.y = 0
                GOAL.pose.orientation.z = 0.38
                GOAL.pose.orientation.w = 0.92
            goal_pub.publish(GOAL)
        
        else:
            goal_pub.publish()
            continue
        