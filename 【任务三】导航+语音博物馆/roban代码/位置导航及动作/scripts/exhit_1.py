#!/usr/bin/env python
# coding=utf-8

from lejulib import *
import rospy
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import subprocess
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose
from bodyhub.srv import * 
from actexecpackage.srv import *


TRACK_SCRIPT = "/home/lemon/Documents/path_track/Task_path_tracking.py"
DETCET_TASK =  "/home/lemon/robot_ros_application/catkin_ws/src/ros_museum_docent/scripts/main.py"
FACE_TASK = "/home/lemon/robot_ros_application/catkin_ws/src/ros_demo_case/face_tracking/face_tracking.py"


ACTION_FINISH_TOPIC = "/subtask_done"
is_finished = False

    
def timer_event(event):
    clear_octo.call()

    # goal_pub.publish(GOAL)
def action_finish_callback(msg):
    global is_finished
    is_finished = msg.data
init=0
if __name__ == '__main__':
    # ResetBodyhub()
    print("Here in path tracking!")
    node_initial(stateJump=False)
    clear_octo = rospy.ServiceProxy("/octomap_server/reset",Empty)
    # goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=2)
    rospy.Timer(rospy.Duration(2),timer_event)
    rospy.Subscriber(ACTION_FINISH_TOPIC, Bool, action_finish_callback, queue_size=1)
    rospy.sleep(1)
    if init==0:
        rospy.wait_for_service("/pointType")
        changeid = rospy.ServiceProxy("/pointType",SrvTLSuint)
        changeid(1)
        init=1


    print("ready to walk")
    p1 = subprocess.Popen("python {}".format(TRACK_SCRIPT.encode("utf-8")), shell=True)
    while is_finished == False:
        rospy.sleep(1)
    # rospy.wait_for_service = rospy.Service("/pointType")
    # changeid = rospy.ServiceProxy("/pointType",SrvTLSuint)
    # changeid(0)
    p1.kill()
    is_finished = False

    # p2 = subprocess.Popen("python {}".format(DETCET_TASK.encode("utf-8")), shell=True)
    # while is_finished == False:
    #     rospy.sleep(1)
    # p2.kill()
    # is_finished = False
    
    # p3 = subprocess.Popen("python {}".format(FACE_TASK.encode("utf-8")), shell=True)
    # while is_finished == False:
    #     rospy.sleep(1)
    # p3.kill()

    