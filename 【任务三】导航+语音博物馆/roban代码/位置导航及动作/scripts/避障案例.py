#!/usr/bin/python
# coding=utf-8
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from lejulib import *
import numpy as np
from motion.motionControl import *

GAIT_RANGE = 0.05
ROTATION_RANGE = 10.0
ROI = (100, 100)


def slow_walk(direction, stepnum=1, angle=None):
    """
    :param direction: "forward" ,"backward" or "rotation"
    :param stepnum: int num
    :return:
    """
    array = [0.0, 0.0, 0.0]
    if direction == "forward":
        array[0] = GAIT_RANGE
    elif direction == "backward":
        array[0] = -1 * GAIT_RANGE
    elif direction == "rotation":
        array[2] = ROTATION_RANGE if angle > 0 else -1 * ROTATION_RANGE
        stepnum = int(abs(angle) / ROTATION_RANGE)
    else:
        rospy.logerr("error walk direction")
    for _ in range(stepnum):
        SendGaitCommand(array[0], array[1], array[2])


def move(mean_distance):
    print(mean_distance)
    if mean_distance < 150:
        print("离得太近了，我识别不到了。")
        return
    elif mean_distance > 1000:
        print("前方障碍物比较远，我可以走的快一些")
        slow_walk("forward", 4)
    elif mean_distance > 500:
        print("我准备往前走了。")
        slow_walk("forward", 1)
    elif mean_distance < 250:
        print("有点近，我需要后退一下")
        slow_walk("backward", 1)
    else:
        print("前方有障碍物，我准备右转30度")
        slow_walk("rotation", angle=-30)
    WaitForWalkingDone()


def callback(image):
    cv_image = bridge.imgmsg_to_cv2(image, "16UC1")

    cv_image = np.array(cv_image)
    height, width = cv_image.shape
    roi_image = cv_image[height / 2 - ROI[1] / 2: height / 2 + ROI[1] / 2,
                         width / 2 - ROI[0] / 2: width / 2 + ROI[0] / 2]
    mean_distance = roi_image.mean()
    return mean_distance


if __name__ == "__main__":
    rospy.init_node('roban_avoidance')
    rospy.Subscriber('terminate_current_process', String, terminate)
    bridge = CvBridge()
    topic = "/camera/depth/image_rect_raw"
    print SetBodyhubTo_walking(2)

    while not rospy.is_shutdown():
        means_distance = []
        for _ in range(5):
            means_distance.append(
                callback(rospy.wait_for_message(topic, Image)))
        move(sum(means_distance) / len(means_distance))
