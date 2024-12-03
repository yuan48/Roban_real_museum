#!/usr/bin/python
# coding=utf-8
import os
import json
import rospy
import subprocess
import time
from std_msgs.msg import String
from bodyhub.srv import *  # for SrvState.srv
from lejulib import *
from ros_face_node.srv import FaceDetect
from ros_fruit_node.srv import FruitCognition
GAIT_RANGE = 0.05
ROTATION_RANGE = 10.0
FACE_CENTER_X = 160
FACE_CENTER_Y = 120
FACE_THRESHOLD_FORWARD = 5000
FACE_THRESHOLD_BACKWARD = 10000
Walker = client_walk.WalkTransfer()


def slow_walk(direction, stepnum=None, angle=None):
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
        array[2] = ROTATION_RANGE if angle > 0 else -1*ROTATION_RANGE
        stepnum = int(abs(angle) / ROTATION_RANGE)
    else:
        rospy.logerr("error walk direction")
    Walker.walkingPublish(array,stepnum)
    Walker.waitForWalkingDone()


def getfile(fname):
    """

    :param fname:
    :return:
    """
    return os.path.join(os.environ['HOME'], 'Music', 'greetings', fname)


def get_angle_value(angle):
    """

    :param angle: the angle from micarrays
    :return: [-180.0 , 180.0]
    """
    angle = 300.0 - angle
    if angle > 180.0 and angle <= 300.0:
        angle = angle - 360.0
        return angle
    else:
        return angle

def facelocation():
    """

    :return:
    """
    rospy.wait_for_service('ros_face_node/face_detect')
    client = rospy.ServiceProxy('ros_face_node/face_detect', FaceDetect)
    result = client()
    face_x, face_y, face_area = result.result
    return face_x, face_y, face_area


def facedetect():
    """
    :return:
    """
    result = client_face.face_service()
    target_face = client_face.face_filter(result)
    gender = target_face.get('gender')['type']
    age = target_face.get('age')
    return gender,age


def fruitdetect():
    """
    :return:
    """
    rospy.wait_for_service("ros_fruit_node/fruit_cognition")
    client = rospy.ServiceProxy("ros_fruit_node/fruit_cognition", FruitCognition)
    while not rospy.is_shutdown():
        result = client().result
        print(result)
        if result == "苹果":
            rospy.loginfo("苹果")
            subprocess.check_output(['play', getfile("apple.wav")])
        elif result == "香蕉":
            rospy.loginfo("香蕉")
            subprocess.check_output(['play', getfile("banana.wav")])
        elif result == "橙子":
            rospy.loginfo("橙子")
            subprocess.check_output(['play', getfile("orange.wav")])
        elif result == "梨":
            rospy.loginfo("梨")
            subprocess.check_output(['play', getfile("pear.wav")])
        elif result == "红心火龙果":
            rospy.loginfo("红心火龙果")
            subprocess.check_output(['play', getfile("pitaya.wav")])
        else:
            rospy.loginfo("没有识别到水果，请重新识别一次")
            time.sleep(2)

def walktarget():
    """
    :return:
    """
    while not rospy.is_shutdown():
        face_x,face_y,area = facelocation()
        x_err = FACE_CENTER_X - face_x
        if abs(x_err) > 20:
            angle = ROTATION_RANGE if x_err > 0 else -1*ROTATION_RANGE
            slow_walk("rotation", angle=angle)
        else:
            break


def motowalk(angle):
    """

    :param angle:
    :return:
    """
    angle_value = get_angle_value(angle)
    rospy.loginfo("angle:%s", str(angle_value))
    slow_walk("rotation",angle=angle_value)
    face_x,face_y,area = facelocation()
    while area < FACE_THRESHOLD_FORWARD:
        slow_walk("forward",2)
        walktarget()
        face_x,face_y,area = facelocation()
    while area > FACE_THRESHOLD_BACKWARD:
        slow_walk("backward",2)
        walktarget()
        face_x,face_y,area = facelocation()


def main():
    try:
        node_initial()
        sound_topic = "/micarrays/wakeup"
        msg = rospy.wait_for_message(sound_topic, String)
        data = json.loads(msg.data.replace("'", '"'))
        angle = float(data['angle'])
        Walker.walkingClient("walking")
        motowalk(angle)
        gender,age = facedetect()
        if gender == 'male':
            if age > 30:
                print("hello gentlenman")
                subprocess.check_output(['play', getfile("hello_gentleman.wav")])
            else:
                print("hello xiaogege")
                subprocess.check_output(['play', getfile("hello_xiaogege.wav")])
        else:
            if age > 30:
                print("hello madam")
                subprocess.check_output(['play', getfile("hello_madam.wav")])
            else:
                print("hello xiaojiejie")
                subprocess.check_output(['play', getfile("hello_xiaojiejie.wav")])
        subprocess.check_output(['play', getfile("greeting.wav")])
        fruitdetect()
    except Exception as err:
        serror(err)
    finally:
        finishsend()
        Walker.walkingClient("stop")


if __name__ == '__main__':
    main()