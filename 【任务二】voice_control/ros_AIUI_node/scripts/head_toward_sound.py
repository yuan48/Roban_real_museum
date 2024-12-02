#! /usr/bin/python
import sys
import os
import signal
import json
import rospy
from std_msgs.msg import String
import array
from bodyhub.srv import *  # for SrvState.srv
from ros_AIUI_node.srv import demoIsRunning
import rospkg
sys.path.append(rospkg.RosPack().get_path('ros_actions_node')+ '/scripts')
from lejulib import *
from motion.motionControl import SetBodyhubTo_setStatus, GetBodyhubStatus
SERVO = client_action.SERVO


def set_head_servo(angles):
    """set head servos angle

    :param angles:[angle, 0.0]
    :return:
    """
    array_angles = array.array("d", angles)
    SERVO.HeadJointTransfer(array_angles)
    SERVO.MotoWait()


def terminate():
    """Terminate all threads
    """
    pid = os.getpid()
    print('kill pid = {}'.format(pid))
    os.kill(pid, signal.SIGKILL)
    sys.exit()


def get_angle_value(angle):
    """

    :param angle: the angle from micarrays
    :return: [-90.0 , 90.0]
    """
    angle = 300.0 - angle
    if angle > 180.0 and angle <= 300.0:
        angle = angle - 360.0
        return angle
    else:
        return angle


def moto_destination(preAngle, angle):
    """

    :param angle: the angle from micarrays
    :return:
    """
    angle_value = get_angle_value(angle)
    moto_angle = preAngle + angle_value
    if moto_angle > 90.0:
        moto_angle = 90.0
    if moto_angle < -90.0:
        moto_angle = -90.0
    rospy.loginfo("angle:%s", str(moto_angle))
    set_head_servo([moto_angle, 0.0])
    return moto_angle


def main():
    try:
        rospy.init_node("head_toward_sound", anonymous=False)
        sound_topic = "/micarrays/wakeup"
        preAngle = client_action.get_servo_value(21)
        demo_running_status_client = rospy.ServiceProxy("aiui/demo_running_status", demoIsRunning)

        while not rospy.is_shutdown():
            msg = rospy.wait_for_message(sound_topic, String)
            data = demo_running_status_client()
            is_demo_running = data.demoIsRunning
            if is_demo_running == False:
                data = json.loads(msg.data.replace("'", '"'))
                angle = float(data['angle'])
                rospy.loginfo("angle-----:%s", str(angle))
                current_status = GetBodyhubStatus()
                if current_status.data != "ready" and current_status.data != "pause":
                    SetBodyhubTo_setStatus(2)
                preAngle = moto_destination(preAngle, angle)

    except Exception as err:
        serror(err)
    finally:
        finishsend()


if __name__ == '__main__':
    main()
