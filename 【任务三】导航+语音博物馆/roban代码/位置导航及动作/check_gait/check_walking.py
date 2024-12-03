#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import gait
from std_msgs.msg import String
import time
import logging

logger = logging.getLogger("出厂检测")
if not logger.handlers:
    logger.setLevel(level = logging.DEBUG)
    handler = logging.FileHandler("/home/lemon/robot_ros_application/scripts/check_routine/factory_check_log.txt")
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    # console = logging.StreamHandler()   # 输出打印屏幕
    # console.setLevel(logging.DEBUG)
    logger.addHandler(handler)
    # logger.addHandler(console)

def terminate(data):
    rospy.loginfo(data.data)
    rospy.signal_shutdown("kill")

def main(factory=False):
    from motion import bodyhub_client as bodycli

    rospy.init_node('botech_pose_board_node', anonymous=True)
    rospy.Subscriber('terminate_current_process', String, terminate)
    time.sleep(0.2)
    print('检查步态***********************************')
    if factory: logger.debug('检查步态***********************************')
    check_walking = gait.PoseBoard(bodycli.BodyhubClient(2))
    check_walking.start()
    while True:
        print("步态是否正常（Y/N）")
        if factory: logger.debug("步态是否正常（Y/N）")
        judge = raw_input()
        if judge == 'y' or judge == 'Y':
            print("步态正常")
            if factory: logger.info("步态正常")
            break
        elif judge == 'n' or judge == 'N':
            print("步态异常")
            if factory: logger.warning("步态异常")
            break
        else:
            print("请重新确认输入的字符")

if __name__ == '__main__':
    main()

