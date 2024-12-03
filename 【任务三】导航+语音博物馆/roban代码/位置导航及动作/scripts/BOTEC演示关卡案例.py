#!/usr/bin/env python
# coding=utf-8
import os
from lejulib import *
import time

def main():
    node_initial()

    try:
        # os.system('cd ~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/shell && ./botec.sh')
        # os.system(os.environ['/home/lemon/robot_ros_application/catkin_ws/devel/setup.sh'])
        os.system('cd ~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/shell && ./slam_node.sh')
        os.system('. /home/lemon/robot_ros_application/catkin_ws/devel/setup.sh && roslaunch ar_track_alvar roban_chin_camera.launch &')
        os.system('. /home/lemon/robot_ros_application/catkin_ws/devel/setup.sh && rosrun ros_actions_node pose_board_company.py')
        os.system('rosnode kill /RGBD')
        os.system('cd /home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/shell && ./stair_node.sh')
        os.system('rosnode kill /action_node')
        os.system('. /home/lemon/robot_ros_application/catkin_ws/devel/setup.sh && rosrun ros_actions_node to_pile_company.py')
        os.system('cd /home/lemon/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/shell && ./pile_node.sh')
        os.system('rosnode kill $(rosnode list | grep color_detect)')
        os.system('rosnode kill /chin/ar_track_alvar')
        os.system('rosnode kill /action_node')

    except Exception as err:
        serror(err)

    finally:
        finishsend()

if __name__ == '__main__':
    main()
