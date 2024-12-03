#!/bin/bash

motion()
{
    roscore > /tmp/artagcheck.log 2>&1
    sleep 1
    cd ~/robot_ros_application/scripts/
    echo "softdev" | sudo -S bash bodyhub.sh > /tmp/artagcheck.log 2>&1 &
    sleep 2
}

walking()
{
    . /home/lemon/robot_ros_application/catkin_ws/devel/setup.sh
    rosservice call /imuState "state: 'off'"
    cd ~/robot_ros_application/catkin_ws/
    . devel/setup.sh
    rosrun ros_actions_node check_walking.py
    rosservice call /imuState "state: 'on'"
}

motion
walking

rosnode kill /BodyHubNode