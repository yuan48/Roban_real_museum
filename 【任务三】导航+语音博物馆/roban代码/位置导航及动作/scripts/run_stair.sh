#!/bin/bash
source ~/robot_ros_application/catkin_ws/devel/setup.bash
jy901() 
{
    rosrun jy901_module jy901_module_node &
}

stair_node() 
{
    rosrun ljhn_torcon ljhn_torcon_node $1
}

ar_tag() 
{
    roslaunch ar_track_alvar roban_chin_camera.launch &
}

stair_convert() 
{
    rosrun ros_actions_node stair_convert_artag.py
}

set_bodyhub_status_to_upstairs()
{
    rosservice call /MediumSize/BodyHub/StateJump 2 external_control
}

set_bodyhub_status_to_preReady()
{
    rosservice call /MediumSize/BodyHub/StateJump 2 reset
}

set_bodyhub_status_to_Ready()
{
    rosservice call /MediumSize/BodyHub/StateJump 2 "setStatus"
}

kill_ar_track_alvar()
{
    rosnode kill /chin/ar_track_alvar
}

kill_jy901()
{
    rosnode kill /jy901Module_node
}

# sleep 7

jy901
stair_node 8
set_bodyhub_status_to_Ready

sleep 2
ar_tag
stair_convert
kill_ar_track_alvar

sleep 1
jy901
stair_node 9
set_bodyhub_status_to_preReady
kill_jy901

