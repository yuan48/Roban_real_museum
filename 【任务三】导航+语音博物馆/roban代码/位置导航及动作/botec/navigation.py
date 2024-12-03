#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import math
import time
import threading
import copy
import numpy as np
import numpy.matlib
import scipy.linalg as linalg
import rospy
import rospkg
import tf
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *


class MarkerInfo(object):
    def __init__(self, _debug=False):
        rospy.Subscriber("/chin/visualization_marker_chin", Marker, self.chin_marker_callback)
        rospy.Subscriber("/head/visualization_marker_head", Marker, self.head_marker_callback)
        self.chin_marker_info = [{'valid': False, 'pos': [0, 0, 0], 'rot':[0, 0, 0]} for i in range(18)]
        self.head_marker_info = [{'valid': False, 'pos': [0, 0, 0], 'rot':[0, 0, 0]} for i in range(18)]
        self.head_rot = [0, 0, 0]
        self.chin_marker_lock = threading.Lock()
        self.head_marker_lock = threading.Lock()
        self.__debug_print = _debug

    def set_debug_print(self, _debug):
        self.__debug_print = _debug

    def set_head_rot(self, rot):
        self.head_rot = rot

    def quart_to_rpy(self, w, x, y, z):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return [r, p, y]

    def rotate_mat(self, axis, radian):
        rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
        return rot_matrix

    def chin_marker_callback(self, msg):
        if msg.id > 18:
            print (msg.id, 'chin marker id > 18 ！！！')
            return
        rpy = self.quart_to_rpy(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        self.chin_marker_lock.acquire()
        self.chin_marker_info[msg.id]['pos'] = [-msg.pose.position.y, -msg.pose.position.x, msg.pose.position.z]
        self.chin_marker_info[msg.id]['rot'] = [-rpy[1] * 180.0 / math.pi, rpy[0] * 180.0 / math.pi, -rpy[2] * 180.0 / math.pi]

        cam_rot = self.rotate_mat([0, 1, 0], -(self.head_rot[1] - 10) * math.pi / 180.0)  # 绕y轴旋转
        pos_in_torso = np.dot(cam_rot, np.array([[-msg.pose.position.y], [-msg.pose.position.x], [msg.pose.position.z]])).tolist()
        self.chin_marker_info[msg.id]['pos'] = [pos_in_torso[0][0], pos_in_torso[1][0], pos_in_torso[2][0]]

        self.chin_marker_info[msg.id]['valid'] = True
        self.chin_marker_lock.release()

        if self.__debug_print == True:
            print ("chin marker:",  msg.id, self.chin_marker_info[msg.id])

    def head_marker_callback(self, msg):
        print("head_marker_callback")
        if msg.id > 18:
            print msg.id, 'head marker id > 18 ！！！'
            return
        rpy = self.quart_to_rpy(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)
        self.head_marker_lock.acquire()
        self.head_marker_info[msg.id]['pos'] = [msg.pose.position.z, -msg.pose.position.x, -msg.pose.position.y]
        self.head_marker_info[msg.id]['rot'] = [rpy[2] * 180.0 / math.pi, rpy[0] * 180.0 / math.pi, -rpy[1] * 180.0 / math.pi]

        cam_rot = self.rotate_mat([0, 1, 0], -(self.head_rot[1]) * math.pi / 180.0)  # 绕y轴旋转
        pos_in_torso = np.dot(cam_rot, np.array([[msg.pose.position.z], [-msg.pose.position.x], [-msg.pose.position.y]])).tolist()
        self.head_marker_info[msg.id]['pos'] = [pos_in_torso[0][0], pos_in_torso[1][0], pos_in_torso[2][0]]

        self.head_marker_info[msg.id]['valid'] = True
        self.head_marker_lock.release()

        if self.__debug_print == True:
            print "head marker:",  msg.id, self.head_marker_info[msg.id]

    def get_chin_marker_pose(self, mk_id):
        self.chin_marker_lock.acquire()
        temp = copy.deepcopy(self.chin_marker_info[mk_id])
        self.chin_marker_info[mk_id]['valid'] = False
        self.chin_marker_lock.release()
        return temp

    def get_head_marker_pose(self, mk_id):
        self.head_marker_lock.acquire()
        temp = copy.deepcopy(self.head_marker_info[mk_id])
        self.chin_marker_info[mk_id]['valid'] = False
        self.head_marker_lock.release()
        return temp


class SlamInfo(object):
    def __init__(self, _debug=False):
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.pose_callback)
        self.pose_in_world = {'valid': False, 'pos': [0, 0, 0], 'rot': [0, 0, 0]}
        self.pose_in_world_lock = threading.Lock()
        self.__debug_print = _debug

    def set_debug_print(self, _debug):
        self.__debug_print = _debug

    def toRPY(self, pose):
        return tf.transformations.euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def pose_callback(self, msg):
        p = msg.pose.pose.position
        roll, pitch, yaw = self.toRPY(msg.pose.pose)
        self.pose_in_world_lock.acquire()
        self.pose_in_world['pos'] = [p.x, p.y, p.z]
        self.pose_in_world['rot'] = [roll * (180.0 / math.pi), pitch * (180.0 / math.pi), yaw * (180.0 / math.pi)]
        self.pose_in_world['valid'] = True
        self.pose_in_world_lock.release()
        if self.__debug_print:
            print ('slam_pos: \npos:{} \nrot:{}\n'.format(self.pose_in_world["pos"],self.pose_in_world["rot"]))

    def get_pose_in_world(self):
        self.pose_in_world_lock.acquire()
        temp = copy.deepcopy(self.pose_in_world)
        self.pose_in_world['valid'] = False
        self.pose_in_world_lock.release()
        return temp


class MarkerLocation(MarkerInfo):
    def __init__(self, body_client, _debug=False):
        MarkerInfo.__init__(self, _debug)
        self.bodyhub = body_client
        self.marker_err_threshold = [0.02, 0.03, 3.0]

    def goto_rot(self, mk_id,  goal_rot):
        while not rospy.is_shutdown():
            self.get_chin_marker_pose(mk_id)
            time.sleep(0.4)
            marker_pose = self.get_chin_marker_pose(mk_id)
            if marker_pose['valid']:
                err = marker_pose['rot'][2] - goal_rot
                if (abs(err) < self.marker_err_threshold[2]):
                    break
                theta = err * 0.9
                self.bodyhub.walking_the_distance(0.0, 0.0, theta)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('%d marker no found!', mk_id)
                self.bodyhub.walking_n_steps([0.0, 0.0, 10.0], 2)
                self.bodyhub.wait_walking_done()

    def goto_pose(self, mk_id, goal_pose):
        while not rospy.is_shutdown():
            self.get_chin_marker_pose(mk_id)
            time.sleep(0.4)
            marker_pose = self.get_chin_marker_pose(mk_id)
            print("mark_p",marker_pose)
            if marker_pose['valid']:
                x_err = marker_pose['pos'][0] - goal_pose[0]
                y_err = marker_pose['pos'][1] - goal_pose[1]
                a_err = marker_pose['rot'][2] - goal_pose[2]
                if (abs(x_err) < self.marker_err_threshold[0]) and (abs(y_err) < self.marker_err_threshold[1]) and (abs(a_err) < self.marker_err_threshold[2]):
                    break
                x_len = x_err * 0.5
                y_len = y_err * 0.6
                a_len = a_err * 0.5
                self.bodyhub.walking_the_distance(x_len, y_len, a_len)
                self.bodyhub.wait_walking_done()
            else:
                rospy.logwarn('%d marker no found!', mk_id)
                self.bodyhub.walking_n_steps([-0.04, 0.0, 0.0], 1)
                self.bodyhub.wait_walking_done()


class SlamNavigation(SlamInfo):
    def __init__(self, body_client, _debug=False):
        SlamInfo.__init__(self, _debug)
        self.bodyhub = body_client
        self.slam_err_threshold = [0.15, 0.15, 10]
        self.slam_step_len_max = [0.06, 0.03, 10]
        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=1)

    def rot_mat(self, theta):
        theta = theta * (math.pi / 180.0)
        return np.array([[math.cos(theta), -math.sin(theta)],
                        [math.sin(theta), math.cos(theta)]], dtype=np.float32)

    def path_tracking(self, path_point, mode=1,jump_list=[]):
        """
        path_point:路径点列表
        jump_list:允许跳过的点在path_point中的索引,比如起始点和中间不太重要的点,避免机器人往回走
        """
        
        step_len = [0, 0, 0]
        rot_adjust = False
        path_marker, marker_num = 0, len(path_point)#0,2
        while not rospy.is_shutdown():
            rospy.wait_for_message('/requestGaitCommand', Bool, 10)
            self.get_pose_in_world()
            time.sleep(0.5)
            pose_in_w = self.get_pose_in_world()
            if pose_in_w['valid'] == False:
                print ('location not updated!',pose_in_w)
                time.sleep(0.5)
                continue
            current_pose = [pose_in_w['pos'][0], pose_in_w['pos'][1], pose_in_w['rot'][2]]#x,y,rot
            if len(jump_list) and path_marker in jump_list and path_marker!=marker_num-1:
                if path_point[path_marker][0]<current_pose[0]<path_point[path_marker+1][0]:
                    path_marker+=1
                    print("jump point",path_point[path_marker])
                    continue
            # if (mode == 1) and (path_marker == marker_num-1):
            #     time.sleep(0.2)

            for i in range(3):
                step_len[i] = path_point[path_marker][i] - current_pose[i]#第path_marker点到cp的距离
            v = np.dot(np.linalg.inv(self.rot_mat(current_pose[2])), np.array([step_len[0], step_len[1]])).tolist()
            if (mode == 1) and (path_marker == marker_num-1):
                w = step_len[2]#最后一次的转角
            else:
                w = (math.atan2(step_len[1], step_len[0]) * 180.0 / math.pi) - current_pose[2]
            w = (w-360.0) if w >= 180.0 else w
            w = (w+360.0) if w <= -180.0 else w
            step_len = [v[0], v[1], w]
            print ('cur_pose:{};target:{};\ntotal_step:{}'.format(current_pose,path_point,step_len))
            self.pose_update = False

            pos_err_scale, rot_err_scale = 1.0, 10.0
            if (mode == 1) and (path_marker == marker_num-1):
                pos_err_scale, rot_err_scale = 0.4, 0.2
            if (abs(step_len[0]) < (self.slam_err_threshold[0]*pos_err_scale)) \
            and (abs(step_len[1]) < (self.slam_err_threshold[1]*pos_err_scale)) \
                and (abs(step_len[2]) < (self.slam_err_threshold[2]*rot_err_scale)):
                path_marker = path_marker + 1
                print ('path marker', path_marker, '/', marker_num)
                if path_marker >= marker_num:
                    self.bodyhub.wait_walking_done()
                    break

            if abs(step_len[2]) > 30:
                rot_adjust = True#转角太大先调整转角
            else:
                rot_adjust = False

            for i in range(3):#步长限制
                step_len[i] = self.slam_step_len_max[i] if step_len[i] > self.slam_step_len_max[i] else step_len[i]
                step_len[i] = -self.slam_step_len_max[i] if step_len[i] < -self.slam_step_len_max[i] else step_len[i]
            print(step_len)
            if rot_adjust:
                self.__gait_cmd_pub.publish(data=[0.01, 0, step_len[2]])
            else:
                self.__gait_cmd_pub.publish(data=step_len)
        self.bodyhub.wait_walking_done()


class Navigation(MarkerLocation, SlamNavigation):
    def __init__(self, body_client, _debug=False):
        MarkerLocation.__init__(self, _debug)
        SlamNavigation.__init__(self, _debug)
        self.bodyhub = body_client
        self.slam_err_threshold = [0.15, 0.15, 10]
        self.slam_step_len_max = [0.06, 0.02, 10]
        self.__gait_cmd_pub = rospy.Publisher('/gaitCommand', Float64MultiArray, queue_size=1)

    def set_debug_print(self, _debug_mask):
        if (_debug_mask & (1 << 0)) != 0:
            MarkerLocation.set_debug_print(self, True)
        else:
            MarkerLocation.set_debug_print(self, False)
        if (_debug_mask & (1 << 1)) != 0:
            SlamNavigation.set_debug_print(self, True)
        else:
            SlamNavigation.set_debug_print(self, False)


if __name__ == '__main__':
    from motion import bodyhub_client as bodycli
    rospy.init_node('location_node', anonymous=True)
    time.sleep(0.2)
    bodyhub = bodycli.BodyhubClient(2)
    obj = Navigation(bodyhub,_debug=True)
    ar_tar_distance={"0":[0.094,-0.236,0.615],
    "1":[0.088,-0.214,0.575],
    "2":[0.15, -0.218, 0.769],
    "3":[0.04, -0.23, 0.62]}
    print(sys.argv)
    if len(sys.argv)==2 :
        if  sys.argv[1].isdigit() and int(sys.argv[1])<len(ar_tar_distance):
            bodyhub.walk()
            print("goto pose:",int(sys.argv[1]), ar_tar_distance[sys.argv[1]])
            # bodyhub.walking_n_steps([0.08, 0.0, 0.0], 1)
            obj.goto_pose(int(sys.argv[1]), ar_tar_distance[sys.argv[1]])
            # obj.goto_rot(int(sys.argv[1]),-2.6)
            bodyhub.reset()
        elif sys.argv[1]== "walk":
            bodyhub.walk()
        elif sys.argv[1]=="to_pile_adjust":
            # if bodyhub.get_status().data=="walking":
            #     print("reset ")
            #     bodyhub.reset()
            bodyhub.walk()
            bodyhub.walking_n_steps([0.08, 0.0, 0.0], 1)
            
            bodyhub.wait_walking_done()
            obj.goto_rot(2,-2.3)
            print("rot adjust done \n\n\n\n")
            obj.goto_pose(2, ar_tar_distance["2"])
            bodyhub.reset()
        elif sys.argv[1]=="ready":
            bodyhub.ready()
        else:
            bodyhub.reset()
    else:
        bodyhub.walk()
        # obj.path_tracking([[0.1, 0.0, 0.0], [0.2, 0.0, 0.0]])
        # obj.goto_pose(1, [0.14, -0.22, 0])
