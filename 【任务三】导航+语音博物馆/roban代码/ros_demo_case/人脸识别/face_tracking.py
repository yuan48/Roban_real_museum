#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import rospy
import rospkg
# 添加leju_lib_pkg包的路径到python查找目录中
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
from lejulib import finishsend
from head_controller import Head_Joint_Controller, limit_head_angle
from face_detecter import Face_detecter
from motion.motionControl import ResetBodyhub

actuator_pos = [0, 0] # 初始执行器输出为(0，0)

def tracking(face_pos, desired_pos=(320, 240)):
    """根据人脸位置计算头部舵机控制量
    @param    face_pos: 当前人脸位置(x,y),画面左上角为(0,0)
              desired_pos: 期望人脸落于画面中的位置(默认为中心点)
    @return   actuator_pos ：输出头部舵机执行器位置
    """
    global actuator_pos
    errx = -face_pos[0] + desired_pos[0]  # 获取期望位置(图像中心位置)与当前位置的差值
    erry = face_pos[1] - desired_pos[1]
    out_x = 0  # 控制器输出初始化为0
    out_y = 0

    # 执行阈值，只有超过该阈值头部关节才开始执行动作，避免变化过小时频繁调节位置
    if abs(errx) > controller.err_threshold[0]:
        out_x = controller.pid_x.run(errx)  # 计算x方向上pid控制器输出

    if abs(erry) > controller.err_threshold[1]:
        out_y = controller.pid_y.run(erry)  # 计算y方向上pid控制器输出

    # 由控制器输出累加得到执行器的控制量(即需要发布的舵机角度)
    actuator_pos[0] += out_x
    actuator_pos[1] += out_y

    # 对控制量进行限幅，避免过大调节量
    actuator_pos = limit_head_angle(actuator_pos)
    return actuator_pos

def rosShutdownHook():
    """节点关闭前触发回调函数"""
    detecter.join()  # 等待人脸识别线程退出
    finishsend()  # 节点结束时的统一处理, 如关闭声音、发送finish消息等
    ResetBodyhub()  # 重置机器人状态到preReady


if __name__ == '__main__':
    debug = True if len(sys.argv) > 1 else False
    # 初始化节点face_tracking, 匿名节点
    rospy.init_node("face_tracking", anonymous=True)
    rospy.on_shutdown(rosShutdownHook)  # 设置节点关闭时的回调函数

    # 实例化人脸识别线程，通过get_face_pos()即可方便地获取人脸位置
    detecter = Face_detecter(debug)
    detecter.start()

    # 实例化头部关节控制器，通过set_head_servo((x,y))即可设置头部关节位置
    controller = Head_Joint_Controller(debug)
    while not rospy.core.is_shutdown_requested():
        actuator_pos = tracking(detecter.get_face_pos()) # 获取人脸中心位置传入控制
        controller.set_head_servo(actuator_pos) # 获取控制量并发布
        time.sleep(0.01) # 控制频率约为100Hz

