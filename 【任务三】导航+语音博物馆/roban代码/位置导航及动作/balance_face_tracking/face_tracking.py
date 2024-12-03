#!/usr/bin/python
import sys
import os
import cv2
import signal
import Queue
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import array
import time
import threading
from bodyhub.srv import *  # for SrvState.srv
from bodyhub.msg import JointControlPoint
from lejulib import *
import rospkg
SERVO = client_action.SERVO
QUEUE_IMG = Queue.Queue(maxsize=2)
bridge = CvBridge()
faceadd = rospkg.RosPack().get_path("ros_actions_node") + "/scripts/tracking/haarcascade_frontalface_alt2.xml"
face_detector = cv2.CascadeClassifier(faceadd)

# Horizontal and vertical limit value
H_limit=80
V_limit=40
DEBUG=False
tracking_threads=[]
class FaceConfig:
    def __init__(self):
        self.running = True
        self.size = 0.5
        self.face = 0, 0, 0, 0
        self.face_roi = 0, 0, 0, 0
        self.face_template = None
        self.found_face = False
        self.template_matching_running = False
        self.template_matching_start_time = 0
        self.template_matching_current_time = 0
        self.center_x = 160
        self.center_y = 120
        self.pan = 0
        self.tlt = 0
        self.error_pan = 0
        self.error_tlt = 0
        self.controlID=2
        self.HeadJointPub = rospy.Publisher('MediumSize/BodyHub/HeadPosition', JointControlPoint, queue_size=100)
        self.update_ctlid()
        
    def update_ctlid(self):#get the newest Main_controlID
        try:
            rospy.wait_for_service('MediumSize/BodyHub/GetMasterID',1)
        except:
            print ('error: wait_for_service GetMasterID!',sys.exc_info())
            return 0
        client = rospy.ServiceProxy('MediumSize/BodyHub/GetMasterID', SrvTLSstring)
        response = client('get')
        self.controlID= response.data
        return 1
Face = FaceConfig()


def doubleRectSize(input_rect, keep_inside):
    xi, yi, wi, hi = input_rect
    xk, yk, wk, hk = keep_inside
    wo = wi * 2
    ho = hi * 2
    xo = xi - wi // 2
    yo = yi - hi // 2
    if wo > wk:
        wo = wk
    if ho > hk:
        ho = hk
    if xo < xk:
        xo = xk
    if yo < yk:
        yo = yk
    if xo + wo > wk:
        xo = wk - wo
    if yo + ho > hk:
        yo = hk - ho
    return xo, yo, wo, ho


def face_size(face):
    x, y, w, h = face
    return w * h


def face_filter(face_list):
    face_size_list = map(face_size, face_list)
    target_index = face_size_list.index(max(face_size_list))
    return face_list[target_index]


def detectFaceAllSizes(frame):
    """Detect using cascades over whole image

    :param frame:
    :return:
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)    
    # gray = cv2.equalizeHist(gray)
    face_locations = face_detector.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=3, minSize=(frame.shape[1] / 12, frame.shape[0] / 12),
        maxSize=(2 * frame.shape[1] / 3, 2 * frame.shape[1] / 3))
    if len(face_locations) <= 0:
        Face.face = 0, 0, 0, 0
        return
    Face.found_face = True
    Face.face = face_filter(face_locations)
    Face.face_template = frame[Face.face[1]:(Face.face[1] + Face.face[3]),
                         Face.face[0]:(Face.face[0] + Face.face[2])].copy()
    Face.face_roi = doubleRectSize(Face.face, (0, 0, frame.shape[1], frame.shape[0]))


def detectFaceAroundRoi(frame):
    """Detect using cascades only in ROI

    :param frame:
    :return:
    """
    face_tem = frame[Face.face_roi[1]:Face.face_roi[1] + Face.face_roi[3],
               Face.face_roi[0]:Face.face_roi[0] + Face.face_roi[2]]
    gray = cv2.cvtColor(face_tem, cv2.COLOR_BGR2GRAY)
    face_locations = face_detector.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=3, minSize=(frame.shape[1] / 12, frame.shape[0] / 12),
        maxSize=(2 * frame.shape[1] / 3, 2 * frame.shape[1] / 3))
    if len(face_locations) <= 0:
        Face.template_matching_running = True
        if Face.template_matching_start_time == 0:
            Face.template_matching_start_time = cv2.getTickCount()
        return
    Face.template_matching_running = False
    Face.template_matching_current_time = 0
    Face.template_matching_start_time = 0

    Face.face = face_filter(face_locations)
    Face.face[0] += Face.face_roi[0]
    Face.face[1] += Face.face_roi[1]
    Face.face_template = frame[Face.face[1]:Face.face[1] + Face.face[3],
                         Face.face[0]:Face.face[0] + Face.face[2]].copy()
    Face.face_roi = doubleRectSize(Face.face, (0, 0, frame.shape[1], frame.shape[0]))


def detectFacesTemplateMatching(frame):
    """Detect using template matching

    :param frame:
    :return:
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    Face.template_matching_current_time = cv2.getTickCount()
    duration = (Face.template_matching_current_time - Face.template_matching_start_time) / cv2.getTickFrequency()
    if duration > 1:
        Face.found_face = False
        Face.template_matching_running = False
        Face.template_matching_start_time = 0
        Face.template_matching_current_time = 0
    target = gray[Face.face_roi[1]:Face.face_roi[1] + Face.face_roi[3],
             Face.face_roi[0]:Face.face_roi[0] + Face.face_roi[2]]
    Face.face_template = cv2.cvtColor(Face.face_template, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(target, Face.face_template, cv2.TM_CCOEFF)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
    max_x = max_loc[0] + Face.face_roi[0]
    max_y = max_loc[1] + Face.face_roi[1]

    Face.face = max_x, max_y, Face.face[2], Face.face[3]
    Face.face_template = frame[Face.face[1]:Face.face[1] + Face.face[3],
                         Face.face[0]:Face.face[0] + Face.face[2]].copy()
    Face.face_roi = doubleRectSize(Face.face, (0, 0, frame.shape[1], frame.shape[0]))


def show_face(face):
    face_cx = (face[0] + face[2] / 2) / Face.size
    face_cy = (face[1] + face[3] / 2) / Face.size
    client_label.set_camera_label((255, 0, 0), (face_cx, face_cy), face[2]/Face.size, face[3]/Face.size)


def detectFace():
    rate = rospy.Rate(100)
    while not Face.found_face and Face.running:
        time.sleep(0.01)
        if not QUEUE_IMG.empty():
            frame = QUEUE_IMG.get()
        else:
            continue
        detectFaceAllSizes(frame)
        show_face(Face.face)

        while Face.found_face and Face.running:
            rate.sleep()
            if not Face.face_template.any():
                continue
            if not QUEUE_IMG.empty():
                frame = QUEUE_IMG.get()
            else:
                continue
            detectFaceAroundRoi(frame)
            if Face.template_matching_running:
                detectFacesTemplateMatching(frame)
            show_face(Face.face)

def async_do_job(func):
    async_thread = threading.Thread(target=func)
    async_thread.setDaemon(True)
    async_thread.start()
    tracking_threads.append(async_thread)


def set_head_servo(angles):
    """set head servos angle

    :param angles:[pan, tilt]
    :return:
    """
    # Face.update_ctlid()
    # angles = array.array("d", angles)
    if not rospy.core.is_shutdown_requested():
        print("Head servo angle ot trach face:[{}]".format(angles))
        Face.HeadJointPub.publish(positions=angles, mainControlID=Face.controlID)
    time.sleep(0.01)


def terminate(data):
    """Terminate all threads
    """
    rospy.loginfo(data.data)
    Face.running = False
    rospy.signal_shutdown("kill")


def thread_face_center():
    while Face.running:
        time.sleep(0.01)
        face_x = Face.face[0] + Face.face[2] / 2
        face_y = Face.face[1] + Face.face[3] / 2
        if face_x == 0 and face_y == 0:
            face_x = Face.center_x
            face_y = Face.center_y
        Face.error_pan = Face.center_x - face_x
        Face.error_tlt = Face.center_y - face_y
        rospy.logdebug("Face.error_pan,Face.error_tlt %f,%f", Face.error_pan, Face.error_tlt)


def thread_set_servos():
    set_head_servo([Face.pan, Face.tlt])
    step = 0.01
    Face.update_ctlid()
    while Face.running:
        if abs(Face.error_pan) > 15 or abs(Face.error_tlt) > 15:
            if abs(Face.error_pan) > 15:
                Face.pan += step * Face.error_pan
            if abs(Face.error_tlt) > 15:
                Face.tlt += step * Face.error_tlt
            
            if Face.pan > H_limit:
                Face.pan = H_limit
            elif Face.pan < -H_limit:
                Face.pan = -H_limit
            if Face.tlt > V_limit:
                Face.tlt = V_limit
            elif Face.tlt < -V_limit:
                Face.tlt = -V_limit
            set_head_servo([Face.pan, -Face.tlt])
        else:
            time.sleep(0.01)
    # set_head_servo([0,0])



def image_callback(msg):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as err:
        print(err)
    else:
        cv2_img = cv2.resize(cv2_img, (0, 0), fx=Face.size, fy=Face.size)
        cv2.rectangle(cv2_img, (Face.face[0], Face.face[1]), (Face.face[0]+Face.face[2],Face.face[1]+Face.face[3]), (0, 0, 255), 2)
        if DEBUG:
            cv2.imshow("result",cv2_img)
            cv2.waitKey(1)
        if QUEUE_IMG.full():
            QUEUE_IMG.get()
        QUEUE_IMG.put(cv2_img, block=True)


def face_tracking_start(debug=False):
    global DEBUG
    DEBUG=debug
    try:
        client_controller.send_label_on(True)
        client_controller.send_video_status(True, "/camera/label/image_raw",width=640,height=480)
        image_topic = "/camera/color/image_raw"
        rospy.Subscriber(image_topic, Image, image_callback)
        async_do_job(detectFace)
        async_do_job(thread_face_center)
        async_do_job(thread_set_servos)
        
    except Exception as err:
        serror(err)
        
def face_tracking_stop():
    Face.running = False
    for th in tracking_threads:
        th.join()
    client_controller.send_label_on(False)
    client_controller.send_video_status(False, "/camera/label/image_raw",width=640,height=480)

if __name__ == '__main__':
    node_initial(name = "face_tracking")
    rospy.on_shutdown(face_tracking_stop)
    face_tracking_start(debug=True)
    rospy.Subscriber('terminate_current_process', String, terminate)
    rospy.spin()
