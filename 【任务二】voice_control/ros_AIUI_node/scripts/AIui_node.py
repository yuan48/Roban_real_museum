#!/usr/bin/env python3
# coding=utf-8
import rospy
import rospkg
from std_msgs.msg import String, UInt8MultiArray
from ros_AIUI_node.srv import *
from Recorder import Recorder
import pyaiui
from pyAIUIConstant import AIUIConstant
import json5
import os
import sys
from Player import Player
import random
import json
from threading import Thread
import time
from ros_AIUI_node.srv import demoIsRunning, setDemoRunning, textToSpeak, textToSpeakMultipleOptions,direct_control
from EventListener import EventListener,EventData
from SocketClient import Socket_runner_client
WAKEUP_TOPIC = "/micarrays/wakeup"
HLW_SN_STREAM_AUDIO_TOPIC = "/audio/stream"
CHUNK = 2048

ROS_PACKAGE_NAME = "ros_AIUI_node"
PKG_DIR = rospkg.RosPack().get_path(ROS_PACKAGE_NAME)
cfg_file = os.path.join(PKG_DIR, "config/aiui.cfg")
sn_file_path = "/home/lemon/.lejuconfig/iflyos_config.json"

WAITING = 0
ABORT = -1
ALLOW = 1

def read_json5(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json5.loads(f.read())
    except Exception as e:
        rospy.logerr("{},{}".format(e,__file__))
    return None

def load_local_skills(skill_config_path):
    stime= time.time()
    original_skills = read_json5(skill_config_path)
    skills_dict={}
    for skill in original_skills:
        for key_word in original_skills[skill]["key_words"]:
            if key_word not in skills_dict:
                skills_dict[key_word] = original_skills[skill]
    # print(skills_dict,time.time()-stime)
    return skills_dict
    
class AIUINode():
    def __init__(self, event_listener, debug=False, wakeup_mute = False):
        self.local_custom_skills_dict = event_listener.local_custom_skills_dict
        self.player = Player()
        self.agent_status = None
        self.socket_client = Socket_runner_client()
        event_listener.set_agent(self)
        event_listener.set_player(self.player)
        self.event_listener = event_listener
        self.agent = pyaiui.IAIUIAgent.createAgent(json5.dumps(read_json5(cfg_file), quote_keys=True), event_listener)
        self.__recorder_ = None
        self.is_recording = False
        self.recorder_states = WAITING# -1:中断状态;0:等待状态;1:允许录音状态
        self.wakeup_mute = wakeup_mute
        self.is_hlw = False
        self.set_is_hlw()
        self.aiui_running_demo_status_srv = rospy.Service("aiui/demo_running_status", demoIsRunning, self.socket_client.get_execution_status)
        self.nlp_pub = rospy.Publisher("/aiui/nlp", String, queue_size = 2)
        self.text_to_speak_service = rospy.Service("/aiui/text_to_speak", textToSpeak, self.text_to_speak_callback)
        self.direct_control_service = rospy.Service("/aiui/direct_control", direct_control, self.direct_control_callback)
        self.text_to_speak_multiple_options = rospy.Service("/aiui/text_to_speak_multiple_options", textToSpeakMultipleOptions, self.text_to_speak_multiple_options_callback)

    def text_to_speak_multiple_options_callback(self, req):
        self.cmd_tts(req.text, req.vcn, req.speed, req.pitch, req.volume)
        return True
    
    def direct_control_callback(self, req):
        text = req.text
        self.event_listener.EventNLP(EventData({"text":text}))
        return True
    
    def text_to_speak_callback(self, req):
        text = req.text
        self.cmd_tts(text)
        return True

    def set_is_hlw(self):
        if rospy.get_param("/is_hlw_mic", False) == True: self.is_hlw = True

    def cmd_tts(self, text, vcn="qige", speed=50, pitch=50, volume=50):
        text = pyaiui.Buffer.create(bytes(text, encoding="utf-8"))
        TTSMsg = pyaiui.IAIUIMessage.create(pyaiui.AIUIConstant.CMD_TTS, 1, 0, "text_encoding=utf-8,vcn={},ent=x,speed={},pitch={},volume={}".format(vcn, speed, pitch, volume), text)
        self.agent.sendMessage(TTSMsg)
        TTSMsg.destroy()

    def cmd_tts_stop(self):
        text = pyaiui.Buffer.create(bytes("", encoding="utf-8"))
        TTSMsg = pyaiui.IAIUIMessage.create(pyaiui.AIUIConstant.CMD_TTS, 2, 0, "text_encoding=utf-8,vcn=qige,ent=x", text)
        self.agent.sendMessage(TTSMsg)
        TTSMsg.destroy()

    def wakeup_callback(self, msg):
        self.recorder_states = ABORT# 中断上一次录音等待
        self.getState()
        if self.is_recording:
            self.reset_wakeup_agent()
            self.is_recording = False
        if self.agent_status != AIUIConstant.STATE_WORKING:
            self.wakeup_agent()
        if self.player.is_playing:
            self.player.stop()
        wakeup_reply = self.local_custom_skills_dict["wakeup"]["reply"]
        choose_one_reply = random.choice(wakeup_reply)
        if not self.wakeup_mute:
            self.cmd_tts(choose_one_reply)
            self.recorder_states = WAITING # 进入正常等待状态
        else:
            self.recorder_states = ALLOW # 直接允许录音
        Thread(target=self.__wait_play_to_start_record).start()

    def __wait_play_to_start_record(self):
        while self.player.is_playing == False:# 正在等待开始播放
            rospy.sleep(0.02)
            if self.recorder_states == ABORT:# 如果重新开始唤醒流程,中止
                return
            elif self.recorder_states == ALLOW:# 如果直接允许录音
                break
        while self.player.is_playing == True:# 正在等待播放结束
            rospy.sleep(0.02)
            if self.recorder_states == ABORT:# 如果重新开始唤醒流程,中止
                return
            elif self.recorder_states == ALLOW:
                break
        self.recorder_states = ALLOW # 直接赋值允许录音
        self.start_record()

    def start_record(self):
        self.is_recording = True
        if self.is_hlw == False:
            self.__start_record()

    @property
    def __recorder(self):
        if self.__recorder_ == None:
            self.__recorder_ = Recorder(CHUNK)
        return self.__recorder_

    def __start_record(self):
        for data in self.__recorder.read():
            if self.is_recording:
                self.audio_stream_callback(data)

    def audio_stream_callback(self, data):
        if self.is_recording:
            if self.is_hlw:
                audio_buffer = pyaiui.Buffer.create(data.data)
                self.record_stream_call_back(data.data)
            else:
                audio_buffer = pyaiui.Buffer.create(data)
                self.record_stream_call_back(data)
            self.cmd_write_audio(audio_buffer)
            
    def record_stream_call_back(self,buffer):
        pass
    def wakeup_agent(self):
        wakeup_msg = pyaiui.IAIUIMessage.create(AIUIConstant.CMD_WAKEUP)
        self.agent.sendMessage(wakeup_msg)
        wakeup_msg.destroy()

    def reset_wakeup_agent(self):
        reset_wakeup_msg = pyaiui.IAIUIMessage.create(AIUIConstant.CMD_RESET_WAKEUP)
        self.agent.sendMessage(reset_wakeup_msg)
        reset_wakeup_msg.destroy()

    def getState(self):
        get_state_msg = pyaiui.IAIUIMessage.create(AIUIConstant.CMD_GET_STATE)
        self.agent.sendMessage(get_state_msg)
        get_state_msg.destroy()

    def cmd_write_audio(self, audio_buffer):
        writeMsg = pyaiui.IAIUIMessage.create(AIUIConstant.CMD_WRITE, 0, 0, "data_type=audio", audio_buffer)
        self.agent.sendMessage(writeMsg)
        writeMsg.destroy()
        

    def cmd_write_text(self, text):
        text = pyaiui.Buffer.create(bytes(text, encoding="utf-8"))
        writeMsg = pyaiui.IAIUIMessage.create(pyaiui.AIUIConstant.CMD_WRITE, 0, 0, "data_type=text", text)
        self.agent.sendMessage(writeMsg)
        writeMsg.destroy()

    def stop_write(self):
        writeMsgStop = pyaiui.IAIUIMessage.create(AIUIConstant.CMD_STOP_WRITE, params="data_type=audio")
        self.agent.sendMessage(writeMsgStop)
        writeMsgStop.destroy()

    def start(self,spin=True):
        self.cmd_tts_stop()
        if os.path.exists(self.event_listener.tts_pcm_file) == True:
            os.remove(self.event_listener.tts_pcm_file)
        rospy.Subscriber(WAKEUP_TOPIC, String, self.wakeup_callback)
        if self.is_hlw:
            self.hlw_audio_in = rospy.Subscriber(HLW_SN_STREAM_AUDIO_TOPIC, UInt8MultiArray, self.audio_stream_callback)
        if spin:#是否阻塞运行
            rospy.spin()

    def rosShutdownHook(self):
        self.player.stop()
        self.socket_client.close()

def load_sn():
    data = read_json5(sn_file_path)
    return str(data["device_id"])
pyaiui.AIUISetting.setSystemInfo("sn", load_sn())
pyaiui.AIUISetting.setMscDir("/tmp/")

