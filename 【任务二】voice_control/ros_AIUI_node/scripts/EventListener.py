#!/usr/bin/env python3
# coding=utf-8
import rospy
import rospkg
from ros_AIUI_node.srv import *
import pyaiui
from pyAIUIConstant import AIUIConstant
import json5
import os
import sys
import random
sys.path.append(rospkg.RosPack().get_path('leju_lib_pkg'))
from motion.motionControl import ResetBodyhub
from execute_funcs import Exec_func
import time

#### EVENT_RESULT ####
IAT = "iat"     # 听写结果
NLP = "nlp"     # 语义结果
TPP = "tpp"     # 后处理服务结果
TTS = "tts"     # 云端tts结果
ITRANS = "itrans"   # 翻译结果

NO_SKILL = 0
IS_LOCAL_SKILL = 1
IS_BOTH_AIUI_LOCAL_SKILL = 2
LAST_TTS_FLAG_KEY = "lrst"
IS_LAST_TTS_AUDIO = "1"
TTS_SAVE_PCM = "/tmp/aiui_voice.pcm"
EMPTY_PACKET ="AIUI DATA NULL"
class EventData(object):
    def __init__(self, data):
        self.data = data

    def get_data(self):
        return self.data

class EventListener(pyaiui.AIUIEventListener):
    def __init__(self, skills_dict, debug=False):
        self.Event_Listener_dict = {
            AIUIConstant.EVENT_STATE: self.getEventState,
            AIUIConstant.EVENT_WAKEUP: self.eventWakeup,
            AIUIConstant.EVENT_SLEEP: self.eventSleep,
            AIUIConstant.EVENT_VAD: self.eventVAD,
            AIUIConstant.EVENT_RESULT: self.eventResult,
            AIUIConstant.EVENT_ERROR: self.eventError
        }
        self.tts_pcm_file = TTS_SAVE_PCM
        self.local_custom_skills_dict = skills_dict
        self.debug = debug
        self.agent = None
        self.socket_client = None
        self.player = None
        self.param = None
        self.content = None
        self.is_aiui_local_skill = NO_SKILL
        self.event_func_executer = Exec_func()
        self.event_func_executer.start()
        self.is_tts_reply = False

    def set_player(self,player):
        self.player = player

    def set_agent(self,agent):
        self.agent = agent
        self.socket_client = self.agent.socket_client

    def onEvent(self, event):
        """agent事件回调解析"""
        eventType = event.getEventType()
        if eventType in self.Event_Listener_dict:
            self.Event_Listener_dict[eventType](event)
        else:
            print("接收到未处理的 EventType: {}".format(eventType))

    def eventError(self, event):
        rospy.logerr("Error code: {}, info: {}".format(event.getArg1(), event.getInfo()))

    def getEventState(self, event):
        arg1 = event.getArg1()
        self.agent.agent_status = arg1
        self.rosInfo("agent status: {}".format(arg1))

    def eventWakeup(self, event):
        print("Event Wakeup")

    def eventSleep(self, event):
        arg1 = event.getArg1()
        if arg1 == AIUIConstant.TYPE_AUTO:
            print("自动休眠")
        if arg1 == AIUIConstant.TYPE_COMPEL:
            print("强制休眠")
            self.agent.wakeup_agent()

    def eventVAD(self, event):
        arg1 = event.getArg1()
        if AIUIConstant.VAD_BOS == arg1:
            debugMsg = "VAD:检测到前端点"
            self.rosInfo(debugMsg)
        elif AIUIConstant.VAD_EOS == arg1:
            debugMsg = "VAD:检测到后端点"
            self.agent.is_recording = False
            self.rosInfo(debugMsg)

    def eventResult(self, event):
        info = json5.loads(event.getInfo().encode("utf-8"))
        datas = info["data"]
        data0 = datas[0]
        self.param = data0["params"]
        contents = data0["content"]
        content0 = contents[0]
        self.content = content0

        sub = self.param["sub"]

        dataBundle = event.getData()
        cnt_id = content0["cnt_id"]
        if NLP == sub:
            resultStr = dataBundle.getBinaryAsStr(cnt_id)
            resultJson = json5.loads(resultStr)
            if "rc" in resultJson["intent"]:
                nlp_result_text = resultJson["intent"]["text"]
                self.EventNLP(EventData({"text":nlp_result_text}))
                resultStr = dataBundle.getBinaryAsStr(cnt_id)
                if "answer" in resultJson["intent"]:    # 命中 aiui 技能
                    aiui_reply = resultJson["intent"]["answer"]["text"]
                    if self.is_aiui_local_skill == IS_LOCAL_SKILL: # aiui技能和本地技能同时命中
                        self.is_aiui_local_skill = IS_BOTH_AIUI_LOCAL_SKILL 
                    pass    # 可以用消息发布命中技能的文字返回
                else:
                    aiui_reply = resultStr
                self.rosInfo(aiui_reply)
        elif IAT == sub:
            resultStr = dataBundle.getBinaryAsStr(cnt_id)
            resultJson = json5.loads(resultStr)
            if self.debug: parse_iat(resultJson)
        elif TTS == sub:
            if "error" in content0 and content0["error"] == EMPTY_PACKET: return
            buffer = dataBundle.getBinary(cnt_id)
            dts = content0["dts"] # 音频块位置状态信息
            self.EventTTS(EventData({"buffer":buffer,"dts":dts}))

    def EventNLP(self, event):
        """语义结果回调
        @event  EventData
        """
        nlp_result_text = event.data["text"]
        debugMsg = "nlp: {0}".format(nlp_result_text)
        self.agent.nlp_pub.publish(nlp_result_text)
        self.rosInfo(debugMsg)
        if nlp_result_text in self.local_custom_skills_dict.keys():
            self.rosInfo("match skill {}".format(nlp_result_text))
            self.is_aiui_local_skill = IS_LOCAL_SKILL
            self.agent.cmd_tts_stop()
            local_skill_content = self.local_custom_skills_dict[nlp_result_text]
            local_FAQ_reply = local_skill_content["reply"]
            choose_reply_text = random.choice(local_FAQ_reply)
            debugMsg = choose_reply_text
            self.rosInfo(debugMsg)
            self.is_tts_reply = False
            self.agent.cmd_tts(choose_reply_text)
            if "cmd_type" in local_skill_content:
                if local_skill_content["cmd_type"] == "stop_node":
                    self.socket_client.send(self.packed_socket_msg(local_skill_content["cmd_type"]))
                elif local_skill_content["cmd_type"] == "run_node":
                    if self.socket_client.get_execution_status():# 执行指令之前查询是否已经有一个案例在运行
                        self.socket_client.send(self.packed_socket_msg("stop_node"))
                    while self.socket_client.get_execution_status() or self.socket_client.get_aborting_status():
                        time.sleep(0.01)
                            
                    # ResetBodyhub()
                    path_dict = {"path": local_skill_content["path"]}
                    self.socket_client.send(self.packed_socket_msg(local_skill_content["cmd_type"], path_dict))
                    if "interpret_reply" in local_skill_content.keys():
                        while not self.is_tts_reply:# 等待动作前的语音合成完毕
                            time.sleep(0.1)
                        interpret_reply = local_skill_content["interpret_reply"]
                        self.player.wait_for_stop()
                        self.agent.cmd_tts(random.choice(interpret_reply))# 动作时的语音合成
                elif local_skill_content["cmd_type"] == "execute_func":
                    self.event_func_executer.exec({"func":local_skill_content["execute_func"],"argv":nlp_result_text})
                else:
                    rospy.logwarn("Aiui skill action notdefine!")


    def EventTTS(self, event):
        """TTS结果
        @event  EventData
        """
        buffer = event.data["buffer"]
        if self.player.is_playing:
            self.player.stop()
            while self.player.is_abort == False:
                rospy.sleep(0.1)
        if (event.data["dts"] == 0 or event.data["dts"] == 3)and os.path.exists(self.tts_pcm_file):# 接收到语音头部或短语音,删除旧文件
            os.remove(self.tts_pcm_file)
        with open(self.tts_pcm_file, "ab+") as tts:
            tts.write(buffer)
        if not rospy.core.is_shutdown_requested() and self.is_tts_download_complete() == True:
            if self.is_aiui_local_skill == IS_BOTH_AIUI_LOCAL_SKILL and self.param["cmd"] == "iat-kc-tts": 
                # 当同时命中本地和aiui技能,并且当前音频为aiui技能tts,则不播放
                pass
            else:
                self.player.play(self.tts_pcm_file)
                self.is_tts_reply = True
            self.is_aiui_local_skill = NO_SKILL

    def packed_socket_msg(self, cmd_type, params_dict={}):
        msg = {
            "cmd": cmd_type
        }
        msg.update(params_dict)
        return msg

    def is_tts_download_complete(self):
        if "dts" in self.content and self.content["dts"] in (2,3):
            return True
        return False

    def rosInfo(self, msg):
        if self.debug: rospy.loginfo(msg)

def parse_iat(resultJson):
    is_last = resultJson["text"]["ls"]
    joinText = ""
    if is_last == False:
        for ws in resultJson["text"]["ws"]:
            joinText += ws["cw"][0]["w"]
        print(joinText)
