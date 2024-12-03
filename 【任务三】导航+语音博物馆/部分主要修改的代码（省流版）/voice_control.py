#!/usr/bin/env python3
# coding=utf-8
import rospy
import rospkg
import os
from std_msgs.msg import String
from time import sleep
import sys
import random
import subprocess
from motion.motionControl import ResetBodyhub,SetBodyhubTo_setStatus,GetBodyhubStatus
# from ros_museum_docent.main import MuseumDocent
sys.path.append(rospkg.RosPack().get_path('ros_AIUI_node')+"/scripts")
from EventListener import EventListener as aiui_eventlistener
from AIui_node import AIUINode, load_local_skills
skills_config_path = os.path.join(os.path.split(os.path.realpath(__file__))[0],"VoiceControlConfig.json")
local_custom_skills_dict = load_local_skills(skills_config_path)

class EventListener(aiui_eventlistener):
    def __init__(self, skills_dict, debug=False):
        super(EventListener,self).__init__(skills_dict, debug)
        self.ex_status = ""
        self.question_num = 0
        self.question_list = ["名字","作者","风格","简介"]
        self.museum_ex=["1","展品二","展品三","展品四","展品五","展品六"]
        self.next="下一个"
        self.museum_num=0
    # def detect_callback(self,data):
    #     rospy.loginfo("Get detect data is %s!",data.data)

    def EventNLP(self, event):
        """重写父类语义结果事件回调函数"""
        print("now budyhub status is:",GetBodyhubStatus())
        ResetBodyhub()
        print(event.data)
        nlp_result_text = event.data["text"] # 获取语义结果
        # pyindex={"小金人":xjr_path,"rdfw":rdfw_path,"三星堆":sxd_path}
        xjr_path =  "~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/exhit_1.py"
        rdfw_path = "~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/exhit_2.py"
        sxd_path = "~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/exhit_3.py"
        pyindex={"带我去展位一":xjr_path,"带我去展位二":rdfw_path,"带我去展位三":sxd_path,"去小金人":xjr_path,"带我去展位2":rdfw_path,"带我去展位3":sxd_path}

        # nlp_result_text ="跳舞"
        if (nlp_result_text in pyindex) or nlp_result_text in self.local_custom_skills_dict.keys() or (nlp_result_text in self.question_list)or (nlp_result_text in self.next):# 查找匹配的命令
            if nlp_result_text in self.local_custom_skills_dict.keys():
                local_skill_content = self.local_custom_skills_dict[nlp_result_text]

            # choose_reply_text = random.choice(local_skill_content["interpret_reply"]) # 随机选择一个回答
            print("in local")

            if (nlp_result_text == "这是什么") or (nlp_result_text in self.museum_ex) or (nlp_result_text in self.question_list) or (nlp_result_text in self.next):
                print("in exhibition")
                if (nlp_result_text in self.museum_ex):
                    self.ex_status = nlp_result_text
                    local_skill_content = self.local_custom_skills_dict[nlp_result_text]
                    choose_reply_text = local_skill_content["reply"][0] # 0
                    if "cmd_type" in local_skill_content: # 如果时cmd_type类型的命令
                        if local_skill_content["cmd_type"] == "stop_node":
                            socket_msg = {"cmd":"stop_node"}
                        elif local_skill_content["cmd_type"] == "run_node":
                            socket_msg = {"cmd":"run_node","path": local_skill_content["path"]}
                        self.socket_client.send(socket_msg) # 发送socket消息执行案例
                elif (nlp_result_text == self.next):
                    self.museum_num = self.museum_ex.index(self.ex_status)
                    self.ex_status = self.museum_ex[(self.museum_num+1)%6]
                    local_skill_content = self.local_custom_skills_dict[self.ex_status]
                    choose_reply_text = local_skill_content["reply"][0] # 0
                    if "cmd_type" in local_skill_content: # 如果时cmd_type类型的命令
                        if local_skill_content["cmd_type"] == "stop_node":
                            socket_msg = {"cmd":"stop_node"}
                        elif local_skill_content["cmd_type"] == "run_node":
                            socket_msg = {"cmd":"run_node","path": local_skill_content["path"]}
                        self.socket_client.send(socket_msg) # 发送socket消息执行案例                 

                elif (nlp_result_text in self.question_list):
                    self.question_num = self.question_list.index(nlp_result_text)
                    choose_reply_text = self.local_custom_skills_dict[self.ex_status]["reply"][self.question_num] # choose question
                elif (nlp_result_text == "这是什么"):
                    print("what's this?")
                    # rospy.Subscriber("/detected_exhibition",String,self.detect_callback)
                    ex_detected = rospy.wait_for_message("/detected_exhibition",String)
                    self.ex_status = ex_detected.data   #get detected exhibition
                    rospy.loginfo("Received detected result! It is %s",self.ex_status)
                    local_skill_content = self.local_custom_skills_dict[self.ex_status]
                    choose_reply_text = local_skill_content["reply"][0]

                    if "cmd_type" in local_skill_content: # 如果时cmd_type类型的命令
                        if local_skill_content["cmd_type"] == "stop_node":
                            socket_msg = {"cmd":"stop_node"}
                        elif local_skill_content["cmd_type"] == "run_node":
                            socket_msg = {"cmd":"run_node","path": local_skill_content["path"]}
                        self.socket_client.send(socket_msg) # 发送socket消息执行案例
                self.agent.cmd_tts(choose_reply_text) # 调用tts将回答的文字转语音播放
                # ResetBodyhub()
            # pyindex={"小金人":xjr_path,"rdfw":rdfw_path,"三星堆":sxd_path}
            # xjr_path =  "~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/exhit_1.py"
            # rdfw_path = "~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/exhit_2.py"
            # sxd_path = "~/robot_ros_application/catkin_ws/src/ros_actions_node/scripts/exhit_3.py"
            elif (nlp_result_text in pyindex):
                print("heading target:",nlp_result_text)
                choose_reply_text = random.choice(local_skill_content["reply"]) # 随机选择一个回答
                if "cmd_type" in local_skill_content: # 如果时cmd_type类型的命令
                    if local_skill_content["cmd_type"] == "stop_node":
                        socket_msg = {"cmd":"stop_node"}
                    elif local_skill_content["cmd_type"] == "run_node":
                        runpath = pyindex[nlp_result_text]
                        print("we are no executing python path:",runpath)
                        p1 = subprocess.Popen("python {}".format(runpath), shell=True)
                        # print("starting cmd")
                        # socket_msg = {"cmd":"run_node","path": local_skill_content["path"]}
                    print("小金人 to socket")
            else:
                print("normal")
                # SetBodyhubTo_setStatus(2)
                choose_reply_text = random.choice(local_skill_content["reply"]) # 随机选择一个回答
                if "cmd_type" in local_skill_content: # 如果时cmd_type类型的命令
                    if local_skill_content["cmd_type"] == "stop_node":
                        socket_msg = {"cmd":"stop_node"}
                    elif local_skill_content["cmd_type"] == "run_node":
                        print("starting cmd")
                        socket_msg = {"cmd":"run_node","path": local_skill_content["path"]}
                    print("sending to socket")
                    self.socket_client.send(socket_msg) # 发送socket消息执行案例
                self.agent.cmd_tts(choose_reply_text) # 调用tts将回答的文字转语音播放
                # ResetBodyhub()
        else:
            local_skill_content = self.local_custom_skills_dict["不知道"]
            choose_reply_text = random.choice(local_skill_content["reply"]) # 随机选择一个回答
            self.agent.cmd_tts(choose_reply_text) # 调用tts将回答的文字转语音播放

    def EventTTS(self, event):
        """重写父类TTS(文字转语音)事件回调函数"""
        buffer = event.data["buffer"] # 获取一段语音数据
        if self.player.is_playing: # 如果正在播放语音,则停止播放
            self.player.stop()
            while self.player.is_abort == False:
                rospy.sleep(0.1)
        with open(self.tts_pcm_file, "ab+") as tts: # 将语音数据写入文件中
            tts.write(buffer)
        if self.is_tts_download_complete() == True: # 判断是否接收完成语音合成的结果
            self.player.play(self.tts_pcm_file) # 接收完则开始播放

if __name__ == "__main__":
    rospy.init_node("ros_aiui_node", anonymous=False) # 实例化aiui节点
    debug = True if len(sys.argv) > 1 and sys.argv[1] == "debug" else False
    eventlistener = EventListener(skills_dict = local_custom_skills_dict, debug = debug) # 实例化EventListener
    aiuinode = AIUINode(eventlistener, debug)
    rospy.on_shutdown(aiuinode.rosShutdownHook)# 设置节点结束时的回调函数
    aiuinode.start()
