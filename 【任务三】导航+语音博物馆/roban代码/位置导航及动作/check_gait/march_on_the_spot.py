#!/usr/bin/env python
# -*- coding: utf-8 -*-.
import rospkg
import bodyhub_action as bodyact
import time
from std_msgs.msg import *
import threading
import yaml
sys.path.append(rospkg.RosPack().get_path('ros_actions_node') + '/scripts/')
from lejulib import *
from motion import bodyhub_client as bodycli
from utils.artag import multiARtag_detecter as ARtag_detecter
from utils.keyboardlistener import keyboardlistener
SCRIPTS_PATH = os.path.split(sys.argv[0])[0]
with open(os.path.join(SCRIPTS_PATH,"config.yaml"),"r")as f:
    CONFIG = yaml.safe_load(f)
    
walk_time = CONFIG["walk_time"]     # 原地踏步持续时间
stand_time = CONFIG["stand_time"]     # 原地站立持续时间
Marker_id = CONFIG["Marker_id"]      # 校准的artag码编号
global_position = CONFIG["global_position"]
use_artag = CONFIG["use_artag"] # 是否使用artag进行校准
adjust_interval = CONFIG["adjust_interval"] # 校准 artag 的间隔时间
artag_alvar_launch_file = "~/robot_ros_application/catkin_ws/src/ar_tag_detect/ar_track_alvar/ar_track_alvar/launch/roban_chin_camera.launch"
class march_on_the_spot(bodyact.Action,ARtag_detecter.MarkerLocation):
    def __init__(self, body_client):
        bodyact.Action.__init__(self, body_client)
        ARtag_detecter.MarkerLocation.__init__(self,body_client)
        self.bodyhub = body_client
        self.start_time = time.time()
        self.config = CONFIG
        

    def march_on_the_spot(self):
        if use_artag:
            self.startArTrack(artag_alvar_launch_file)
        count = 0
        while not rospy.is_shutdown():
            round_time = time.time()
            self.bodyhub_walk()
            self.walk_n_s(walk_time)
            self.pause(stand_time)
            count += 1
            print("完成第{}轮测试, 本轮耗时{}, 总共耗时{}\n{}\n\n".format(count, time.strftime("%H:%M:%S", time.gmtime(time.time() - round_time)), time.strftime("%H:%M:%S", time.gmtime(time.time() - self.start_time)),"----"*10))
        self.stopArTrack()
            
    def walk_n_s(self, duration = 5):
        print("行走{}s".format(duration))
        stime = time.time()
        time_to_complete_adjust = stime
        try:
            while not rospy.is_shutdown() and time.time() - stime < duration:
                self.bodyhub.walking(0, 0, 0)
                if use_artag and time.time() - time_to_complete_adjust > adjust_interval:
                    self.adjust_position()
                    time_to_complete_adjust = time.time()
            self.bodyhub.wait_walking_done()
        except Exception as e:
            rospy.logerr("{}".format(e))
            
    def adjust_position(self,timeout = 5):
        print("调整位置中...")
        stime = time.time()
        while not rospy.is_shutdown():
            if time.time() - stime > timeout:
                raise Exception("Timed out while trying to adjust position")
            current_pos = self.get_chin_marker_pose(Marker_id)
            print(current_pos)
            if current_pos['valid']:
                d_rotation = current_pos['rot'][2] - global_position[2]
                if d_rotation > 20:
                    self.goto_rot(Marker_id,global_position[2])
                self.goto_pose(Marker_id,global_position)
                break
            rospy.sleep(0.2)
                
    def pause(self, duratioon = 5):
        print("站立{}s".format(duratioon))
        rospy.sleep(duratioon)
        
    def ros_on_shutdown(self):
        self.stopArTrack()
        
    def debug(self):
        print("正在启动ar_track_alvar...")
        self.startArTrack(artag_alvar_launch_file)
        self.bodyhub_walk()
        keyboard = keyboardlistener()
        tips = "(按F键标定,q->退出):"
        print("\n开始标定artag位置,请将机器人移动到适当位置:\n{}".format(tips))
        while not rospy.is_shutdown():
            posdata = self.get_chin_marker_pose(Marker_id)
            k = keyboard.getKey(0.1)
            if k.lower() == "q":
                print("退出")
                break
            pos,rot = posdata["pos"],posdata["rot"]
            cur_pos = [pos[0],pos[1],rot[2]]
            sys.stdout.write("\r\033[{}mx:{:8.5f} y:{:8.5f} a:{:8.5f},valid:{}   \033[0m".format(32 if posdata["valid"] else 31,pos[0],pos[1],rot[2], posdata["valid"]))
            sys.stdout.flush()
            if posdata is None or not posdata["valid"]:
                continue
            
            if k == "f":
                self.config["global_position"] = cur_pos
                print("\n标定坐标{}".format(cur_pos))
                print('校准完成，程序退出')
                self.write_config()
                break
        
        self.bodyhub.reset()
        self.stopArTrack()
        rospy.signal_shutdown('exit')
        
    def write_config(self):
        with open(os.path.join(SCRIPTS_PATH,"config.yaml"),"w")as f:
            yaml.dump(self.config,f,encoding="utf-8")
        print("\n写入配置文件成功")
        
if __name__ == '__main__':
    try:
        node_initial()
        obj = march_on_the_spot(bodycli.BodyhubClient(2))
        rospy.on_shutdown(obj.ros_on_shutdown)
        if len(sys.argv)>=2:
            obj.debug()
        else:
            obj.march_on_the_spot()
    except Exception as err:
        serror(err)

